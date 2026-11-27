"""Run Bleak on a worker loop and deliver events to the GUI through a mailbox."""

import asyncio
from collections import deque
import queue
import threading

from esp32.ble_protocol import (
    CHUNK_BYTES, DEVICE_NAME, NONCE_BYTES, RX_UUID, SERVICE_UUID, TX_UUID,
    Records, Session, pack, proof,
)


HEARTBEAT = b'ping\n'
EMERGENCY = (b'stop\n', b'hold\n')
MAX_QUEUE = 64
SCAN_SECONDS = 10


class _Connection:
    def __init__(self):
        self.records = Records()
        self.inbox = asyncio.Queue(maxsize=MAX_QUEUE)
        self.outbox = deque()
        self.writable = asyncio.Event()
        self.lost = asyncio.Event()
        self.reason = 'BLE link disconnected.'

    def fail(self, reason):
        self.reason = reason
        self.lost.set()
        if not self.inbox.full():
            self.inbox.put_nowait(None)

    def notify(self, data):
        if self.lost.is_set():
            return
        try:
            for record in self.records.feed(data):
                self.inbox.put_nowait(record)
        except (ValueError, asyncio.QueueFull):
            self.fail('BLE receive data was invalid or exceeded the buffer.')

    def enqueue(self, data):
        if self.lost.is_set():
            return
        if data in EMERGENCY:
            # Finish only the record already being written. Cancel queued motion.
            stop_pending = b'stop\n' in self.outbox
            self.outbox = deque(item for item in self.outbox
                                if item.split(None, 1)[0] in (b'hello2', b'joint', b'pose', b'ping'))
            self.outbox.appendleft(b'stop\n' if stop_pending else data)
        elif data != HEARTBEAT or data not in self.outbox:
            if len(self.outbox) >= MAX_QUEUE:
                self.fail('BLE send buffer exceeded. Motion held on link loss.')
                return
            self.outbox.append(data)
        self.writable.set()

    async def read(self):
        record = await self.inbox.get()
        if self.lost.is_set():
            raise ConnectionError(self.reason)
        return record


class BleTransport:
    """Use one asyncio loop for scanning, connecting, and all GATT operations."""

    def __init__(self, *, scanner=None, client_factory=None):
        self.scanner = scanner
        self.client_factory = client_factory
        self.events = queue.Queue(maxsize=128)
        self.overflow = threading.Event()
        self.loop = asyncio.new_event_loop()
        self._devices = {}
        self._active = None
        self._connect_lock = asyncio.Lock()
        self._job = None
        self._scan_job = None
        self.thread = threading.Thread(target=self._run, name='RobotArm BLE', daemon=True)
        self.thread.start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        tasks = asyncio.all_tasks(self.loop)
        for task in tasks:
            task.cancel()
        if tasks:
            self.loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        self.loop.close()

    def _post(self, kind, token, value):
        try:
            self.events.put_nowait((kind, token, value))
            return True
        except queue.Full:
            self.overflow.set()
            return False

    def scan(self, token):
        if self._scan_job is not None and not self._scan_job.done():
            self._scan_job.cancel()
        self._scan_job = asyncio.run_coroutine_threadsafe(self._scan(token), self.loop)

    async def _scan(self, token):
        try:
            scanner = self.scanner
            if scanner is None:
                from bleak import BleakScanner
                scanner = BleakScanner
            found = await scanner.discover(timeout=SCAN_SECONDS, return_adv=True, service_uuids=[SERVICE_UUID])
            devices = {}
            for device, advertisement in found.values():
                name = advertisement.local_name or device.name
                matches_service = any(uuid.lower() == SERVICE_UUID for uuid in advertisement.service_uuids)
                if name != DEVICE_NAME and not matches_service:
                    continue
                label = '{} ({})'.format(DEVICE_NAME, device.address)
                devices[label] = device
            self._devices = devices
            self._post('scan', token, sorted(devices))
        except ImportError:
            self._post('scan_error', token, 'Install Bleak to use Bluetooth LE: python -m pip install bleak')
        except Exception as exc:
            self._post('scan_error', token, 'Bluetooth scan failed: ' + str(exc))

    def connect(self, label, key, token):
        self.disconnect()
        self._job = asyncio.run_coroutine_threadsafe(self._connect(label, key, token), self.loop)

    def disconnect(self):
        if self._job is not None and not self._job.done():
            self._job.cancel()

    def shutdown(self):
        self.disconnect()
        self.loop.call_soon_threadsafe(self.loop.stop)

    def send(self, token, data):
        self.loop.call_soon_threadsafe(self._send, token, data)

    def _send(self, token, data):
        if self._active is not None and self._active[0] == token:
            self._active[1].enqueue(data)

    @staticmethod
    async def _write(client, record):
        for offset in range(0, len(record), CHUNK_BYTES):
            await asyncio.wait_for(client.write_gatt_char(
                RX_UUID, record[offset:offset + CHUNK_BYTES], response=True), 2)

    async def _authenticate(self, client, link, key):
        await client.start_notify(TX_UUID, lambda _char, data: self.loop.call_soon_threadsafe(
            link.notify, bytes(data)))
        await self._write(client, pack(b'?'))
        challenge = await link.read()
        if challenge[:1] != b'C' or len(challenge) != NONCE_BYTES + 1:
            raise ValueError('Unexpected RobotArm authentication challenge.')
        nonce = challenge[1:]
        channel = Session(key, nonce, central=True)
        await self._write(client, pack(b'A' + proof(key, nonce)))
        reply = await link.read()
        if reply == b'!':
            raise ValueError('RobotArm rejected the Bluetooth key.')
        status = channel.open(reply)
        if status != b'ok':
            raise ValueError('RobotArm is already controlled by another client.')
        return channel

    async def _read_frames(self, link, channel, token):
        while True:
            payload = channel.open(await link.read())
            if not payload.startswith(b'@') or not payload.endswith(b'\n') or b'\n' in payload[:-1]:
                raise ValueError('Invalid response line from RobotArm.')
            if not self._post('data', token, payload):
                raise ConnectionError('Bluetooth response backlog exceeded.')

    async def _write_frames(self, client, link, channel):
        while True:
            await link.writable.wait()
            while link.outbox:
                await self._write(client, channel.seal(link.outbox.popleft()))
            link.writable.clear()

    async def _connect(self, label, key, token):
        async with self._connect_lock:
            client = None
            tasks = []
            message = 'Bluetooth LE disconnected; the last PWM may remain active.'
            try:
                client_factory = self.client_factory
                if client_factory is None:
                    from bleak import BleakClient
                    client_factory = BleakClient
                device = self._devices[label]
                link = _Connection()
                client = client_factory(device, services=[SERVICE_UUID], disconnected_callback=lambda _client:
                                        self.loop.call_soon_threadsafe(link.fail, 'RobotArm BLE link lost.'))
                await asyncio.wait_for(client.connect(), 15)
                channel = await asyncio.wait_for(self._authenticate(client, link, key), 7)
                self._active = (token, link)
                if not self._post('connected', token, None):
                    raise ConnectionError('Bluetooth response backlog exceeded.')
                tasks = [asyncio.create_task(self._read_frames(link, channel, token)),
                         asyncio.create_task(self._write_frames(client, link, channel)),
                         asyncio.create_task(link.lost.wait())]
                finished, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in finished:
                    task.result()
                message = link.reason
            except ImportError:
                message = 'Install Bleak to use Bluetooth LE: python -m pip install bleak'
            except KeyError:
                message = 'Scan for RobotArm again before connecting.'
            except asyncio.TimeoutError:
                message = 'Bluetooth operation timed out.'
            except asyncio.CancelledError:
                pass
            except Exception as exc:
                message = 'Bluetooth LE: ' + str(exc)
            finally:
                self._active = None
                for task in tasks:
                    task.cancel()
                if tasks:
                    await asyncio.gather(*tasks, return_exceptions=True)
                if client is not None:
                    try:
                        await asyncio.wait_for(client.disconnect(), 2)
                    except Exception:
                        pass
                self._post('closed', token, message)
