import contextlib
import io
import json
import threading
import time
from types import SimpleNamespace
import unittest

from test_ble_link import KEY
from esp32.ble_protocol import CHUNK_BYTES, RX_UUID, SERVICE_UUID, TX_UUID, Records, Session, pack, proof
from robot.ble_transport import BleTransport, SCAN_SECONDS, _Connection
from robot.client import RobotClient


LABEL = 'RobotArm (mock-device)'
DEVICE = SimpleNamespace(name=None, address='mock-device')


class Peer:
    def __init__(self, device, *, services, disconnected_callback):
        assert device is DEVICE and services == [SERVICE_UUID]
        self.disconnected = disconnected_callback
        self.reader = Records()
        self.session = None
        self.commands = []
        self.nonce = bytes([7]) * 16
        self.held = 1550

    async def connect(self):
        self.thread = threading.current_thread()

    async def disconnect(self):
        pass

    async def start_notify(self, characteristic, callback):
        assert characteristic == TX_UUID
        self.notify = callback

    def reply(self, message):
        for i in range(0, len(message), CHUNK_BYTES):
            self.notify(None, message[i:i + CHUNK_BYTES])

    def frame(self, *fields):
        self.reply(self.session.seal(('@' + json.dumps(fields) + '\n').encode()))

    async def write_gatt_char(self, characteristic, data, response):
        assert characteristic == RX_UUID and response is True and len(data) <= CHUNK_BYTES
        assert threading.current_thread() is self.thread
        for record in self.reader.feed(data):
            if record == b'?':
                self.reply(pack(b'C' + self.nonce))
            elif self.session is None:
                assert record == b'A' + proof(KEY, self.nonce)
                self.session = Session(KEY, self.nonce, central=False)
                self.reply(self.session.seal(b'ok'))
            else:
                line = self.session.open(record).decode().strip()
                self.commands.append(line)
                if line == 'hello2':
                    self.frame('h2', 6)
                    self.frame('cap', 'heartbeat')
                    self.frame('cap', 'park')
                    self.frame('cap', 'sync')
                elif line.startswith('joint '):
                    i = int(line.split()[1])
                    self.frame('j', i, 400, 2500, 1500, self.held)
                    self.frame('n', i, f'Joint {i}', 'Servo')
                    self.frame('v', i, 500, 1500)
                elif line == 'ping':
                    self.frame('hb')
                elif line == 'pose':
                    self.frame('p', *([self.held] * 6))


class BluetoothClientTests(unittest.TestCase):
    def setUp(self):
        self.peers = []
        self.events = []
        self.moments = []
        self.now = 10.0

        async def discover(**kwargs):
            assert kwargs['service_uuids'] == [SERVICE_UUID]
            assert kwargs['timeout'] == SCAN_SECONDS
            self.moments.append(threading.current_thread())
            return {DEVICE.address: (DEVICE, SimpleNamespace(local_name=None,
                                                               service_uuids=[SERVICE_UUID.upper()]))}

        def connect(device, **kwargs):
            peer = Peer(device, **kwargs)
            self.peers.append(peer)
            return peer

        self.worker = BleTransport(scanner=SimpleNamespace(discover=discover), client_factory=connect)
        self.client = RobotClient(self.events.append, ble_factory=lambda: self.worker, clock=lambda: self.now)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.client.shutdown()
        if self.client._ble is None:
            self.worker.shutdown()
        self.worker.thread.join(3)
        self.assertFalse(self.worker.thread.is_alive())

    def until(self, condition):
        with contextlib.redirect_stdout(io.StringIO()):
            deadline = time.monotonic() + 4
            while not condition():
                self.client.poll()
                if time.monotonic() > deadline:
                    self.fail(f'Timed out waiting for BLE event: {self.events[-10:]}')
                time.sleep(.002)

    def open(self):
        self.client.connect_ble(LABEL, KEY.hex())
        self.until(lambda: self.client.ready)

    def test_ble_worker_handshake_reconnect_and_shared_commands(self):
        self.client.scan_ble()
        self.until(lambda: not self.client.scanning)
        self.assertIn({'type': 'scan', 'devices': [LABEL]}, self.events)
        self.assertIs(self.moments[0], self.worker.thread)
        self.open()
        peer = self.peers[-1]
        self.assertIs(peer.thread, self.worker.thread)
        self.assertEqual(peer.commands, ['hello2', *(f'joint {i}' for i in range(1, 7))])
        self.assertEqual(self.events[-1]['current'][1], 1550)
        self.assertTrue(self.client.move_group([1550] * 5, [1600] * 5))
        self.assertTrue(self.client.request_pose())
        self.until(lambda: self.events[-1]['type'] == 'pose')
        self.assertEqual(peer.commands[-2:], ['sync 1550 1550 1550 1550 1550 1600 1600 1600 1600 1600', 'pose'])
        self.now += .6
        self.until(lambda: 'ping' in peer.commands and not self.client._waiting_for_pong)
        peer.disconnected(peer)
        self.until(lambda: not self.client.connected)
        self.open()
        self.assertIsNot(peer, self.peers[-1])
        self.assertEqual(self.events[-1]['current'][1], 1550)
        self.assertEqual(self.peers[-1].commands, ['hello2', *(f'joint {i}' for i in range(1, 7))])

    def test_lost_heartbeat_closes_client_and_invalid_key_does_not_connect(self):
        self.client.scan_ble()
        self.until(lambda: not self.client.scanning)
        self.client.connect_ble(LABEL, 'invalid')
        self.assertFalse(self.client.connected)
        self.assertEqual(self.events[-1]['type'], 'error')
        self.open()
        peer = self.peers[-1]
        original_frame = peer.frame
        peer.frame = lambda *fields: None if fields == ('hb',) else original_frame(*fields)
        self.now += .6
        self.until(lambda: 'ping' in peer.commands)
        self.now += 3
        self.until(lambda: not self.client.connected)
        self.assertIn('heartbeat timed out', self.events[-1]['message'])
        self.assertEqual(peer.commands.count('ping'), 1)

    def test_stop_cancels_unsent_actuator_commands(self):
        link = _Connection()
        for line in (b'move 1 1600\n', b'park\n', b'ping\n', b'pose\n', b'enable 1 1400\n'):
            link.enqueue(line)
        link.enqueue(b'stop\n')
        link.enqueue(b'hold\n')
        self.assertEqual(list(link.outbox), [b'stop\n', b'ping\n', b'pose\n'])


if __name__ == '__main__':
    unittest.main()
