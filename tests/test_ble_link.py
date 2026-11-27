import hmac
import json
import random
from types import SimpleNamespace
import unittest
from uuid import UUID

from test_serial_bridge import CONFIG, START, firmware_importer
from esp32.ble_protocol import CHUNK_BYTES, Records, Session, hmac_sha256, pack, proof


KEY = bytes(range(32))
WRONG_KEY = bytes(reversed(range(32)))


class ProtocolTests(unittest.TestCase):
    def test_fragmentation_and_coalescing_keep_order(self):
        nonce = bytes(16)
        tx, rx = Session(KEY, nonce, central=True), Session(KEY, nonce, central=False)
        lines = [b'hello2\n', b'sync 1500 497 1500 1500 2031 1800 760 1220 1700 1940\n',
                 b'pose\n', b'hold\n', b'stop\n']
        wire = b''.join(tx.seal(line) for line in lines)
        decoder = Records()
        received = []
        randomizer = random.Random(55)
        while wire:
            take = randomizer.randint(1, CHUNK_BYTES * 2)
            received.extend(rx.open(record) for record in decoder.feed(wire[:take]))
            wire = wire[take:]
        self.assertEqual(received, lines)
        self.assertFalse(decoder.buffer)

    def test_authentication_matches_hmac_and_rejects_tampering_replay_and_gaps(self):
        self.assertEqual(hmac_sha256(KEY, b'message'), hmac.digest(KEY, b'message', 'sha256'))
        tx = Session(KEY, bytes(16), central=True)
        first, second = tx.seal(b'enable 1 1700\n')[1:], tx.seal(b'move 1 1720\n')[1:]
        rx = Session(KEY, bytes(16), central=False)
        for invalid in (first[:-1], first[:6] + b'X' + first[7:], second):
            with self.assertRaises(ValueError):
                rx.open(invalid)
        self.assertEqual(rx.open(first), b'enable 1 1700\n')
        with self.assertRaises(ValueError):
            rx.open(first)
        with self.assertRaises(ValueError):
            Session(KEY, bytes([9]) * 16, central=False).open(first)
        with self.assertRaises(ValueError):
            Session(WRONG_KEY, bytes(16), central=False).open(first)
        with self.assertRaises(ValueError):
            Records().feed(b'\xff')


class Radio:
    def __init__(self):
        self.rx = bytearray()
        self.tx = []
        self.rejected = []

    def active(self, _active):
        pass

    def config(self, *, gap_name):
        # ESP32 MicroPython builds can omit the optional rxbuf setting.
        assert gap_name == 'RobotArm'

    def gatts_register_services(self, services):
        self.services = services
        return ((11, 12),)

    def gatts_set_buffer(self, handle, size, append):
        assert handle == 11 and size >= CHUNK_BYTES and append

    def irq(self, callback):
        self.callback = callback

    def gap_advertise(self, _period, adv_data):
        self.advertisement = adv_data

    def gatts_read(self, _handle):
        data = bytes(self.rx)
        self.rx.clear()
        return data

    def gatts_notify(self, conn, handle, data):
        assert conn == self.connection and handle == 12 and len(data) <= CHUNK_BYTES
        self.tx.append(data)

    def gap_disconnect(self, conn):
        self.rejected.append(conn)
        self.callback(2, (conn, 0, None))
        return True

    def arrive(self, conn):
        self.connection = conn
        self.callback(1, (conn, 0, None))

    def write(self, chunk):
        self.rx.extend(chunk)
        self.callback(3, (self.connection, 11))


class FakeArm:
    def __init__(self):
        self.current = dict(enumerate((*START, None), 1))
        self.writes = []
        self.stops = []

    def settings(self, i):
        return CONFIG['SERVOS'][i]

    def limits(self, i):
        s = self.settings(i)
        return s['min_us'], s['max_us']

    def _write_pulse(self, i, pulse):
        self.current[i] = pulse
        self.writes.append((i, pulse))

    def stop(self, i):
        self.current[i] = None
        self.stops.append(i)

    def stop_all(self):
        for i in self.current:
            self.stop(i)


class BleBoardTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.radio = Radio()
        self.arm = FakeArm()
        self.nonce_count = 0

        def urandom(size):
            self.nonce_count += 1
            return bytes([self.nonce_count]) * size

        modules = {
            'time': SimpleNamespace(ticks_ms=lambda: self.now, ticks_diff=lambda a, b: a - b,
                                    ticks_add=lambda a, b: a + b),
            'bluetooth': SimpleNamespace(BLE=lambda: self.radio, UUID=lambda value: UUID(value).bytes[::-1],
                                         FLAG_WRITE=8, FLAG_NOTIFY=16),
            'servo_config': SimpleNamespace(**CONFIG),
            'os': SimpleNamespace(urandom=urandom),
        }
        load = firmware_importer(modules)['__import__']
        ControlLink = load('control_link').ControlLink
        BlePeripheral = load('ble_peripheral').BlePeripheral
        self.control = ControlLink(self.arm, watchdog_ms=3000)
        self.ble = BlePeripheral(self.control, KEY, 15000)
        self.assertEqual(len(self.radio.advertisement), 31)
        self.assertIn(b'RobotArm', self.radio.advertisement)
        self.usb_frames = []
        self.usb = SimpleNamespace(send=lambda data, _: self.usb_frames.append(json.loads(data[1:])),
                                   expired=lambda: None)

    def advance(self, ms):
        for _ in range(0, ms, 2):
            self.now += 2
            self.control.expire(self.now)
            self.ble.poll(self.now)
            self.control.motion.update(self.now)

    def drain(self):
        records = self.decoder.feed(b''.join(self.radio.tx))
        self.radio.tx.clear()
        return records

    def send(self, wire):
        for start in range(0, len(wire), CHUNK_BYTES):
            self.radio.write(wire[start:start + CHUNK_BYTES])
        self.advance(20)

    def authenticate(self, *, key=KEY, conn=1):
        self.decoder = Records()
        self.radio.arrive(conn)
        self.advance(2)
        self.send(pack(b'?'))
        challenge, = self.drain()
        nonce = challenge[1:]
        self.central = Session(key, nonce, central=True)
        self.send(pack(b'A' + proof(key, nonce)))
        reply, = self.drain()
        return reply if reply == b'!' else self.central.open(reply)

    def command(self, *lines):
        self.send(b''.join(self.central.seal(line.encode() + b'\n') for line in lines))
        self.advance(40)
        return [json.loads(self.central.open(record)[1:]) for record in self.drain()]

    def test_wrong_key_and_unsigned_commands_cannot_control(self):
        self.assertEqual(self.authenticate(key=WRONG_KEY), b'!')
        self.assertIsNone(self.control.owner)
        self.advance(300)
        self.radio.arrive(2)
        self.advance(2)
        self.send(pack(b'enable 6 1900\n'))
        self.assertIsNone(self.control.owner)
        self.assertFalse(self.arm.writes)
        self.assertIsNone(self.arm.current[6])

    def test_authenticated_transport_uses_same_commands_and_has_one_owner(self):
        self.control.receive(self.usb, ['hello2'], self.now, usb=True)
        self.assertEqual(self.authenticate(), b'busy')
        self.assertIs(self.control.owner, self.usb)
        self.control.receive(self.usb, ['bye'], self.now, usb=True)
        self.advance(300)
        self.assertEqual(self.authenticate(conn=2), b'ok')
        self.assertIn(['h2', 6], self.command('hello2'))
        self.control.receive(self.usb, ['hello2', 'enable 6 1850'], self.now, usb=True)
        self.assertEqual(self.usb_frames[-1], ['link', 'busy'])
        self.assertIsNone(self.arm.current[6])
        self.assertIn(['s', 6, 1800, False], self.command('enable 6 1800'))
        self.assertIn(['hb'], self.command('ping'))
        self.assertIn(['p', *START, 1800], self.command('pose'))
        self.assertIn(['s', 6, None, False], self.command('off 6'))

    def test_disconnect_discards_park_and_pending_and_reconnect_reports_held_pwm(self):
        self.assertEqual(self.authenticate(), b'ok')
        self.command('ping', 'enable 6 1900', 'move 1 2400', 'move 1 2200', 'park')
        self.assertTrue(self.control.motion.active)
        self.assertTrue(self.control.motion.pending)
        self.assertTrue(self.control.motion.parking)
        held = tuple(self.arm.current.values())
        self.radio.callback(2, (1, 0, None))
        self.advance(200)
        self.assertEqual(tuple(self.arm.current.values()), held)
        self.assertFalse(self.arm.stops or self.control.motion.active or self.control.motion.pending)
        self.assertFalse(self.control.motion.parking)
        self.assertEqual(self.authenticate(conn=2), b'ok')
        self.assertIn(['p', *held], self.command('ping', 'pose'))
        self.advance(300)
        self.assertEqual(tuple(self.arm.current.values()), held)

    def test_watchdog_holds_even_if_non_heartbeat_packets_continue(self):
        self.assertEqual(self.authenticate(), b'ok')
        self.command('ping', 'move 1 2430')
        self.advance(2000)
        self.command('move 1 2200')
        self.advance(1100)
        self.assertIsNone(self.control.owner)
        self.assertFalse(self.control.motion.active or self.control.motion.pending or self.arm.stops)
        self.assertIn(1, self.radio.rejected)
        last = tuple(self.arm.current.values())
        self.advance(600)
        self.assertEqual(tuple(self.arm.current.values()), last)

    def test_watchdog_cancels_park_during_its_shutdown_delay(self):
        self.arm.current.update(CONFIG['PARK_POSITION'])
        self.control.watchdog_ms = 500
        self.assertEqual(self.authenticate(), b'ok')
        self.assertIn(['park', 'settling'], self.command('ping', 'park'))
        self.advance(1500)
        self.assertFalse(self.arm.stops)
        self.assertEqual(self.arm.current, CONFIG['PARK_POSITION'])
        self.assertIsNone(self.control.owner)

    def test_corruption_holds_before_any_following_command(self):
        self.assertEqual(self.authenticate(), b'ok')
        self.command('ping', 'move 1 2430')
        before = tuple(self.arm.current.values())
        bad = self.central.seal(b'enable 6 1700\n')
        self.send(bad[:-1] + bytes([bad[-1] ^ 1]) + self.central.seal(b'stop\n'))
        self.assertIsNone(self.control.owner)
        self.assertEqual(tuple(self.arm.current.values()), before)
        self.assertFalse(self.arm.stops)


if __name__ == '__main__':
    unittest.main()
