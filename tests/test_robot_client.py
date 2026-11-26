import contextlib
import io
import json
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot.client import RobotClient


class SerialStub:
    def __init__(self):
        self.incoming = bytearray()
        self.written = []
        self.is_open = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, count):
        data = bytes(self.incoming[:count])
        del self.incoming[:count]
        return data

    def write(self, data):
        self.written.append(data)

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.port = SerialStub()
        self.now = 10.0
        self.client = RobotClient(self.events.append, serial_factory=lambda **_: self.port, clock=lambda: self.now)
        self.client.connect('COM10')

    def feed(self, *packets):
        for packet in packets:
            self.port.incoming.extend(b'@' + json.dumps(packet).encode() + b'\r\n')
        with contextlib.redirect_stdout(io.StringIO()):
            self.client.poll()

    def handshake(self, *capabilities):
        self.feed(['h2', 6], *(['cap', name] for name in capabilities))
        for i in range(1, 7):
            self.feed(['j', i, 400, 2500, 1500, None], ['n', i, f'Joint {i}', 'Servo'], ['v', i, 500, 1500])

    def test_collects_joint_limits_and_capabilities_without_moving(self):
        self.now += .4
        self.client.poll()
        self.assertEqual(self.port.written, [b'hello2\r'])
        self.assertFalse(self.client.move(1, 1500))
        self.handshake('park', 'sync')
        self.assertTrue(self.client.ready)
        self.assertEqual(self.client.capabilities, {'park', 'sync'})
        self.assertEqual(self.port.written[1:], [f'joint {i}\n'.encode() for i in range(1, 7)])
        hello = self.events[-1]
        self.assertEqual(hello['type'], 'hello')
        self.assertEqual(hello['motors'][3]['min_us'], 400)
        self.assertEqual(hello['current'], dict.fromkeys(range(1, 7)))

    def test_routes_shared_commands_and_replies(self):
        self.handshake('park', 'sync')
        self.port.written.clear()
        self.assertTrue(self.client.enable(2, 1500))
        self.assertTrue(self.client.move(2, 1550))
        self.assertTrue(self.client.disable(2))
        self.assertTrue(self.client.request_pose())
        self.assertTrue(self.client.park())
        self.assertTrue(self.client.move_group([1500] * 5, [1600] * 5))
        self.feed(['s', 2, 1550, False], ['p', 1500, 1550, 1500, 1500, 1500, None], ['sync', 'done'])
        self.assertEqual(self.events[-3], dict(type='state', motor=2, pulse=1550, busy=False))
        self.assertEqual(self.events[-2]['current'][6], None)
        self.assertEqual(self.events[-1], dict(type='sync', stage='done', motors=[]))
        self.client.disconnect()
        self.assertEqual(self.port.written, [
            b'enable 2 1500\n', b'move 2 1550\n', b'off 2\n', b'pose\n', b'park\n',
            b'sync 1500 1500 1500 1500 1500 1600 1600 1600 1600 1600\n', b'hold\n',
        ])
        self.assertFalse(self.port.is_open)
        self.assertFalse(self.client.ready)
        self.assertFalse(self.client.capabilities)

    def test_bootstrap_and_old_bridge_keep_manual_control_available(self):
        self.port.incoming.extend(b'>>> ')
        self.client.poll()
        self.assertEqual(len(self.port.written), 1)
        self.assertIn(b"'/robot_arm'", self.port.written[0])
        self.assertIn(b"'slider_control_v2.py'", self.port.written[0])
        self.handshake()
        self.assertFalse(self.client.park())
        self.assertFalse(self.client.move_group([1500] * 5, [1600] * 5))
        self.assertTrue(self.client.hold())
        self.assertTrue(self.client.stop())
        self.assertEqual(self.port.written[-2:], [b'hold\n', b'stop\n'])


if __name__ == '__main__':
    unittest.main()
