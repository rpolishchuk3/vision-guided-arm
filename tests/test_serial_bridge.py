import builtins
import json
import math
from collections import deque
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace
import unittest


ESP32 = Path(__file__).resolve().parents[1] / 'esp32'
sys.path.insert(0, str(ESP32.parent))
CONFIG = runpy.run_path(str(ESP32 / 'servo_config.py'))
BRIDGE = compile((ESP32 / 'serial_bridge.py').read_bytes(), 'serial_bridge.py', 'exec')
START = (1500, 497, 1500, 1500, 2031)
TARGET = (1800, 760, 1220, 1700, 1940)


class EndSimulation(Exception):
    pass


def firmware_importer(modules):
    def mocked_import(name, *args):
        if name not in modules and name in ('arm_controller', 'arm_service', 'ble_protocol',
                                             'ble_peripheral', 'control_link', 'link_config'):
            module = SimpleNamespace(__name__=name, __builtins__=context)
            modules[name] = module
            file = ESP32 / (name + '.py')
            exec(compile(file.read_bytes(), str(file), 'exec'), vars(module))
        return modules[name] if name in modules else builtins.__import__(name, *args)

    def no_device_files(*_args):
        raise OSError('No board files in simulation')
    context = {**vars(builtins), '__import__': mocked_import, 'open': no_device_files, 'print': lambda *_: None}
    return context


def simulate(commands, start=START, *, heartbeat=True):
    now = 0
    if heartbeat:
        commands = commands + [(n, 'ping') for n in range(1000, 12000, 500)]
    line_queue = deque(sorted(commands))
    chars = deque()
    frames, writes, disabled = [], [], []
    settings = {i: dict(s, speed=170 + i * 50, accel=800 + i * 95)
                for i, s in CONFIG['SERVOS'].items()}

    class MockArm:
        def __init__(self):
            self.current = dict(enumerate((*start, 1900), 1))

        def limits(self, i):
            return settings[i]['min_us'], settings[i]['max_us']

        def settings(self, i):
            return settings[i]

        def _write_pulse(self, i, pulse):
            writes.append((now, i, pulse))
            self.current[i] = pulse

        def stop(self, i):
            disabled.append((now, i))
            self.current[i] = None

        def stop_all(self):
            for i in settings:
                self.stop(i)

    def poll(_timeout):
        while line_queue and line_queue[0][0] <= now:
            chars.extend(line_queue.popleft()[1] + '\n')
        return bool(chars)

    def sleep(ms):
        nonlocal now
        now += ms
        if now >= 12000:
            raise EndSimulation

    modules = {
        'sys': SimpleNamespace(stdin=SimpleNamespace(read=lambda _n: chars.popleft()),
                               stdout=SimpleNamespace(write=lambda data: frames.append((now, json.loads(data[1:]))))),
        'time': SimpleNamespace(ticks_ms=lambda: now, ticks_diff=lambda a, b: a - b,
                                ticks_add=lambda a, b: a + b, sleep_ms=sleep),
        'select': SimpleNamespace(poll=lambda: SimpleNamespace(poll=poll, register=lambda *_: None), POLLIN=1),
        'servo_driver': SimpleNamespace(ServoController=MockArm),
        'servo_config': SimpleNamespace(**CONFIG),
    }

    try:
        exec(BRIDGE, {'__builtins__': firmware_importer(modules)})
    except EndSimulation:
        pass
    return frames, writes, disabled, settings


def sync_command(start=START, target=TARGET):
    return 'sync ' + ' '.join(map(str, (*start, *target)))


class BridgeTests(unittest.TestCase):
    def test_common_progress_respects_each_joint_and_leaves_gripper_alone(self):
        frames, writes, disabled, settings = simulate([(40, sync_command())])
        self.assertIn((20, ['cap', 'sync']), frames)
        duration = max(math.ceil(1000 * max(1.875 * abs(b - a) / settings[i]['speed'],
                                            math.sqrt(5.773503 * abs(b - a) / settings[i]['accel']), 0.1))
                       for i, (a, b) in enumerate(zip(START, TARGET), 1))
        settled = [t for t, f in frames if f[0] == 's' and not f[3]]
        self.assertEqual(len(set(settled)), 1)
        self.assertLessEqual(40 + duration, settled[0])
        self.assertIn((settled[0], ['sync', 'done']), frames)
        self.assertFalse(disabled)
        current = dict(enumerate(START, 1))
        for t, i, pulse in writes:
            progress = min(1, (t - 40) / duration)
            eased = progress**3 * (10 + progress * (-15 + 6 * progress))
            self.assertEqual(pulse, round(START[i - 1] + (TARGET[i - 1] - START[i - 1]) * eased))
            current[i] = pulse
        self.assertEqual(tuple(current.values()), TARGET)

    def test_preflight_rejects_every_axis_atomically(self):
        for requested, targets, actual, reason, motor in (
            (START, TARGET, (*START[:3], None, START[4]), 'init', 4),
            (START, (419, *TARGET[1:]), START, 'limits', 1),
            ((1490, *START[1:]), TARGET, START, 'start', 1),
        ):
            with self.subTest(reason=reason):
                frames, writes, disabled, _ = simulate([(40, sync_command(requested, targets))], actual)
                self.assertIn((40, ['sync', reason, motor]), frames)
                self.assertFalse(writes)
                self.assertFalse(disabled)

    def test_hold_and_stop_interrupt_synchronized_motion(self):
        for command, event in (('hold', 'held'), ('stop', 'stopped')):
            with self.subTest(command=command):
                frames, writes, disabled, _ = simulate([(40, sync_command()), (280, command)])
                self.assertIn((280, ['sync', event]), frames)
                self.assertFalse(any(f == ['sync', 'done'] for _, f in frames))
                self.assertTrue(writes and all(t < 280 for t, _, _ in writes))
                self.assertEqual(len(disabled), 6 if command == 'stop' else 0)

    def test_existing_motion_and_parking(self):
        frames, _, disabled, _ = simulate([(40, 'move 1 1600'), (60, sync_command()), (2400, 'park')])
        self.assertIn((60, ['sync', 'busy']), frames)
        self.assertTrue(any(f == ['s', 1, 1600, False] for _, f in frames))
        settling = next(t for t, f in frames if f == ['park', 'settling'])
        done = next(t for t, f in frames if f == ['park', 'done'])
        self.assertEqual(done - settling, 1000)
        self.assertEqual(disabled, [(done, i) for i in range(1, 7)])

    def test_usb_watchdog_holds_and_requires_a_new_handshake(self):
        frames, writes, disabled, _ = simulate([
            (40, 'move 1 2430'), (100, 'move 1 2300'), (200, 'ping'),
            (500, 'move 1 2350'), (3300, 'move 1 1500'), (3400, 'hello2'), (3500, 'pose'),
        ], heartbeat=False)
        expired = next(t for t, frame in frames if frame == ['link', 'timeout'])
        self.assertEqual(expired, 3200)
        self.assertTrue(writes and all(t < expired for t, _, _ in writes))
        self.assertFalse(disabled)
        self.assertIn((3500, ['p', writes[-1][2], *START[1:], 1900]), frames)

    def test_emergency_supersedes_queued_enables(self):
        frames, writes, disabled, _ = simulate([
            (40, 'enable 1 1500'), (40, 'move 1 1530'), (40, 'stop'),
        ], (None, *START[1:]))
        self.assertFalse(writes)
        self.assertEqual(disabled, [(40, i) for i in range(1, 7)])
        self.assertTrue(any(f[:2] == ['s', 1] for _, f in frames))
        self.assertIn((40, ['cancel', 'stop']), frames)


if __name__ == '__main__':
    unittest.main()
