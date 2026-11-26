"""Polling desktop client for the MicroPython arm bridge."""

from collections import deque
import json
import time

import serial
from serial.tools import list_ports

from .joint_calibration import PWM_LIMITS_US


JOINT_COUNT = 6
BRIDGE_START = (
    "import os, sys; sys.path.insert(0, '/robot_arm'); "
    "bridge = 'serial_bridge.py' if 'serial_bridge.py' in os.listdir('/robot_arm') "
    "else 'slider_control_v2.py'; exec(open('/robot_arm/' + bridge).read())"
)


class RobotClient:
    """Notify on_event with connection, joint, pose, and motion events. Call poll periodically."""

    def __init__(self, on_event, *, serial_factory=serial.Serial, clock=time.monotonic):
        self.on_event = on_event
        self.serial_factory = serial_factory
        self.clock = clock
        self._serial = None
        self._buffer = bytearray()
        self.ready = False
        self.capabilities = set()
        self.log = deque(maxlen=500)
        self._phase = None
        self._phase_since = self._deadline = self._last_request = 0
        self._joint = 1
        self._parts = {}
        self._traceback = []
        self._in_traceback = False

    @staticmethod
    def ports():
        return [port.device for port in list_ports.comports()]

    @property
    def connected(self):
        return self._serial is not None

    def _emit(self, kind, **data):
        self.on_event(dict(type=kind, **data))

    def _status(self, message):
        self._emit('status', message=message)

    def connect(self, port):
        if self.connected:
            return
        try:
            link = self.serial_factory(port=None, baudrate=115200, timeout=0, write_timeout=.25)
            link.port = port.strip()
            link.dtr = False
            link.rts = False
            link.open()
        except (serial.SerialException, OSError) as exc:
            self._emit('error', message=str(exc))
            return
        self._serial = link
        self._buffer.clear()
        self.log.clear()
        self.capabilities.clear()
        self._parts = {}
        self._joint = 1
        self._traceback.clear()
        self._in_traceback = False
        self._last_request = 0
        self._phase = 'probe'
        self._phase_since = self.clock()
        self._deadline = self._phase_since + 30
        self._emit('connected')
        self._status('Waiting for the running ESP32 controller...')

    def disconnect(self, reason='Disconnected; last PWM may remain active.'):
        link = self._serial
        if link is not None:
            try:
                if self.ready:
                    link.write(b'hold\n')
            except (serial.SerialException, OSError):
                pass
            try:
                link.close()
            except (serial.SerialException, OSError):
                pass
        self._serial = None
        self.ready = False
        self._phase = None
        self.capabilities.clear()
        self._emit('disconnected', message=reason)

    def _write(self, data):
        if not self.connected:
            return False
        try:
            self._serial.write(data)
        except (serial.SerialException, OSError) as exc:
            self.disconnect(str(exc))
            return False
        return True

    def _send(self, *parts):
        return self.ready and self._write((' '.join(map(str, parts)) + '\n').encode())

    def enable(self, motor, pulse):
        return self._send('enable', motor, pulse)

    def move(self, motor, pulse):
        return self._send('move', motor, pulse)

    def disable(self, motor):
        return self._send('off', motor)

    def hold(self):
        return self._send('hold')

    def stop(self):
        return self._send('stop')

    def park(self):
        return 'park' in self.capabilities and self._send('park')

    def request_pose(self):
        return self._send('pose')

    def move_group(self, start, target):
        return 'sync' in self.capabilities and self._send('sync', *start, *target)

    def poll(self):
        if not self.connected:
            return
        try:
            now = self.clock()
            self._buffer.extend(self._serial.read(min(8192, self._serial.in_waiting)))
            prompt = self._read_lines() or self._buffer.strip().endswith(b'>>>')
            if len(self._buffer) > 32768:
                self._buffer.clear()
            if self.connected and not self.ready:
                self._advance_handshake(now, prompt)
        except (serial.SerialException, OSError) as exc:
            self.disconnect(str(exc))

    def _read_lines(self):
        prompt = False
        while b'\n' in self._buffer:
            raw, _, rest = self._buffer.partition(b'\n')
            self._buffer = bytearray(rest)
            line = raw.decode(errors='replace').strip()
            if not line:
                continue
            self.log.append(line)
            print('[ESP32]', line, flush=True)
            prompt |= line == '>>>'
            if line.startswith(('@', '>>> @')):
                try:
                    self._frame(json.loads(line[line.index('@') + 1:]))
                except (ValueError, KeyError, TypeError, IndexError) as exc:
                    self._status('Protocol error: ' + str(exc))
            elif 'Traceback' in line:
                self._in_traceback = True
                self._traceback = [line]
            elif self._in_traceback:
                if line.startswith('>>>'):
                    self._in_traceback = False
                else:
                    self._traceback.append(line)
                    self._status(line)
        return prompt

    def _advance_handshake(self, now, prompt):
        elapsed = now - self._phase_since
        phase = self._phase
        if phase in ('probe', 'interrupt', 'wait_prompt') and prompt:
            self._phase = 'launch'
            self._phase_since = self.clock()
            self._traceback.clear()
            self._in_traceback = False
            self._buffer.clear()
            self._status('MicroPython REPL detected; starting the serial bridge...')
            self._write((BRIDGE_START + '\r').encode())
        elif phase == 'probe' and elapsed >= 3:
            self._phase = 'interrupt'
            self._phase_since = now
            self._status('No bridge response; requesting MicroPython REPL...')
            self._write(b'\x03\x03\r')
        elif phase == 'interrupt' and elapsed >= 2:
            self._phase = 'wait_prompt'
            self._write(b'\x03\r')
        elif phase == 'collect' and now - self._last_request >= .8:
            self._request_joint()
        elif phase == 'launch' and prompt and self._traceback:
            self.disconnect('ESP32 startup failed: ' + self._traceback[-1] + ' | Open Serial log')
        elif ((phase == 'probe' and elapsed >= .3) or
              (phase == 'launch' and not prompt and elapsed >= 1)):
            if now - self._last_request >= 1:
                self._last_request = now
                self._write(b'hello2\r')
        if self.connected and not self.ready and now > self._deadline:
            if self._phase == 'collect':
                detail = f'Incomplete joint settings: motor {self._joint}'
            else:
                detail = self._traceback[-1] if self._traceback else 'No valid short-frame response from MicroPython'
            self.disconnect('No handshake: ' + detail + ' | Open Serial log')

    def _request_joint(self):
        self._last_request = self.clock()
        self._write(f'joint {self._joint}\n'.encode())

    def _frame(self, packet):
        if not isinstance(packet, list) or not packet:
            return
        kind = packet[0]
        if kind == 'h2' and len(packet) == 2:
            if self.ready or self._phase == 'collect':
                return
            if packet[1] != JOINT_COUNT:
                raise ValueError('Expected six joints')
            self._parts = {}
            self._joint = 1
            self._phase = 'collect'
            self._deadline = self.clock() + 25
            self._status('Reading joint 1/6...')
            self._request_joint()
        elif kind in ('j', 'n', 'v') and self._phase == 'collect':
            self._collect_joint(kind, packet)
        elif kind == 'cap' and len(packet) == 2 and isinstance(packet[1], str):
            self.capabilities.add(packet[1])
            self._emit('capability', name=packet[1])
        elif kind in ('park', 'sync') and self.ready and len(packet) >= 2:
            self._emit(kind, stage=packet[1], motors=packet[2:])
        elif kind == 's' and self.ready and len(packet) == 4:
            self._emit('state', motor=packet[1], pulse=packet[2], busy=packet[3])
        elif kind == 'p' and self.ready and len(packet) == JOINT_COUNT + 1:
            self._emit('pose', current=dict(enumerate(packet[1:], 1)))
        elif kind in ('e', 'i') and len(packet) == 2:
            self._emit('error' if kind == 'e' else 'info', message=packet[1])

    def _collect_joint(self, kind, packet):
        if len(packet) < 2 or packet[1] != self._joint:
            return
        record = self._parts.setdefault(self._joint, {})
        if kind == 'j' and len(packet) == 6:
            _, _, lo, hi, center, current = packet
            if (not all(type(v) is int for v in (lo, hi, center)) or
                    not PWM_LIMITS_US[0] <= lo < hi <= PWM_LIMITS_US[1] or not lo <= center <= hi):
                raise ValueError('Invalid joint limits')
            if current is not None and (type(current) is not int or not lo <= current <= hi):
                raise ValueError('Invalid current pulse')
            record['j'] = (lo, hi, center, current)
        elif kind == 'n' and len(packet) == 4:
            if not all(isinstance(v, str) for v in packet[2:]):
                raise ValueError('Invalid joint name')
            record['n'] = packet[2:]
        elif kind == 'v' and len(packet) == 4:
            if not all(isinstance(v, (int, float)) and v > 0 for v in packet[2:]):
                raise ValueError('Invalid trajectory settings')
            record['v'] = packet[2:]
        if not all(k in record for k in ('j', 'n', 'v')):
            return
        if self._joint < JOINT_COUNT:
            self._joint += 1
            self._status(f'Reading joint {self._joint}/6...')
            self._request_joint()
            return
        motors, current = {}, {}
        for motor, joint in self._parts.items():
            lo, hi, center, current[motor] = joint['j']
            name, model = joint['n']
            speed, accel = joint['v']
            motors[motor] = dict(min_us=lo, max_us=hi, center_us=center, name=name, model=model,
                                 speed=speed, accel=accel)
        self.ready = True
        self._phase = None
        self._in_traceback = False
        self._emit('hello', motors=motors, current=current)
