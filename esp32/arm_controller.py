"""Transport-independent commands and nonblocking servo trajectories."""
import math
import time

import servo_config as cfg


POSITION_AXES = range(1, 6)


class ArmController:
    def __init__(self, arm, frame):
        self.arm = arm
        self.frame = frame
        self.active = {}
        self.pending = {}
        self.next_update = time.ticks_ms()
        self.parking = False
        self.park_deadline = None
        self.syncing = False

    def cancel_all(self):
        """Discard all movement and keep the last PWM command on each output."""
        self.active.clear()
        self.pending.clear()
        self.parking = False
        self.park_deadline = None
        self.syncing = False

    def emit(self, kind, **data):
        if kind == 'state':
            self.frame('s', data['motor'], data['pulse'], data['busy'])
        elif kind == 'pose':
            self.frame('p', *[data['current'][str(k)] for k in range(1, 7)])
        else:
            # Human-readable diagnostics may exceed one USB packet. Split them.
            message = str(data.get('message', ''))
            for pos in range(0, len(message), 30):
                self.frame('e' if kind == 'error' else 'i', message[pos:pos+30])

    def hello(self):
        self.frame('h2', len(cfg.SERVOS))
        self.frame('cap', 'park')
        self.frame('cap', 'sync')
        self.frame('cap', 'heartbeat')

    def state(self, motor):
        self.emit('state', motor=motor, pulse=self.arm.current[motor],
             busy=(motor in self.active or motor in self.pending))

    def duration_ms(self, motor, target):
        servo = self.arm.settings(motor)
        d = abs(target - self.arm.current[motor])
        duration = max(1.875 * d / servo['speed'],
                       math.sqrt(5.773503 * d / servo['accel']), 0.1)
        return math.ceil(duration * 1000)

    def begin(self, motor, target, now, duration=None):
        start = self.arm.current[motor]
        if start != target:
            if duration is None:
                duration = self.duration_ms(motor, target)
            self.active[motor] = (start, target, now, duration)
        self.state(motor)

    def cancel(self, motor):
        self.active.pop(motor, None)
        self.pending.pop(motor, None)

    def move(self, motor, target, now):
        if motor not in self.active:
            self.begin(motor, target, now)
        elif target == self.active[motor][1]:
            self.pending.pop(motor, None)
        else:
            # Latest target wins. Finish the current smooth segment first.
            self.pending[motor] = target

    def start_park(self):
        if self.parking:
            return
        if self.syncing:
            self.frame('park', 'busy')
            return
        targets = getattr(cfg, 'PARK_POSITION', None)
        if not isinstance(targets, dict):
            self.frame('park', 'invalid', *cfg.SERVOS)
            return
        uninitialized = []
        invalid = []
        for motor in cfg.SERVOS:
            lo, hi = self.arm.limits(motor)
            target = targets.get(motor)
            start = self.arm.current[motor]
            if start is None:
                uninitialized.append(motor)
            if (type(target) is not int or not lo <= target <= hi or
                    (start is not None and not lo <= start <= hi)):
                invalid.append(motor)
        if uninitialized:
            self.frame('park', 'init', *uninitialized)
            return
        if invalid:
            self.frame('park', 'invalid', *invalid)
            return
        self.pending.clear()
        self.parking = True
        self.park_deadline = None
        self.frame('park', 'moving')
        now = time.ticks_ms()
        for motor in cfg.SERVOS:
            self.move(motor, targets[motor], now)

    def start_sync(self, starts, targets):
        if self.parking or self.syncing or self.active or self.pending:
            self.frame('sync', 'busy')
            return
        uninitialized, invalid, stale = [], [], []
        for i, start, target in zip(POSITION_AXES, starts, targets):
            lo, hi = self.arm.limits(i)
            current = self.arm.current[i]
            if current is None:
                uninitialized.append(i)
            if not lo <= start <= hi or not lo <= target <= hi:
                invalid.append(i)
            if start != current:
                stale.append(i)
        for reason, motors in (('init', uninitialized), ('limits', invalid), ('start', stale)):
            if motors:
                for i in motors:
                    self.state(i)
                self.frame('sync', reason, *motors)
                return
        duration = max(self.duration_ms(i, target) for i, target in zip(POSITION_AXES, targets))
        self.syncing = True
        self.frame('sync', 'moving')
        now = time.ticks_ms()
        for i, target in zip(POSITION_AXES, targets):
            self.begin(i, target, now, duration)

    def command(self, line):
        parts = line.split()
        if not parts:
            return
        op = parts[0].lower()
        if op == 'hello2' and len(parts) == 1:
            self.hello()
            return
        if op == 'joint' and len(parts) == 2:
            motor = int(parts[1])
            servo = self.arm.settings(motor)
            self.frame('j', motor, servo['min_us'], servo['max_us'],
                  servo['center_us'], self.arm.current[motor])
            self.frame('n', motor, servo['name'], servo['model'])
            self.frame('v', motor, servo['speed'], servo['accel'])
            return
        if op == 'pose' and len(parts) == 1:
            self.emit('pose', current={str(k): v for k, v in self.arm.current.items()})
            return
        if op == 'park' and len(parts) == 1:
            self.start_park()
            return
        if op == 'sync' and len(parts) == 11:
            self.start_sync(tuple(int(x) for x in parts[1:6]), tuple(int(x) for x in parts[6:]))
            return
        if op in ('hold', 'stop') and len(parts) == 1:
            interrupted = self.parking
            interrupted_sync = self.syncing
            self.cancel_all()
            for motor in cfg.SERVOS:
                if op == 'stop':
                    self.arm.stop(motor)
                self.state(motor)
            if interrupted:
                self.frame('park', 'stopped' if op == 'stop' else 'held')
            elif interrupted_sync:
                self.frame('sync', 'stopped' if op == 'stop' else 'held')
            else:
                self.emit('info', message='Motion cancelled; PWM ' + ('disabled' if op == 'stop' else 'held'))
            return
        if op not in ('enable', 'move', 'off') or len(parts) != (2 if op == 'off' else 3):
            raise ValueError('Invalid command')
        if self.parking or self.syncing:
            raise ValueError('Use Hold or Stop to cancel motion')
        motor = int(parts[1])
        lo, hi = self.arm.limits(motor)
        if op == 'off':
            self.cancel(motor)
            self.arm.stop(motor)
            self.state(motor)
            return
        target = int(parts[2])
        if not lo <= target <= hi:
            raise ValueError('Motor {}: allowed {}-{} us'.format(motor, lo, hi))
        if op == 'enable':
            if self.arm.current[motor] is not None:
                raise ValueError('Already enabled')
            # An unpowered/uncommanded servo's real angle is unknown.
            self.arm._write_pulse(motor, target)
            self.state(motor)
        else:
            if self.arm.current[motor] is None:
                raise ValueError('Enable motor first')
            self.move(motor, target, time.ticks_ms())

    def update(self, now):
        if time.ticks_diff(now, self.next_update) >= 0:
            self.next_update = time.ticks_add(now, cfg.UPDATE_MS)
            for motor in list(self.active):
                start, target, started, duration = self.active[motor]
                t = min(1.0, max(0.0, time.ticks_diff(now, started) / duration))
                s = t*t*t * (10 + t * (-15 + 6*t))
                pulse = round(start + (target - start)*s)
                if pulse != self.arm.current[motor]:
                    # Reuse driver's register writes; trajectory is nonblocking here.
                    self.arm._write_pulse(motor, pulse)
                if t >= 1.0:
                    del self.active[motor]
                    if motor in self.pending:
                        self.begin(motor, self.pending.pop(motor), now)
                    else:
                        self.state(motor)
        if self.syncing and not self.active:
            self.syncing = False
            self.frame('sync', 'done')
        if self.parking and not self.active and not self.pending:
            now = time.ticks_ms()
            if self.park_deadline is None:
                self.park_deadline = time.ticks_add(now, 1000)
                self.frame('park', 'settling')
            elif time.ticks_diff(now, self.park_deadline) >= 0:
                self.arm.stop_all()
                for motor in cfg.SERVOS:
                    self.state(motor)
                self.parking = False
                self.park_deadline = None
                self.frame('park', 'done')
