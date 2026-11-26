"""ESP32 serial bridge. Place in /robot_arm with the existing driver and config."""
import sys
import time
import math
import json
import select
import servo_config as cfg
from servo_driver import ServoController


POSITION_AXES = range(1, 6)


def frame(*data):
    # Keep each USB serial write short. The desktop requests one joint at a time.
    sys.stdout.write('@' + json.dumps(data) + '\n')
    if data[0] in ('h2', 'j', 'n', 'v'):
        time.sleep_ms(20)


def emit(kind, **data):
    if kind == 'state':
        frame('s', data['motor'], data['pulse'], data['busy'])
    elif kind == 'pose':
        frame('p', *[data['current'][str(k)] for k in range(1, 7)])
    else:
        # Human-readable diagnostics may exceed one USB packet. Split them.
        message = str(data.get('message', ''))
        for pos in range(0, len(message), 30):
            frame('e' if kind == 'error' else 'i', message[pos:pos+30])


def run():
    arm = ServoController()
    active = {}
    pending = {}
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ''
    next_update = time.ticks_ms()
    parking = False
    park_deadline = None
    syncing = False

    def hello():
        frame('h2', len(cfg.SERVOS))
        frame('cap', 'park')
        frame('cap', 'sync')

    def state(motor):
        emit('state', motor=motor, pulse=arm.current[motor],
             busy=(motor in active or motor in pending))

    def duration_ms(motor, target):
        servo = arm.settings(motor)
        d = abs(target - arm.current[motor])
        duration = max(1.875 * d / servo['speed'],
                       math.sqrt(5.773503 * d / servo['accel']), 0.1)
        return math.ceil(duration * 1000)

    def begin(motor, target, now, duration=None):
        start = arm.current[motor]
        if start != target:
            if duration is None:
                duration = duration_ms(motor, target)
            active[motor] = (start, target, now, duration)
        state(motor)

    def cancel(motor):
        active.pop(motor, None)
        pending.pop(motor, None)

    def move(motor, target, now):
        if motor not in active:
            begin(motor, target, now)
        elif target == active[motor][1]:
            pending.pop(motor, None)
        else:
            # Latest target wins. Finish the current smooth segment first.
            pending[motor] = target

    def start_park():
        nonlocal parking, park_deadline
        if parking:
            return
        if syncing:
            frame('park', 'busy')
            return
        targets = getattr(cfg, 'PARK_POSITION', None)
        if not isinstance(targets, dict):
            frame('park', 'invalid', *cfg.SERVOS)
            return
        uninitialized = []
        invalid = []
        for motor in cfg.SERVOS:
            lo, hi = arm.limits(motor)
            target = targets.get(motor)
            start = arm.current[motor]
            if start is None:
                uninitialized.append(motor)
            if (type(target) is not int or not lo <= target <= hi or
                    (start is not None and not lo <= start <= hi)):
                invalid.append(motor)
        if uninitialized:
            frame('park', 'init', *uninitialized)
            return
        if invalid:
            frame('park', 'invalid', *invalid)
            return
        pending.clear()
        parking = True
        park_deadline = None
        frame('park', 'moving')
        now = time.ticks_ms()
        for motor in cfg.SERVOS:
            move(motor, targets[motor], now)

    def start_sync(starts, targets):
        nonlocal syncing
        if parking or syncing or active or pending:
            frame('sync', 'busy')
            return
        uninitialized, invalid, stale = [], [], []
        for i, start, target in zip(POSITION_AXES, starts, targets):
            lo, hi = arm.limits(i)
            current = arm.current[i]
            if current is None:
                uninitialized.append(i)
            if not lo <= start <= hi or not lo <= target <= hi:
                invalid.append(i)
            if start != current:
                stale.append(i)
        for reason, motors in (('init', uninitialized), ('limits', invalid), ('start', stale)):
            if motors:
                for i in motors:
                    state(i)
                frame('sync', reason, *motors)
                return
        duration = max(duration_ms(i, target) for i, target in zip(POSITION_AXES, targets))
        syncing = True
        frame('sync', 'moving')
        now = time.ticks_ms()
        for i, target in zip(POSITION_AXES, targets):
            begin(i, target, now, duration)

    def command(line):
        nonlocal parking, park_deadline, syncing
        parts = line.split()
        if not parts:
            return
        op = parts[0].lower()
        if op == 'hello2' and len(parts) == 1:
            hello()
            return
        if op == 'joint' and len(parts) == 2:
            motor = int(parts[1])
            servo = arm.settings(motor)
            frame('j', motor, servo['min_us'], servo['max_us'],
                  servo['center_us'], arm.current[motor])
            frame('n', motor, servo['name'], servo['model'])
            frame('v', motor, servo['speed'], servo['accel'])
            return
        if op == 'pose' and len(parts) == 1:
            emit('pose', current={str(k): v for k, v in arm.current.items()})
            return
        if op == 'park' and len(parts) == 1:
            start_park()
            return
        if op == 'sync' and len(parts) == 11:
            start_sync(tuple(int(x) for x in parts[1:6]), tuple(int(x) for x in parts[6:]))
            return
        if op in ('hold', 'stop') and len(parts) == 1:
            interrupted = parking
            interrupted_sync = syncing
            parking = False
            park_deadline = None
            syncing = False
            for motor in cfg.SERVOS:
                cancel(motor)
                if op == 'stop':
                    arm.stop(motor)
                state(motor)
            if interrupted:
                frame('park', 'stopped' if op == 'stop' else 'held')
            elif interrupted_sync:
                frame('sync', 'stopped' if op == 'stop' else 'held')
            else:
                emit('info', message='Motion cancelled; PWM ' + ('disabled' if op == 'stop' else 'held'))
            return
        if op not in ('enable', 'move', 'off') or len(parts) != (2 if op == 'off' else 3):
            raise ValueError('Invalid command')
        if parking or syncing:
            raise ValueError('Use Hold or Stop to cancel motion')
        motor = int(parts[1])
        lo, hi = arm.limits(motor)
        if op == 'off':
            cancel(motor)
            arm.stop(motor)
            state(motor)
            return
        target = int(parts[2])
        if not lo <= target <= hi:
            raise ValueError('Motor {}: allowed {}-{} us'.format(motor, lo, hi))
        if op == 'enable':
            if arm.current[motor] is not None:
                raise ValueError('Already enabled')
            # An unpowered/uncommanded servo's real angle is unknown.
            arm._write_pulse(motor, target)
            state(motor)
        else:
            if arm.current[motor] is None:
                raise ValueError('Enable motor first')
            move(motor, target, time.ticks_ms())

    hello()
    try:
        while True:
            # Limit work per pass so serial traffic cannot starve motion updates.
            for _ in range(128):
                if not poll.poll(0):
                    break
                c = sys.stdin.read(1)
                if not c:
                    break
                if c in ('\r', '\n'):
                    if buf:
                        try:
                            command(buf)
                        except (ValueError, KeyError) as exc:
                            emit('error', message=str(exc))
                        buf = ''
                elif len(buf) < 128:
                    buf += c
                else:
                    buf = ''
            now = time.ticks_ms()
            if time.ticks_diff(now, next_update) >= 0:
                next_update = time.ticks_add(now, cfg.UPDATE_MS)
                for motor in list(active):
                    start, target, started, duration = active[motor]
                    t = min(1.0, max(0.0, time.ticks_diff(now, started) / duration))
                    s = t*t*t * (10 + t * (-15 + 6*t))
                    pulse = round(start + (target - start)*s)
                    if pulse != arm.current[motor]:
                        # Reuse driver's register writes; trajectory is nonblocking here.
                        arm._write_pulse(motor, pulse)
                    if t >= 1.0:
                        del active[motor]
                        if motor in pending:
                            begin(motor, pending.pop(motor), now)
                        else:
                            state(motor)
            if syncing and not active:
                syncing = False
                frame('sync', 'done')
            if parking and not active and not pending:
                now = time.ticks_ms()
                if park_deadline is None:
                    park_deadline = time.ticks_add(now, 1000)
                    frame('park', 'settling')
                elif time.ticks_diff(now, park_deadline) >= 0:
                    arm.stop_all()
                    for motor in cfg.SERVOS:
                        state(motor)
                    parking = False
                    park_deadline = None
                    frame('park', 'done')
            time.sleep_ms(2)
    finally:
        arm.stop_all()


run()
