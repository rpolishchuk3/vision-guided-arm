"""ESP32 serial bridge. Place in /robot_arm with the existing driver and config."""
import sys
import time
import math
import json
import select
import servo_config as cfg
from servo_driver import ServoController


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

    def hello():
        frame('h2', len(cfg.SERVOS))

    def state(motor):
        emit('state', motor=motor, pulse=arm.current[motor],
             busy=(motor in active or motor in pending))

    def begin(motor, target, now):
        start = arm.current[motor]
        if start == target:
            state(motor)
            return
        servo = arm.settings(motor)
        d = abs(target - start)
        duration = max(1.875 * d / servo['speed'],
                       math.sqrt(5.773503 * d / servo['accel']), 0.1)
        active[motor] = (start, target, now, math.ceil(duration * 1000))
        state(motor)

    def cancel(motor):
        active.pop(motor, None)
        pending.pop(motor, None)

    def command(line):
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
        if op in ('hold', 'stop') and len(parts) == 1:
            for motor in cfg.SERVOS:
                cancel(motor)
                if op == 'stop':
                    arm.stop(motor)
                state(motor)
            emit('info', message='Motion cancelled; PWM ' + ('disabled' if op == 'stop' else 'held'))
            return
        if op not in ('enable', 'move', 'off') or len(parts) != (2 if op == 'off' else 3):
            raise ValueError('Invalid command')
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
            if motor in active:
                # Latest target wins; finish current smooth segment first.
                if target == active[motor][1]:
                    pending.pop(motor, None)
                else:
                    pending[motor] = target
            else:
                begin(motor, target, time.ticks_ms())

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
            time.sleep_ms(2)
    finally:
        arm.stop_all()


run()
