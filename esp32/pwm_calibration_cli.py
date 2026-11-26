"""Interactive calibration; session settings do not rewrite servo_config.py."""
import servo_config as cfg
from servo_driver import ServoController


HELP = '''m 1 ... m 6     Select motor
1500            Move to pulse (us)
+ / -           Change pulse by step
step 10         Set step (us)
speed 250       Set selected motor speed (us/s)
accel 500       Set selected motor acceleration (us/s^2)
center          Move to configured center pulse
min / max       Record current pulse as tested limit
range 450 2400  Set temporary probe bounds for selected motor
range reset     Restore configured bounds
status          Show selected motor settings
results         Print all recorded limits and motion settings
off             Disable selected motor PWM
stop            Disable all PWM
help            Show commands
q               Exit and disable PWM'''


def run():
    arm = ServoController()
    selected = 1
    step = cfg.DEFAULT_STEP_US
    speeds = {m: s['speed'] for m, s in cfg.SERVOS.items()}
    accels = {m: s['accel'] for m, s in cfg.SERVOS.items()}
    bounds = {m: arm.limits(m) for m in cfg.SERVOS}
    recorded = {m: [None, None] for m in cfg.SERVOS}

    def move(target):
        arm.move_pulse(selected, target, speeds[selected], accels[selected], bounds[selected])
        print('Motor {} | command: {} us'.format(selected, target))

    def results():
        print('\nCalibration results (not saved to config):')
        for m, s in cfg.SERVOS.items():
            lo, hi = recorded[m]
            print('{} {}: min_us={}, max_us={}, speed={}, accel={}'.format(
                m, s['name'], lo, hi, speeds[m], accels[m]))

    print('PWM outputs disabled; MS24 may still hold. Support the arm.')
    print(HELP)
    try:
        while True:
            try:
                parts = input('\nM{} > '.format(selected)).strip().lower().split()
                if not parts:
                    continue
                cmd = parts[0]
                pulse = arm.current[selected]
                if cmd == 'q' and len(parts) == 1:
                    break
                elif cmd == 'm' and len(parts) == 2:
                    motor = int(parts[1])
                    s = arm.settings(motor)
                    selected = motor
                    print('Selected:', selected, s['name'], s['model'])
                elif cmd in ('+', '-') and len(parts) == 1:
                    if pulse is None:
                        raise ValueError('Set an initial pulse first')
                    lo, hi = bounds[selected]
                    move(max(lo, min(hi, pulse + (step if cmd == '+' else -step))))
                elif cmd == 'step' and len(parts) == 2:
                    value = int(parts[1])
                    if not 1 <= value <= 100:
                        raise ValueError('Step must be 1-100')
                    step = value
                    print('Step:', step, 'us')
                elif cmd in ('speed', 'accel') and len(parts) == 2:
                    value = float(parts[1])
                    if not 10 <= value <= (5000 if cmd == 'speed' else 20000):
                        raise ValueError('Motion setting outside allowed range')
                    (speeds if cmd == 'speed' else accels)[selected] = value
                    print(cmd, value)
                elif cmd == 'center' and len(parts) == 1:
                    move(cfg.SERVOS[selected]['center_us'])
                elif cmd in ('min', 'max') and len(parts) == 1:
                    if pulse is None:
                        raise ValueError('No active pulse command')
                    other = recorded[selected][1 if cmd == 'min' else 0]
                    if other is not None and ((cmd == 'min' and pulse >= other) or
                                              (cmd == 'max' and pulse <= other)):
                        raise ValueError('Minimum must be less than maximum')
                    recorded[selected][0 if cmd == 'min' else 1] = pulse
                    print('Recorded', cmd, pulse, 'us')
                elif cmd == 'range':
                    if parts[1:] == ['reset']:
                        lo, hi = arm.limits(selected)
                    elif len(parts) == 3:
                        lo, hi = int(parts[1]), int(parts[2])
                    else:
                        raise ValueError('Use range MIN MAX or range reset')
                    if not cfg.PROBE_MIN_US <= lo < hi <= cfg.PROBE_MAX_US:
                        raise ValueError('Probe bounds outside software test range')
                    if pulse is not None and not lo <= pulse <= hi:
                        raise ValueError('Move inside new bounds before changing range')
                    bounds[selected] = lo, hi
                    print('Temporary bounds:', lo, hi, 'us. Verify mechanical clearance!')
                elif cmd == 'status' and len(parts) == 1:
                    s = cfg.SERVOS[selected]
                    print(s['name'], s['model'], '| channel:', s['channel'])
                    print('Command:', pulse, '| step:', step)
                    print('Configured limits:', arm.limits(selected), '| test bounds:', bounds[selected])
                    print('Speed:', speeds[selected], '| accel:', accels[selected])
                    print('Recorded min/max:', recorded[selected])
                elif cmd == 'results' and len(parts) == 1:
                    results()
                elif cmd == 'off' and len(parts) == 1:
                    arm.stop(selected)
                    print('PWM disabled for motor', selected)
                elif cmd == 'stop' and len(parts) == 1:
                    arm.stop_all()
                    print('All PWM outputs disabled')
                elif cmd == 'help' and len(parts) == 1:
                    print(HELP)
                elif len(parts) == 1 and cmd.isdigit():
                    move(int(cmd))
                else:
                    print('Unknown command. Type help.')
            except ValueError as exc:
                print('Error:', exc)
    except (KeyboardInterrupt, EOFError):
        print('Interrupted')
    finally:
        arm.stop_all()
        results()


if __name__ == '__main__':
    run()
