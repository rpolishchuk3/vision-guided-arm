"""PCA9685 hardware access and shared smooth pulse trajectory."""
from machine import Pin, I2C
import math
import time
import servo_config as cfg


class ServoController:
    def __init__(self):
        self.current = {motor: None for motor in cfg.SERVOS}
        self.i2c = I2C(cfg.I2C_BUS, sda=Pin(cfg.SDA_PIN),
                       scl=Pin(cfg.SCL_PIN), freq=cfg.I2C_FREQ)
        if cfg.PCA_ADDRESS not in self.i2c.scan():
            raise RuntimeError('PCA9685 not detected')
        self._validate_config()
        self._initialize()

    def _validate_config(self):
        channels = set()
        for motor, servo in cfg.SERVOS.items():
            ch = servo['channel']
            lo, hi = servo['min_us'], servo['max_us']
            if not 0 <= ch <= 15 or ch in channels:
                raise ValueError('Invalid/duplicate channel for motor {}'.format(motor))
            channels.add(ch)
            if not cfg.PROBE_MIN_US <= lo < hi <= cfg.PROBE_MAX_US:
                raise ValueError('Invalid limits for motor {}'.format(motor))
            if not lo <= servo['center_us'] <= hi:
                raise ValueError('Center outside limits for motor {}'.format(motor))
            if not servo['speed'] > 0 or not servo['accel'] > 0:
                raise ValueError('Invalid motion settings for motor {}'.format(motor))

    def _reg(self, register, data):
        self.i2c.writeto_mem(cfg.PCA_ADDRESS, register, data)

    def _initialize(self):
        self._reg(0x00, b'\x30')  # Sleep, auto-increment
        self._reg(0xFE, bytes([cfg.PCA_PRESCALE]))
        self._reg(0x01, b'\x04')  # Totem-pole
        # Set all configured channels full-off before waking.
        self.stop_all()
        self._reg(0x00, b'\x20')
        time.sleep_ms(10)
        self._reg(0x00, b'\xA0')

    def settings(self, motor):
        if motor not in cfg.SERVOS:
            raise ValueError('Invalid motor')
        return cfg.SERVOS[motor]

    def limits(self, motor):
        servo = self.settings(motor)
        return servo['min_us'], servo['max_us']

    def _channel_register(self, motor):
        return 0x06 + 4 * self.settings(motor)['channel']

    def _write_pulse(self, motor, pulse):
        ticks = round(pulse * cfg.MEASURED_HZ * 4096 / 1_000_000)
        if not 1 <= ticks <= 4095:
            raise ValueError('Invalid PCA9685 pulse ticks')
        self._reg(self._channel_register(motor),
                  bytes([0, 0, ticks & 0xFF, (ticks >> 8) & 0x0F]))
        self.current[motor] = pulse

    def move_pulse(self, motor, target, speed=None, accel=None, probe_limits=None):
        """Blocking movement; only calibration should supply probe_limits."""
        servo = self.settings(motor)
        lo, hi = self.limits(motor) if probe_limits is None else probe_limits
        if not cfg.PROBE_MIN_US <= lo < hi <= cfg.PROBE_MAX_US:
            raise ValueError('Invalid probe limits')
        if not lo <= target <= hi:
            raise ValueError('Pulse must be {}-{} us'.format(lo, hi))
        speed = servo['speed'] if speed is None else speed
        accel = servo['accel'] if accel is None else accel
        if not speed > 0 or not accel > 0 or not math.isfinite(speed) or not math.isfinite(accel):
            raise ValueError('Speed and acceleration must be positive and finite')
        start = self.current[motor]
        if start is None:
            print('Warning: actual position unknown; first command may jump')
            self._write_pulse(motor, target)
            return
        if not lo <= start <= hi:
            raise ValueError('Current command outside selected limits')
        distance = abs(target - start)
        if not distance:
            return
        duration = max(1.875 * distance / speed,
                       math.sqrt(5.773503 * distance / accel), 0.1)
        duration_ms = math.ceil(duration * 1000)
        start_time = time.ticks_ms()
        while True:
            elapsed = time.ticks_diff(time.ticks_ms(), start_time)
            if elapsed >= duration_ms:
                break
            t = elapsed / duration_ms
            s = t*t*t * (10 + t * (-15 + 6*t))
            pulse = round(start + (target - start) * s)
            if pulse != self.current[motor]:
                self._write_pulse(motor, pulse)
            time.sleep_ms(cfg.UPDATE_MS)
        self._write_pulse(motor, target)

    def move_angle(self, motor, angle):
        """Legacy 0-180 command scale; not a calibrated physical joint angle."""
        if not 0 <= angle <= 180:
            raise ValueError('Command angle must be 0-180')
        lo, hi = self.limits(motor)
        self.move_pulse(motor, round(lo + (hi - lo) * angle / 180))

    def stop(self, motor):
        """Disable PWM only. Some digital servos may keep holding torque."""
        self._reg(self._channel_register(motor), bytes([0, 0, 0, 0x10]))
        self.current[motor] = None

    def stop_all(self):
        for motor in cfg.SERVOS:
            self.stop(motor)
