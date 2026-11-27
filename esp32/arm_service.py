"""One motion controller with USB REPL serial and optional BLE transports."""

import select
import sys
import time

from ble_protocol import parse_key
from control_link import ControlLink, Lines, write_frame
from link_config import BLE_HANDSHAKE_MS, BLE_KEY_PATH, WATCHDOG_MS
from servo_driver import ServoController


class UsbSerial:
    def __init__(self):
        self.lines = Lines()
        self.failed = False
        self.reader = select.poll()
        self.reader.register(sys.stdin, select.POLLIN)

    def poll(self):
        buffer = bytearray()
        try:
            for _ in range(256):
                if not self.reader.poll(0):
                    break
                char = sys.stdin.read(1)
                if not char:
                    break
                buffer.extend(char.encode())
        except OSError:
            self.failed = True
        return self.lines.feed(buffer)

    def send(self, data, kind):
        try:
            sys.stdout.write(data.decode())
            if kind in ('h2', 'j', 'n', 'v'):
                time.sleep_ms(20)  # Preserve the pacing of the USB handshake.
        except OSError:
            self.failed = True

    def expired(self):
        self.lines.invalidate()
        write_frame(self, 'link', 'timeout')


def start_ble(control, announce_usb):
    try:
        with open(BLE_KEY_PATH) as keyfile:
            key = parse_key(keyfile.read(129))
        from ble_peripheral import BlePeripheral
        return BlePeripheral(control, key, BLE_HANDSHAKE_MS)
    except (ImportError, OSError, ValueError) as exc:
        if announce_usb:
            print('BLE unavailable; USB remains available ({})'.format(exc))
        return None


def run(*, announce_usb=True):
    control = ControlLink(ServoController(), WATCHDOG_MS)
    usb = UsbSerial()
    peripheral = start_ble(control, announce_usb)
    try:
        if announce_usb:
            control.acquire(usb, time.ticks_ms())
            control.motion.hello()
        while True:
            now = time.ticks_ms()
            control.expire(now)
            lines = usb.poll()
            if not usb.failed:
                control.receive(usb, lines, now, usb=True)
            if usb.failed:
                control.lose(usb)
                usb.lines.invalidate()
                usb.failed = False
            if peripheral is not None:
                peripheral.poll(time.ticks_ms())
            control.expire(time.ticks_ms())
            control.motion.update(time.ticks_ms())
            time.sleep_ms(2)
    finally:
        # Ctrl+C returns to the USB REPL. The PCA9685 keeps the last output.
        control.motion.cancel_all()
        if peripheral is not None:
            peripheral.shutdown()
