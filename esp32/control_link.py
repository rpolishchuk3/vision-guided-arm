"""Exclusive control ownership, watchdog, and line framing."""

import json
import time

from arm_controller import ArmController
from ble_protocol import MAX_LINE


class Lines:
    def __init__(self):
        self.partial = bytearray()
        self.discard = False

    def feed(self, data):
        lines = []
        for byte in data:
            if byte in (10, 13):
                if self.discard:
                    lines.append(None)
                elif self.partial:
                    try:
                        lines.append(self.partial.decode('ascii').strip())
                    except ValueError:
                        lines.append(None)
                self.partial = bytearray()
                self.discard = False
            elif not self.discard:
                if len(self.partial) == MAX_LINE - 1:
                    self.discard = True
                else:
                    self.partial.append(byte)
        return lines

    def invalidate(self):
        if self.partial:
            self.discard = True
        self.partial = bytearray()


def write_frame(link, *frame):
    link.send(('@' + json.dumps(frame) + '\n').encode(), frame[0])


class ControlLink:
    def __init__(self, arm, watchdog_ms):
        self.owner = None
        self.last_seen = 0
        self.heartbeat_only = False
        self.watchdog_ms = watchdog_ms
        self.motion = ArmController(arm, self.frame)

    def frame(self, *frame):
        if self.owner is not None:
            write_frame(self.owner, *frame)

    def acquire(self, link, now, *, heartbeat=False):
        if self.owner is not None and self.owner is not link:
            return False
        self.owner = link
        self.last_seen = now
        self.heartbeat_only = heartbeat
        return True

    def lose(self, link):
        if self.owner is link:
            self.motion.cancel_all()
            self.owner = None

    def expire(self, now):
        owner = self.owner
        if owner is not None and time.ticks_diff(now, self.last_seen) >= self.watchdog_ms:
            self.lose(owner)
            owner.expired()

    def receive(self, link, lines, now, *, usb=False):
        if not lines:
            return
        normalized = [line.lower() if line is not None else None for line in lines]
        if self.owner is not link:
            if not usb:
                return
            if 'hello2' not in normalized:
                return
            if not self.acquire(link, now):
                write_frame(link, 'link', 'busy')
                return
            start = normalized.index('hello2')
            lines = lines[start:]
            normalized = normalized[start:]

        # Emergency commands supersede actuator commands already waiting in
        # this batch. Other commands retain their arrival order.
        emergency = 'stop' if 'stop' in normalized else 'hold' if 'hold' in normalized else None
        if emergency:
            self.motion.command(emergency)
            # A queued park/sync may never have reached the motion controller.
            write_frame(link, 'cancel', emergency)
        for line, lower in zip(lines, normalized):
            if line is None:
                write_frame(link, 'e', 'Invalid or overlong input line')
                continue
            if not line:
                continue
            if not self.heartbeat_only or lower == 'ping':
                self.last_seen = now
            op = lower.split(None, 1)[0]
            if lower == 'bye':
                self.lose(link)
                return
            if lower == 'ping':
                self.heartbeat_only = True
                write_frame(link, 'hb')
            elif not emergency or op in ('hello2', 'joint', 'pose'):
                try:
                    self.motion.command(line)
                except (ValueError, KeyError) as exc:
                    self.motion.emit('error', message=str(exc))
