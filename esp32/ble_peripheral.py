"""RobotArm BLE GATT endpoint. IRQ handlers only record link events and RX work."""

import bluetooth
import os
import time

from ble_protocol import (
    CHUNK_BYTES, DEVICE_NAME, MAX_LINE, NONCE_BYTES, RX_UUID, SERVICE_UUID,
    TX_UUID, Records, Session, pack, proof, same_tag,
)


IRQ_CONNECT = 1
IRQ_DISCONNECT = 2
IRQ_WRITE = 3
RX_BUFFER = 1024
TX_BUFFER = 4096


class BlePeripheral:
    def __init__(self, control, key, handshake_ms):
        self.control = control
        self.key = key
        self.handshake_ms = handshake_ms
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.config(gap_name=DEVICE_NAME)
        service_uuid = bluetooth.UUID(SERVICE_UUID)
        service = (service_uuid, (
            (bluetooth.UUID(RX_UUID), bluetooth.FLAG_WRITE),
            (bluetooth.UUID(TX_UUID), bluetooth.FLAG_NOTIFY),
        ))
        (self.rx_handle, self.tx_handle), = self.ble.gatts_register_services((service,))
        self.ble.gatts_set_buffer(self.rx_handle, RX_BUFFER, True)
        name = DEVICE_NAME.encode()
        self.advertisement = (b'\x02\x01\x06\x11\x07' + bytes(service_uuid)
                              + bytes((len(name) + 1, 0x09)) + name)
        self.events = []
        self.rx_peer = None
        self.fault = False
        self.conn = None
        self.records = Records()
        self.session = None
        self.nonce = None
        self.outgoing = bytearray()
        self.connected_at = 0
        self.rejected_at = None
        self.closing = False
        self.ble.irq(self._irq)
        self._advertise()

    def _advertise(self):
        self.ble.gap_advertise(250_000, adv_data=self.advertisement)

    def _irq(self, event, data):
        if event in (IRQ_CONNECT, IRQ_DISCONNECT):
            if len(self.events) < 8:
                self.events.append((event, data[0]))
            else:
                self.fault = True
        elif event == IRQ_WRITE and data[1] == self.rx_handle:
            if self.rx_peer is not None and self.rx_peer != data[0]:
                self.fault = True
            self.rx_peer = data[0]

    def _disconnected(self):
        self.control.lose(self)
        self.conn = None
        self.rx_peer = None
        self.session = None
        self.nonce = None
        self.rejected_at = None
        self.closing = False
        self.fault = False
        self.records = Records()
        self.outgoing = bytearray()
        self.ble.gatts_read(self.rx_handle)  # Discard bytes left by the old link.
        self._advertise()

    def _close(self):
        self.control.lose(self)
        if self.conn is not None and not self.closing:
            self.closing = True
            try:
                disconnected = self.ble.gap_disconnect(self.conn)
            except OSError:
                disconnected = False
            if disconnected is False:
                self._disconnected()

    def expired(self):
        self._close()

    def send(self, data, _kind):
        if self.session is None or self.closing:
            return
        if len(self.outgoing) + MAX_LINE + 21 > TX_BUFFER:
            self.fault = True
            return
        self.outgoing.extend(self.session.seal(data))

    def _record(self, record, now, lines):
        if self.session is not None:
            payload = self.session.open(record)
            if not payload.endswith(b'\n') or b'\n' in payload[:-1] or b'\r' in payload:
                raise ValueError('Invalid command line')
            lines.append(payload[:-1].decode('ascii').strip())
        elif record == b'?' and self.nonce is None:
            self.nonce = os.urandom(NONCE_BYTES)
            self.outgoing.extend(pack(b'C' + self.nonce))
        elif record[:1] == b'A' and self.nonce is not None:
            if not same_tag(record[1:], proof(self.key, self.nonce)):
                self.outgoing.extend(pack(b'!'))
                self.rejected_at = now
                return
            self.session = Session(self.key, self.nonce, central=False)
            if self.control.acquire(self, now, heartbeat=True):
                self.outgoing.extend(self.session.seal(b'ok'))
            else:
                self.outgoing.extend(self.session.seal(b'busy'))
                self.rejected_at = now
        else:
            raise ValueError('Invalid authentication exchange')

    def poll(self, now):
        try:
            while self.events:
                event, conn = self.events.pop(0)
                if event == IRQ_CONNECT:
                    if self.conn is None:
                        self.conn = conn
                        self.connected_at = now
                    elif self.conn != conn:
                        self.ble.gap_disconnect(conn)
                        self.fault = True
                elif conn == self.conn:
                    self._disconnected()
            if self.conn is None or self.closing:
                return
            if self.fault or (self.session is None and
                              time.ticks_diff(now, self.connected_at) >= self.handshake_ms):
                self._close()
                return
            if self.rejected_at is not None:
                if time.ticks_diff(now, self.rejected_at) >= 250:
                    self._close()
                    return
            elif self.rx_peer is not None:
                peer = self.rx_peer
                self.rx_peer = None
                data = self.ble.gatts_read(self.rx_handle)
                if peer != self.conn or len(data) >= RX_BUFFER:
                    raise ValueError('Invalid or overrun BLE buffer')
                lines = []
                for record in self.records.feed(data):
                    self._record(record, now, lines)
                    if self.rejected_at is not None:
                        break
                if not self.events and not self.fault and self.rejected_at is None:
                    self.control.receive(self, lines, now)
            if self.outgoing and not self.fault:
                chunk = bytes(self.outgoing[:CHUNK_BYTES])
                self.ble.gatts_notify(self.conn, self.tx_handle, chunk)
                self.outgoing = self.outgoing[len(chunk):]
        except (ValueError, OSError):
            self._close()

    def shutdown(self):
        self.control.lose(self)
        self.ble.active(False)
