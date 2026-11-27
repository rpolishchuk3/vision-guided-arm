"""Shared BLE framing and authentication, usable by MicroPython and desktop Python."""

import binascii
import hashlib


DEVICE_NAME = 'RobotArm'
SERVICE_UUID = 'b34b0001-85cb-4f0d-a142-76fa58dc9f4a'
RX_UUID = 'b34b0002-85cb-4f0d-a142-76fa58dc9f4a'
TX_UUID = 'b34b0003-85cb-4f0d-a142-76fa58dc9f4a'

# 23-byte default ATT MTU minus the attribute header.
CHUNK_BYTES = 20
MAX_RECORD = 192
MAX_LINE = 128
NONCE_BYTES = 16
TAG_BYTES = 16
HEARTBEAT_MS = 500


def parse_key(text):
    """Require a pre-shared key of exactly 32 bytes in hexadecimal."""
    try:
        if len(text.strip()) == 64:
            return binascii.unhexlify(text.strip())
    except (TypeError, ValueError):
        pass
    raise ValueError('Enter the 64 hexadecimal characters of the RobotArm BLE key.')


def hmac_sha256(key, data):
    """HMAC for the fixed-size keys used here. MicroPython lacks the hmac module."""
    if len(key) != 32:
        raise ValueError('Invalid key length')
    padded = key + bytes(32)
    inner = hashlib.sha256(bytes(x ^ 0x36 for x in padded) + data).digest()
    return hashlib.sha256(bytes(x ^ 0x5c for x in padded) + inner).digest()


def same_tag(a, b):
    if len(a) != len(b):
        return False
    difference = 0
    for x, y in zip(a, b):
        difference |= x ^ y
    return difference == 0


def proof(key, nonce):
    if len(nonce) != NONCE_BYTES:
        raise ValueError('Invalid authentication challenge')
    return hmac_sha256(key, b'RobotArm/v1/proof' + nonce)


def pack(record):
    if not 0 < len(record) <= MAX_RECORD:
        raise ValueError('Invalid BLE record length')
    return bytes((len(record),)) + record


class Records:
    """Decode a byte stream in order, including split and coalesced records."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while self.buffer:
            size = self.buffer[0]
            if not 0 < size <= MAX_RECORD:
                raise ValueError('Invalid BLE record length')
            end = size + 1
            if len(self.buffer) < end:
                break
            records.append(bytes(self.buffer[1:end]))
            self.buffer = self.buffer[end:]
        return records


class Session:
    """Authenticate each record and reject repeated, skipped, or out-of-order counters."""

    def __init__(self, key, nonce, *, central):
        if len(nonce) != NONCE_BYTES:
            raise ValueError('Invalid authentication challenge')
        self.key = hmac_sha256(key, b'RobotArm/v1/session' + nonce)
        self.tx_role = b'C' if central else b'P'
        self.rx_role = b'P' if central else b'C'
        self.tx_seq = 0
        self.rx_seq = 0

    def seal(self, payload):
        if not 0 < len(payload) <= MAX_LINE:
            raise ValueError('Invalid BLE payload length')
        record = self.tx_seq.to_bytes(4, 'big') + payload
        signature = hmac_sha256(self.key, self.tx_role + record)[:TAG_BYTES]
        self.tx_seq += 1
        return pack(record + signature)

    def open(self, record):
        if not 4 + TAG_BYTES < len(record) <= 4 + TAG_BYTES + MAX_LINE:
            raise ValueError('Invalid authenticated record')
        body, signature = record[:-TAG_BYTES], record[-TAG_BYTES:]
        if (int.from_bytes(body[:4], 'big') != self.rx_seq or
                not same_tag(signature, hmac_sha256(self.key, self.rx_role + body)[:TAG_BYTES])):
            raise ValueError('BLE record authentication or sequence check failed')
        self.rx_seq += 1
        return body[4:]
