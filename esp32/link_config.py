"""Communication settings for the MicroPython arm service."""

# The desktop sends a heartbeat twice per second. Increase this only if needed
# for the connection; a longer timeout also delays holding on lost USB.
WATCHDOG_MS = 3000
BLE_KEY_PATH = '/robot_arm/ble_key.txt'
BLE_HANDSHAKE_MS = 15000
