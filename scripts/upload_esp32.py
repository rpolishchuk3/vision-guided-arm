"""Upload this repository's MicroPython files with mpremote."""

import argparse
from importlib.util import find_spec
from pathlib import Path, PurePosixPath
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
REMOTE_ROOT = PurePosixPath("/robot_arm")
DEFAULT_PORT = "COM10"


def find_firmware(root):
    firmware = root / "esp32"
    if not firmware.is_dir():
        raise ValueError(f"Firmware directory not found: {firmware}")
    return firmware.resolve()


def upload_command(firmware, port):
    files_by_destination = {}
    directories = set()
    for path in sorted(firmware.rglob("*.py")):
        if not path.is_file():
            continue
        if not path.resolve().is_relative_to(firmware):
            raise ValueError(f"Firmware file points outside the source directory: {path}")
        relative = path.relative_to(firmware)
        destination = REMOTE_ROOT / relative.parent.as_posix()
        files_by_destination.setdefault(destination, []).append(str(path))
        directories.update(str(REMOTE_ROOT / parent.as_posix()) for parent in relative.parents)
    if not files_by_destination:
        raise ValueError(f"No Python firmware files found in {firmware}.")

    # Create parent directories before children. A file at a directory path is an error.
    ensure_directories = f"""import errno, os
for path in {tuple(sorted(directories))!r}:
    try:
        mode = os.stat(path)[0]
    except OSError as exc:
        if not exc.args or exc.args[0] != errno.ENOENT:
            raise
        os.mkdir(path)
    else:
        if not mode & 0x4000:
            raise OSError("Expected a directory: " + path)
"""
    command = [sys.executable, "-m", "mpremote", "connect", f"port:{port}", "+",
               "exec", ensure_directories]
    for destination, files in files_by_destination.items():
        # mpremote skips files with matching SHA-256 hashes by default.
        command.extend(["+", "fs", "cp", *files, f":{destination}/"])
    return command


def startup_command(firmware, port):
    launcher = firmware / "standalone_start.py"
    if not launcher.is_file():
        raise ValueError(f"Missing standalone launcher: {launcher}")
    check = """import errno, os, sys
try:
    os.stat('/main.py')
except OSError as exc:
    if not exc.args or exc.args[0] != errno.ENOENT:
        raise
else:
    raise RuntimeError('Existing /main.py preserved. Back it up and integrate the launcher manually.')
os.stat('/robot_arm/arm_service.py')
os.stat('/robot_arm/ble_peripheral.py')
sys.path.insert(0, '/robot_arm')
from ble_protocol import parse_key
from link_config import BLE_KEY_PATH
import bluetooth
with open(BLE_KEY_PATH) as keyfile:
    parse_key(keyfile.read(129))
"""
    return [sys.executable, "-m", "mpremote", "connect", f"port:{port}", "+", "exec", check,
            "+", "fs", "cp", str(launcher), ":/main.py"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial port (default: %(default)s)")
    parser.add_argument("--install-ble-startup", action="store_true",
                        help="explicitly install /main.py after deployment and key setup, only if absent")
    args = parser.parse_args(argv)
    port = args.port.strip()
    if not port:
        parser.error("--port must specify a serial port")

    try:
        firmware = find_firmware(REPO_ROOT)
        command = startup_command(firmware, port) if args.install_ble_startup else upload_command(firmware, port)
        if find_spec("mpremote") is None:
            raise ValueError("mpremote is missing. Install it with: python -m pip install mpremote")
        destination = "/main.py" if args.install_ble_startup else str(REMOTE_ROOT) + "/"
        print(f"Deploying {firmware} to {port}:{destination}", flush=True)
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as exc:
        print(f"Deployment to {port} stopped (mpremote exit code {exc.returncode}). See output above.",
              file=sys.stderr)
        return 1
    except (OSError, ValueError) as exc:
        print(f"Deployment stopped: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Deployment cancelled.", file=sys.stderr)
        return 130
    print("ESP32 deployment complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
