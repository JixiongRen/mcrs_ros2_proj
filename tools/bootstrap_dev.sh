#!/usr/bin/env bash
# Project-local bootstrap. Never opens devices or starts a GPIO daemon.
set -euo pipefail
GCA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$GCA_ROOT"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy underlay is missing: /opt/ros/jazzy/setup.bash' >&2
  exit 2
fi
python3 -m venv --without-pip --system-site-packages "$GCA_ROOT/.venv"
if ! "$GCA_ROOT/.venv/bin/python" -m pip --version >/dev/null 2>&1; then
  # Ubuntu may omit ensurepip. Bootstrap the pinned official pip wheel inside
  # this venv after validating its SHA-256 against PyPI release metadata.
  "$GCA_ROOT/.venv/bin/python" - "$GCA_ROOT" <<'PY'
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request

root = Path(sys.argv[1])
cache = root / '.cache' / 'bootstrap'
cache.mkdir(parents=True, exist_ok=True)
with urllib.request.urlopen('https://pypi.org/pypi/pip/25.0.1/json', timeout=30) as response:
    release = json.load(response)
wheel = next(item for item in release['urls'] if item['filename'].endswith('py3-none-any.whl'))
with urllib.request.urlopen(wheel['url'], timeout=30) as response:
    data = response.read()
if hashlib.sha256(data).hexdigest() != wheel['digests']['sha256']:
    raise SystemExit('pip wheel hash mismatch')
path = cache / wheel['filename']
path.write_bytes(data)
script = 'import runpy,sys; sys.path.insert(0,sys.argv.pop(1)); runpy.run_module("pip",run_name="__main__")'
subprocess.run([sys.executable, '-c', script, str(path), 'install', '--no-deps', str(path)], check=True)
PY
fi
"$GCA_ROOT/.venv/bin/python" -m pip install --disable-pip-version-check -r requirements-dev.txt
# No environment-variable shortcuts to legacy sources or generated packages.
set +u
source /opt/ros/jazzy/setup.bash
set -u
"$GCA_ROOT/.venv/bin/python" - <<'PY'
import importlib
import importlib.metadata
import json
import os
import platform
from pathlib import Path

for name in ('rclpy', 'yaml', 'pytest', 'serial', 'numpy', 'matplotlib', 'colcon_core', 'dynamixel_sdk', 'pigpio'):
    importlib.import_module(name)
versions = {}
for name in ('pytest', 'PyYAML', 'pyserial', 'numpy', 'matplotlib', 'dynamixel-sdk', 'pigpio', 'ruff', 'colcon-core'):
    versions[name] = importlib.metadata.version(name)
model_path = Path('/proc/device-tree/model')
record = {
    'python': platform.python_version(), 'architecture': platform.machine(),
    'kernel': platform.release(), 'ros_distro': os.environ.get('ROS_DISTRO'),
    'model': model_path.read_text().rstrip('\0') if model_path.exists() else 'unavailable',
    'os_release': Path('/etc/os-release').read_text(),
    'memory_total': next(line for line in Path('/proc/meminfo').read_text().splitlines()
                         if line.startswith('MemTotal:')),
    'packages': versions,
}
target = Path('artifacts/environment.json')
target.parent.mkdir(parents=True, exist_ok=True)
target.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(record, indent=2))
PY

