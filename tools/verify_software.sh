#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
exec "$GCA_PYTHON" "$GCA_ROOT/tools/verify_software.py" "$@"
