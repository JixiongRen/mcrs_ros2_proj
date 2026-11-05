#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
gca_source_install "${GCA_INSTALL_ROOT:-$GCA_ROOT/ros2_ws/install}"
exec "$GCA_PYTHON" "$GCA_ROOT/tools/simulation_suite.py" "$@"
