#!/usr/bin/env bash
# Shared shell environment. Callers set errexit/pipefail; setup scripts need unset vars.
GCA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GCA_PYTHON="$GCA_ROOT/.venv/bin/python"
if [[ ! -x "$GCA_PYTHON" ]]; then
  echo 'Run bash tools/bootstrap_dev.sh first.' >&2
  return 2
fi
GCA_RESTORE_NOUNSET=0
if [[ $- == *u* ]]; then GCA_RESTORE_NOUNSET=1; fi
set +u
source /opt/ros/jazzy/setup.bash
source "$GCA_ROOT/.venv/bin/activate"
if [[ "$GCA_RESTORE_NOUNSET" == 1 ]]; then set -u; fi
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONDONTWRITEBYTECODE=1

gca_source_install() {
  local overlay="${1:-$GCA_ROOT/ros2_ws/install}"
  local restore_nounset=0
  if [[ $- == *u* ]]; then restore_nounset=1; fi
  if [[ ! -f "$overlay/local_setup.bash" ]]; then
    echo "GCA installation is missing: $overlay" >&2
    return 2
  fi
  set +u
  source "$overlay/local_setup.bash"
  if [[ "$restore_nounset" == 1 ]]; then set -u; fi
}
