#!/usr/bin/env bash
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
#
# Launch a gaitos simulation, run smoke.py against it and stop it (CI).
#   smoke_launch.sh <launch package> <robot> [smoke.py options] -- [launch arguments]
set -uo pipefail

pkg=$1 robot=$2
shift 2
smoke_args=()
while [[ $# -gt 0 && $1 != -- ]]; do smoke_args+=("$1"); shift; done
[[ ${1:-} == -- ]] && shift

here=$(cd "$(dirname "$0")" && pwd)
log=${SMOKE_LOG_DIR:-.}/smoke_${pkg}_${robot}.log
mkdir -p "$(dirname "$log")"

echo "::group::launch $pkg robot:=$robot $*"
setsid ros2 launch "$pkg" sim.launch.py robot:="$robot" "$@" >"$log" 2>&1 &
launch_pid=$!
echo "launch pid $launch_pid, log $log"
echo "::endgroup::"

stop() {
  kill -INT -- "-$launch_pid" 2>/dev/null
  for _ in $(seq 20); do kill -0 "$launch_pid" 2>/dev/null || return; sleep 0.5; done
  kill -KILL -- "-$launch_pid" 2>/dev/null
}
trap stop EXIT

timeout 900 python3 "$here/smoke.py" "$robot" "${smoke_args[@]}"
rc=$?

if [[ $rc -ne 0 ]]; then
  echo "::group::launch log (last 200 lines)"
  tail -n 200 "$log"
  echo "::endgroup::"
fi
exit $rc
