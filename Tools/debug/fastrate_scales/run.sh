#!/bin/bash
# Build the debug copter, fly the profile for each configuration and analyse the logs.
# usage: Tools/debug/fastrate_scales/run.sh [name:parmlist ...]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
OUT=${OUT:-$HERE/runs}
mkdir -p "$OUT"
if [ $# -eq 0 ]; then
  set -- nofast400:common.parm,nofast.parm \
         fast400:common.parm,fast.parm \
         fast200:common.parm,fast.parm,loop200.parm \
         fast100:common.parm,fast.parm,loop100.parm
fi
(cd "$ROOT" && ./waf configure --board sitl >/dev/null && ./waf copter)
BIN="$ROOT/build/sitl/bin/arducopter"
for spec in "$@"; do
  name=${spec%%:*}; parms=${spec#*:}
  echo "=== $name ($parms)"
  python3 "$HERE/fly.py" "$BIN" "$OUT/$name" "$parms" "$name" > "$OUT/$name.fly.out" 2>&1
  python3 "$HERE/analyze.py" "$OUT/$name" | tee "$OUT/$name.analyze.out"
done
