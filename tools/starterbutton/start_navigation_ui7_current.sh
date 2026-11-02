#!/bin/bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "${script_dir}/ui7_current.py" \
  --pointer "${RPP_UI7_RELEASE_POINTER:-/home/rpp/neupan_failstop_ws/ui7/current.json}" "$@"
