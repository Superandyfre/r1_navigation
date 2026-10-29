#!/bin/bash
# Called after the intended overlay is sourced, before any launch process.
neupan_release_gate() {
  local navigation_repo="$1"
  local release_manifest="${RPP_NEUPAN_RELEASE_MANIFEST:-${navigation_repo}/releases/a-plus/manifest.json}"
  local checker="${navigation_repo}/tools/release/preflight.py"
  if [[ ! -r "${release_manifest}" || ! -r "${checker}" ]]; then
    echo "NeuPAN A+ release refused: reviewed manifest/checker missing (${release_manifest})" >&2
    return 2
  fi
  /usr/bin/python3 "${checker}" --manifest "${release_manifest}"
}
