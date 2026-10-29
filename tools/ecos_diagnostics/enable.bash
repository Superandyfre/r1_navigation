# Source before starting Python. Inherited by multiprocessing spawn workers.
ecos_diagnostics_tools="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${RPP_ECOS_DIAGNOSTICS_DIR:-}" ]]; then
    ecos_diagnostics_root="$(cd "${ecos_diagnostics_tools}/../.." && pwd)"
    export RPP_ECOS_DIAGNOSTICS_DIR="${ecos_diagnostics_root}/n1_results/ecos_diagnostics/$(date +%Y%m%d-%H%M%S)-$$"
fi
mkdir -p "${RPP_ECOS_DIAGNOSTICS_DIR}" || return 1
export PYTHONPATH="${ecos_diagnostics_tools}:${PYTHONPATH:-}"
echo "ECOS diagnostics: ${RPP_ECOS_DIAGNOSTICS_DIR}"
unset ecos_diagnostics_tools ecos_diagnostics_root
