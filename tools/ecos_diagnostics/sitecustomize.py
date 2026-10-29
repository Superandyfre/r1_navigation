"""Installed only through the navigation launcher's opt-in PYTHONPATH."""
import os
if os.environ.get('RPP_ECOS_DIAGNOSTICS_DIR'):
    try:
        from ecos_diagnostics import install
        install()
    except Exception as error:
        import sys
        print(f'[ECOS diagnostics] activation failed: {error}', file=sys.stderr)
