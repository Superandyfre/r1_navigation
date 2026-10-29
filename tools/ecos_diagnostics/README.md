# ECOS diagnostics

Opt-in process-local interception of `ecos.solve`. No solver settings, return values, exceptions, retries or motion authorization are changed. Installation is lazy, so NumPy/BLAS are not imported before the existing Torch thread policy. Multiprocessing spawn workers inherit the hook through PYTHONPATH.

Each worker writes `ecos-<PID>.jsonl`: wall time, call number, duration and the raw ECOS `info` dictionary, including exitFlag, infostring, iter, pres/dres, gap/relgap, numerical-error flags and ECOS timings when supplied by ECOS. Four rotating 4 MiB files per PID limit log storage. Failures also print a summary to the launch terminal before diffcp reduces negative exit codes to a generic SolverError.

Failed solves capture the canonical cone problem (c, G, h, dims, A, b), solver options and available request/cycle/path context in `ecos-failure-<PID>-<slot>.npz`. Eight rotating slots per PID; inputs above 32 MiB are skipped with an explicit error. Numerical arrays can be loaded with `allow_pickle=False`. Disk/diagnostic failures must preserve the original solver outcome. The hook adds log I/O and failure capture time; existing independent watchdogs/deadlines are retained. Old run directories are preserved, so their number is not globally bounded.

Source `enable.bash` before starting Python. The UI launchers do this automatically and print the log directory. Override RPP_ECOS_DIAGNOSTICS_DIR to choose a destination. No system Python/venv packages are modified.

Replay a captured problem offline with the same environment:

```bash
/home/rpp/nav_restore_20261016/venv/bin/python /home/rpp/r1_navigation_1016_diagnostics/tools/ecos_diagnostics/replay.py /path/to/ecos-failure-PID-slot.npz --verbose
```

`--verbose` prints the ECOS iteration table during offline replay; live solver verbosity/options remain unchanged. One captured problem may not reproduce a failure caused by process memory or timing differences.

Run `test_diagnostics.py` with the NeuPAN Python environment. Tests compare wrapped/unwrapped real solves, force an iteration-limit failure, replay its saved input, verify child-process activation and ensure logging errors cannot replace solver errors or add retries.
