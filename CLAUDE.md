# Claude Code Instructions for crrcsim

## Build and Test Policy

**Updated 2026-08-10 (operator).** The previous blanket *"DO NOT run builds or tests yourself"* was
situational — it dates from the `simTimeMsec` divergence session noted below, when an assistant-run build
could muddy the evidence. It is superseded by the same split the parent repo already uses:

- **Build and iterate freely.** Incremental builds (`cd build && make -j8`) and `ctest` are fine, and
  running them is preferred over handing back unverified code.
- **The GATES stay with the operator**: a clean `scripts/rebuild-perf.sh` (Constitution IV —
  determinism-affecting), the eval-vs-training bitwise check, and every training bake
  (Constitution IX; the operator drives the regression gate).

Commands:
- Rebuild: `cd build && make -j8`
- Smoke: `cd /home/gmcnutt/autoc && timeout 300 build/autoc -c autoc_diag.cfg`

## Key Files
- `src/SimStateHandler.cpp` - Simulation time management, `getSimulationTimeSinceReset()`
- `src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` - GP controller, path targeting
- `src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp` - Physics/FDM, `gPhysicsStepCounter`

## Current Issue Being Debugged
Physics traces show identical state (0 ULPs) but different `simTimeMsec` values causing GP command divergence. The `sim_steps` counter is wall-clock dependent because `multiloop` varies with frame timing.
