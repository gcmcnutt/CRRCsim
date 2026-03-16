# Claude Code Instructions for crrcsim

## Build and Test Policy
DO NOT run builds or tests yourself. After making code changes, inform the user that:
- The code needs to be rebuilt: `cd build && make -j8`
- Testing can be done with: `cd /home/gmcnutt/autoc && timeout 300 build/autoc -c autoc_diag.cfg`

The user will run these commands and share results.

## Key Files
- `src/SimStateHandler.cpp` - Simulation time management, `getSimulationTimeSinceReset()`
- `src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` - GP controller, path targeting
- `src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp` - Physics/FDM, `gPhysicsStepCounter`

## Current Issue Being Debugged
Physics traces show identical state (0 ULPs) but different `simTimeMsec` values causing GP command divergence. The `sim_steps` counter is wall-clock dependent because `multiloop` varies with frame timing.
