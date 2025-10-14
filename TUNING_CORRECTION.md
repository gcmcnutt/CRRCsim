# HB1.xml Tuning Correction - October 13, 2025

## Problem Identified

Analysis of CRRCsim flight logs (`~/GP/autoc/data.dat`) revealed the simulator was running **approximately 2x faster** than real flight:

- **Real flight mean velocity**: 13.22 m/s (from INAV blackbox data)
- **CRRCsim mean velocity**: 26.32 m/s (from data.dat analysis)
- **Speed ratio**: 1.99x (99% too fast)

### Symptoms:
- 89.4% of GP training flights exceeded the 16 m/s "rabbit" target speed
- Trained controllers were consistently overrunning the path targets
- Even at generation 90+, mean velocity was 25.28 m/s vs real flight's 13.22 m/s

### Root Cause:

The initial October 9 tuning was based on **peak observed speed** (27.98 m/s) rather than **mean cruise speed** (13.22 m/s). This led to:
- Power increased 25% (F: 5.67 → 7.1)
- Drag reduced 12% (CD_prof: 0.025 → 0.022)

These changes optimized for maximum speed but made cruise speeds unrealistically fast.

## Correction Applied

### Changes to hb1.xml:

| Parameter | Before | After | Change |
|-----------|--------|-------|--------|
| **Power (F)** | 7.1 | **5.0** | -30% |
| **Drag (CD_prof)** | 0.022 | **0.028** | +27% |

### Rationale:

**Balanced correction approach:**
- Reduced power by 30% to limit top-end performance
- Increased drag by 27% to better match cruise efficiency
- Combined effect should reduce mean velocity from ~26 m/s → ~13-15 m/s

**Control authority preserved:**
- Roll authority (Cl_da = 0.1666) - UNCHANGED
- Pitch authority (Cm_de = -0.253) - UNCHANGED
- Damping values (Cl_p, Cm_q) - UNCHANGED

These corrections address **speed only** while maintaining the improved responsiveness from the initial tuning.

## Expected Results

After correction, CRRCsim should show:

| Metric | Real Flight | Before Correction | Expected After |
|--------|-------------|-------------------|----------------|
| Mean velocity | 13.22 m/s | 26.32 m/s | ~13-15 m/s |
| Max velocity | 27.98 m/s | 45.91 m/s | ~28-32 m/s |
| Rabbit overrun | N/A | 89.4% | <30% |

## Verification Steps

1. **Run new GP training** with corrected hb1.xml
2. **Check data.dat** after 10-20 generations
3. **Analyze velocity distribution**:
   ```bash
   python3 ~/GP/autoc/analyze_relvel.py
   ```
4. **Target metrics**:
   - Mean velocity: 13-16 m/s (within 20% of real)
   - Rabbit overrun: <40% of samples exceed 16 m/s
   - Max velocity: <35 m/s

## Files Modified

- `/home/gmcnutt/crsim/crrcsim-0.9.13/models/hb1.xml` - Main aerodynamic model

## Analysis Tools Created

- `~/GP/autoc/analyze_flight_logs.py` - Analyzes real INAV blackbox data
- `~/GP/autoc/analyze_velocity_vs_throttle.py` - Real flight velocity by throttle
- `~/GP/autoc/analyze_relvel.py` - CRRCsim velocity analysis from data.dat

## Historical Context

**Original values** (pre-October 9):
- F = 5.668932
- CD_prof = 0.025

**First tuning** (October 9) - TOO AGGRESSIVE:
- F = 7.1 (+25%)
- CD_prof = 0.022 (-12%)
- Result: 2x too fast

**Correction** (October 13) - BALANCED:
- F = 5.0 (-12% from original, -30% from Oct 9)
- CD_prof = 0.028 (+12% from original, +27% from Oct 9)
- Expected: Close match to real flight mean

## Notes

- Control authority improvements from October 9 are **retained**
- Only power/drag modified to fix velocity mismatch
- Goal is sim-to-real transfer, not exact aerodynamic matching
- Some variance is acceptable and beneficial for GP robustness
