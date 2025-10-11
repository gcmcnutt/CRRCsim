# CRRCsim HB1 Tuning Summary

## Overview

This document summarizes the tuning changes made to the HB1 flying wing model in CRRCsim based on analysis of real flight blackbox data from INAV logs.

## Analysis Process

1. **Data Collection**: Analyzed 6 flight log files (flight5.csv, flight10.csv, etc.)
2. **Valid Flight Data**: Extracted 11,388 samples where motor output > 1100 (active flight)
3. **Key Metrics Extracted**:
   - Roll/pitch rates and control authority
   - Flight speeds (cruise and maximum)
   - Throttle usage and motor output
   - Battery voltage for power estimation

## Flight Data Summary

### Observed Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **Mean Cruise Speed** | 13.22 m/s | Slightly faster than sim default (12.0 m/s) |
| **Max Speed** | 27.98 m/s | Indicates good power-to-weight ratio |
| **Max Roll Rate** | 674 deg/s | Very high - extremely responsive in roll |
| **Mean Active Roll Rate** | 136.2 deg/s | Typical maneuvering roll rate |
| **Max Pitch Rate** | 298 deg/s | High pitch authority |
| **Mean Active Pitch Rate** | 47.2 deg/s | Typical maneuvering pitch rate |
| **Mean Battery Voltage** | 11.62V | 3S LiPo (nominal 11.1V) |

### Key Observations

1. **Very Responsive Roll**: The observed 674 deg/s max roll rate indicates the real aircraft has excellent roll authority - much higher than typical trainers (200-300 deg/s) and approaching aerobatic performance.

2. **Good Pitch Authority**: 298 deg/s max pitch rate is strong for a flying wing, suggesting effective elevator surfaces.

3. **Good Speed Range**: 13-28 m/s (29-62 mph) shows versatile performance from cruise to high-speed flight.

4. **Anecdotal Confirmation**: User reports that sim feels "slower" than real flight - confirmed by data showing need for increased control authority.

## Changes Made

### hb1_new.xml (New Aerodynamic Model)

#### 1. Reference Parameters (`<ref>` section)
- **speed**: 12.0 → **13.2** m/s
- **Reason**: Match observed mean cruise speed

#### 2. Roll Authority (`<l>` section - lateral dynamics)
- **Cl_da** (aileron effectiveness): 0.13327 → **0.1666** (+25%)
- **Cl_p** (roll damping): -0.42547 → **-0.383** (-10% magnitude)
- **Reason**: Increase roll rate and responsiveness to match observed 674 deg/s peak performance

#### 3. Pitch Authority (`<m>` section - pitching moment)
- **Cm_de** (elevator effectiveness): -0.210653 → **-0.253** (+20% magnitude)
- **Cm_q** (pitch damping): -1.68929 → **-1.52** (-10% magnitude)
- **Reason**: Increase pitch rate to match observed 298 deg/s peak performance

#### 4. Drag (`<drag>` section)
- **CD_prof** (profile drag coefficient): 0.025 → **0.022** (-12%)
- **Reason**: Reduce drag slightly to better match cruise speed performance

#### 5. Power (`<power>` section)
- **F** (thrust force): 5.668932 → **7.1** (+25%)
- **Reason**: Increase available power to match observed max speed (27.98 m/s) and acceleration

### autoc_config_new.xml

- **airplane file**: models/hb1.xml → **models/hb1_new.xml**
- **Reason**: Point to the new tuned aerodynamic model
- All other settings unchanged

## Parameter Explanation

### Aerodynamic Derivatives

**Roll Control:**
- `Cl_da`: Rolling moment due to aileron deflection. Higher values = more roll authority.
- `Cl_p`: Roll damping. More negative = slower roll rate convergence. Reduced magnitude = more responsive.

**Pitch Control:**
- `Cm_de`: Pitching moment due to elevator deflection. More negative = stronger pitch-up authority.
- `Cm_q`: Pitch damping. More negative = slower pitch rate convergence. Reduced magnitude = more responsive.

**Drag:**
- `CD_prof`: Parasitic drag coefficient. Lower values = less drag at cruise speeds.

**Power:**
- `F`: Maximum thrust force in Newtons. Higher values = more acceleration and top speed.

## Expected Sim-to-Real Transfer

These changes aim to make the simulator more representative of real flight without exact matching:

1. **Control Response**: Sim should now feel more responsive in roll and pitch, closer to the real aircraft's snappy handling.

2. **Speed Performance**: Cruise speed should increase slightly, and max speed should improve with the power increase.

3. **Power-to-Weight Feel**: The aircraft should feel more powerful during climbs and acceleration.

4. **Damping**: Reduced damping means the aircraft will be slightly less stable but more maneuverable - matching the real HB1's flying wing characteristics.

## Testing Recommendations

1. **Compare Side-by-Side**: Fly both hb1.xml and hb1_new.xml in the simulator to feel the difference.

2. **Roll Response**: Test max roll rate in sim vs. the 674 deg/s observed in real flight.

3. **Cruise Performance**: Verify cruise speed is closer to 13 m/s at moderate throttle.

4. **Power**: Check that acceleration and climb feel more aggressive.

5. **GP Controller Transfer**: Test whether GP controllers trained on hb1_new.xml transfer better to real flight.

## Files Created

- **~/crsim/crrcsim-0.9.13/models/hb1_new.xml**: New tuned aircraft model
- **~/crsim/crrcsim-0.9.13/autoc_config_new.xml**: Configuration pointing to new model
- **~/GP/autoc/analyze_flight_logs.py**: Python script for analyzing blackbox logs
- **~/crsim/crrcsim-0.9.13/TUNING_SUMMARY.md**: This document

## Usage

To use the new configuration:

```bash
# Option 1: Use autoc_config_new.xml directly
# (Point your simulator to this config file)

# Option 2: Replace the old files (backup first!)
cd ~/crsim/crrcsim-0.9.13
cp models/hb1.xml models/hb1_backup.xml
cp models/hb1_new.xml models/hb1.xml
cp autoc_config.xml autoc_config_backup.xml
cp autoc_config_new.xml autoc_config.xml
```

## Future Tuning

If further refinement is needed:

1. **Re-run Analysis**: Use `analyze_flight_logs.py` on new flight data
2. **Iterative Adjustment**: Make smaller incremental changes (5-10%) to parameters
3. **Validation**: Test GP-trained controllers in both sim and real flight
4. **Document Results**: Track which parameters most affect sim-to-real transfer

## Notes

- These changes are conservative (20-25% increases) to avoid over-correcting
- The goal is "close enough" for GP training, not exact aerodynamic matching
- Real HB1 builds vary, so some variance is expected and acceptable
- Focus is on control response rates rather than absolute aerodynamic accuracy
