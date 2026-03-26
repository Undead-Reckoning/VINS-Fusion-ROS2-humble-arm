# Barometer Debug Output - Quick Reference

## All Printf Tags (Grep Friendly)

| Tag | File | Function | Frequency | Purpose |
|-----|------|----------|-----------|---------|
| `[BARO INPUT]` | estimator.cpp | `inputBaro()` | Per measurement | Raw input arrival |
| `[BARO BUFFER]` | estimator.cpp | `inputBaro()` | Per measurement | Queue size after push |
| `[BARO PROCESS]` | estimator.cpp | `processMeasurements()` | Per frame | Frame timing info |
| `[BARO DISCARD]` | estimator.cpp | `processMeasurements()` | Per old meas | Cleanup logging |
| `[BARO ASSIGNED]` | estimator.cpp | `processMeasurements()` | Per assigned | Success assignment |
| `[BARO NO_MEAS]` | estimator.cpp | `processMeasurements()` | Per missing | No measurement available |
| `[BARO OPTIM]` | estimator.cpp | `optimization()` | Per optimization | Setup phase info |
| `[BARO FACTOR ADD]` | estimator.cpp | `optimization()` | Per valid factor | Factor creation |
| `[BARO FACTOR SKIP]` | estimator.cpp | `optimization()` | Per invalid factor | Factor skipped |
| `[BARO_CTOR]` | barometer_factor.h | Constructor | Per BarometerFactor() | Factor instantiation |
| `[BARO_EVAL]` | barometer_factor.h | `Evaluate()` | **Very frequent** | Each cost evaluation |
| `[BARO_SETMEAS]` | barometer_factor.h | `setMeasurement()` | Per update | Measurement change |
| `[BARO MARGIN]` | estimator.cpp | `optimization()` | Per marginalization | Factor marginalized |
| `[BARO MARGIN SKIP]` | estimator.cpp | `optimization()` | Per failed margin | No valid factor to margin |
| `[BARO SLIDE]` | estimator.cpp | `slideWindow()` | Per slide | Window sliding info |

## Grep Commands

Filter to see only barometer messages:
```bash
# Show all baro messages
your_log_command | grep "\[BARO"

# Count by tag
your_log_command | grep "\[BARO" | cut -d']' -f1 | sort | uniq -c

# Show only evaluation messages
your_log_command | grep "\[BARO_EVAL\]"

# Show only factor additions
your_log_command | grep "\[BARO FACTOR"

# Show input vs no measurement ratio
your_log_command | grep -E "\[BARO (INPUT|NO_MEAS)\]"
```

## State Estimate Impact Tracking

To understand how barometer affects the state:

1. **Input ingestion**: `[BARO INPUT]` shows raw measurements
2. **Assignment**: `[BARO ASSIGNED]` shows frame-measurement linkage
3. **Factor creation**: `[BARO FACTOR ADD]` shows optimization involvement
4. **Optimization**: `[BARO_EVAL]` shows z-position correction:
   - Large residuals → large corrections to z
   - Small residuals → small corrections (already near solution)
5. **Marginalization**: `[BARO MARGIN]` shows constraint propagation

## Common Issues & Their Signatures

### Issue: No barometer measurements assigned
**Signature:**
```
[BARO NO_MEAS] Frame X has no valid baro measurement
[BARO FACTOR SKIP] Frame X: Skipping (no valid measurement)
```
**Cause:** Timing mismatch or buffer underflow  
**Fix:** Check barometer message timestamp vs image timestamp

### Issue: Factors not being used in optimization
**Signature:**
```
[BARO OPTIM] BARO_N disabled (value: 0.000000)
```
**Cause:** `BARO_N` parameter is 0  
**Fix:** Enable barometer in config: `BARO_N = 1.0` (or your chosen weight)

### Issue: Poor convergence
**Signature:**
```
[BARO_EVAL] residual=100.000  (all iterations)
[BARO_EVAL] residual=99.999   (staying high)
```
**Cause:** Either wrong measurement or large initial position error  
**Fix:** Check measurement sanity or improve initialization

### Issue: Measurements arriving but not reaching frames
**Signature:**
```
[BARO INPUT] Time: 1234567890.111111, Altitude: 50.0 m
[BARO BUFFER] Queue size after push: 5
... (then)
[BARO NO_MEAS] Frame 0 has no valid baro measurement. Queue size: 5
```
**Cause:** Message timestamps don't match frame times  
**Fix:** Check time synchronization between sensors

