# Barometer Debug Instrumentation - Visual Flow

## Data Flow with Debug Points

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    BAROMETER MEASUREMENT INPUT                          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                            ┌───────▼────────┐
                            │  inputBaro()   │
                            └────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
          [BARO INPUT]                   [BARO BUFFER]
       "Time: X, Alt: Y m"            "Queue size: Z"
                    │                               │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                        ┌──────────────────────┐
                        │ baroBuf (queue store)│
                        └──────────────────────┘
                                    │
                                    │ (on new frame image)
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                  FRAME-LEVEL MEASUREMENT ASSIGNMENT                    │
└─────────────────────────────────────────────────────────────────────────┘
                         processMeasurements()
                                    │
                        ┌───────────▼──────────┐
                        │ [BARO PROCESS]      │
                        │ "Frame N, time X-Y" │
                        └────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
            Cleanup old meas:              Check for new meas:
            ┌──────────────────┐           ┌───────────────────┐
            │ [BARO DISCARD]   │           │ baroBuf empty?    │
            │ "time: Z"        │           └───────────────────┘
            └──────────────────┘                     │
                                    ┌────────────────┴─────────────┐
                                    │                              │
                              YES (found meas)         NO (no meas)
                                    │                              │
                        ┌───────────▼──────────┐   ┌──────────────▼───────┐
                        │ [BARO ASSIGNED]      │   │ [BARO NO_MEAS]       │
                        │ "Frame N: z=X m"     │   │ "Frame N, queue: Z"  │
                        └──────────────────────┘   └──────────────────────┘
                                    │                              │
                        ┌───────────▴──────────┐                  │
                        │ baro_z_by_frame[N] = z (or NaN)         │
                        └───────────────────────┘                  │
                                    │                              │
                                    └──────────────┬───────────────┘
                                                   │
                                                   ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                     OPTIMIZATION & FACTOR CREATION                      │
└─────────────────────────────────────────────────────────────────────────┘
                              optimization()
                                    │
                        ┌───────────▼──────────────┐
                        │ [BARO OPTIM]             │
                        │ "BARO_N: X, N frames"   │
                        │ (or "BARO_N disabled")  │
                        └──────────────────────────┘
                                    │
                    ┌───────────────┴──────────────┐
                    │                              │
              BARO_N > 0                    BARO_N <= 0
                    │                              │
    ┌───────────────▼──────────────┐    (skip all factors)
    │ For each frame:              │
    │ ┌────────────────────────┐   │
    │ │ is z finite? (NaN?)    │   │
    │ └────────┬───────┬───────┘   │
    │          │       │           │
    │        YES      NO           │
    │          │       │           │
    │  ┌──────▼──┐   ┌─▼─────┐     │
    │  │  ADD    │   │ SKIP  │     │
    │  │ FACTOR  │   │FACTOR │     │
    │  └──────┬──┘   └───────┘     │
    │         │                    │
    │  [BARO FACTOR ADD]  [BARO FACTOR SKIP]
    │  "Frame N: z=X m"   "Frame N: no meas"
    │         │                    │
    │  ┌──────▼──────────┐         │
    │  │ new BarometerFactor(z, σ) │
    │  └──────┬──────────┘         │
    │         │                    │
    │ [BARO_CTOR]                  │
    │ "z_meas=X, sigma=Y, ..."     │
    │         │                    │
    │  ┌──────▼──────────┐         │
    │  │ problem.AddResidualBlock   │
    │  └──────┬──────────┘         │
    │         │                    │
    └────────┬┴────────────────────┘
             │
┌────────────▼─────────────────────────────────────────────────────────────┐
│                    CERES SOLVER OPTIMIZATION LOOP                        │
└────────────────────────────────────────────────────────────────────────────┘
             │
    ┌────────▼─────────┐
    │ FOR EACH ITERATION
    │ ├─ Evaluate each residual
    │ │  ├─ IMUFactor::Evaluate()
    │ │  ├─ ProjectionFactor::Evaluate()
    │ │  └─ BarometerFactor::Evaluate()
    │ │         │
    │ │    [BARO_EVAL]
    │ │    "pz=X, z_meas=Y"
    │ │    "residual=Z"
    │ │         │
    │ │    (Residual shrinks: Z₁ → Z₂ → ... → ~0)
    │ │         │
    │ ├─ Compute gradients
    │ └─ Update parameters
    │
    └────────┬─────────┘
             │
    (iterate until convergence or max iterations)
             │
             ▼
┌────────────────────────────────────────────────────────────────────────────┐
│                      MARGINALIZATION & SLIDING WINDOW                      │
└────────────────────────────────────────────────────────────────────────────┘
             │
    ┌────────▼─────────────────┐
    │ if MARGIN_OLD:           │
    │ ├─ Marginalize frame 0    │
    │ │                         │
    │ │ if baro z₀ is valid:    │
    │ │    ┌──────────────────┐ │
    │ │    │ [BARO MARGIN]    │ │
    │ │    │ "Frame 0: z=X m" │ │
    │ │    └──────────────────┘ │
    │ │          │              │
    │ │    new BarometerFactor() │
    │ │          │              │
    │ │    add to margin info    │
    │ │                         │
    │ │ else:                    │
    │ │    ┌──────────────────┐  │
    │ │    │[BARO MARGIN SKIP]│  │
    │ │    │"Frame 0: no meas"│  │
    │ │    └──────────────────┘  │
    │ │                         │
    │ └─ Slide all frame arrays │
    │    (baro_z_by_frame too)  │
    │                         │
    │    [BARO SLIDE]         │
    │    "frame_count=N"      │
    │                         │
    └────────┬─────────────────┘
             │
             ▼
        (Continue to next frame)

```

---

## Message Frequency Distribution

```
                             Frequency
Tag                         (per session)    Location
─────────────────────────────────────────────────────────────
[BARO INPUT]                 ~1 per meas     inputBaro()
[BARO BUFFER]                ~1 per meas     inputBaro()
[BARO PROCESS]               ~1 per frame    processMeasurements()
[BARO DISCARD]               ~0-N per frame  processMeasurements()
[BARO ASSIGNED]              ~0-1 per frame  processMeasurements()
[BARO NO_MEAS]               ~0-1 per frame  processMeasurements()
[BARO OPTIM]                 ~1 per optim    optimization()
[BARO FACTOR ADD]            ~0-N per optim  optimization()
[BARO FACTOR SKIP]           ~0-N per optim  optimization()
[BARO_CTOR]                  ~0-N per optim  BarometerFactor ctor
[BARO_EVAL]      ⚠️ VERY HIGH!  ~1000s/optim  BarometerFactor::Evaluate()
[BARO_SETMEAS]               ~0 per session  setMeasurement()
[BARO MARGIN]                ~1 per marginal optimization()
[BARO MARGIN SKIP]           ~0 per marginal optimization()
[BARO SLIDE]                 ~0-1 per slide  slideWindow()

⚠️  [BARO_EVAL] dominates output - called many times per iteration
```

---

## Critical Paths for Diagnosis

### Path 1: "Measurements aren't being used"
```
Follow: [BARO INPUT] → [BARO ASSIGNED] → [BARO FACTOR ADD] → [BARO_EVAL]
If missing at any step, you've found the problem
```

### Path 2: "Altitude not being corrected"
```
Check: [BARO_EVAL] residuals
- Should decrease iteration by iteration
- Should trend toward zero
- If staying large, something is wrong
```

### Path 3: "Window not including barometer constraints"
```
Follow: [BARO FACTOR ADD] (optimization) → [BARO MARGIN] (marginalization)
If FACTOR ADD happens but MARGIN is skipped, constraint isn't propagating
```

### Path 4: "Measurements arriving but not assigned"
```
Check: [BARO INPUT] exists but [BARO NO_MEAS] appears
Indicates timing mismatch between barometer and image timestamps
```

---

## Expected "Success" Signature

A successful barometer-enabled run should show:

1. **Input phase**: `[BARO INPUT]` messages at sensor rate
2. **Frame processing**: Mix of `[BARO ASSIGNED]` and `[BARO NO_MEAS]` (some frames miss)
3. **Optimization setup**: `[BARO FACTOR ADD]` for frames with valid measurements
4. **Solver**: `[BARO_EVAL]` messages with decreasing residual values
5. **Marginalization**: `[BARO MARGIN]` when frames are dropped from window

```
[BARO INPUT] Time: ..., Altitude: 45.678 m        ✓ Sensor data arriving
[BARO ASSIGNED] Frame 0 assigned altitude: 45.678 m   ✓ Frame assigned
[BARO FACTOR ADD] Frame 0: Adding BarometerFactor ... ✓ Optimization included
[BARO_EVAL] pz=..., residual=45.678              ✓ Initial (large residual)
[BARO_EVAL] pz=..., residual=2.345               ✓ Converging
[BARO_EVAL] pz=..., residual=0.001               ✓ Converged
[BARO MARGIN] Marginalizing frame 0 with baro z= ✓ Constraint propagated
```

