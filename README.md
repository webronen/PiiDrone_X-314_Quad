
# PiiDrone X-314 Quad

Ultra-light, agile, and stable. 70g including LiPo.

## Motor Layout (X Configuration)

```
      Rear
        |
 |----|----|
 | M4 | M3 |
 |----|----|
 | M2 | M1 |
 |----|----|
        |
      Front
```

M1: Front-right (CCW)
M2: Front-left (CW)
M3: Rear-right (CW)
M4: Rear-left (CCW)

## Setpoint Response

| Axis  | Setpoint   | Sign | Response         |
|-------|------------|------|------------------|
| Roll  | Increase   | +    | Roll right       |
| Roll  | Decrease   | –    | Roll left        |
| Pitch | Increase   | +    | Pitch forward    |
| Pitch | Decrease   | –    | Pitch backward   |
| Yaw   | Increase   | +    | Yaw right (CW)   |
| Yaw   | Decrease   | –    | Yaw left (CCW)   |

## Control Authority

- Total thrust: 160g (800 units) across 4 motors
- Mass: 70g (hover thrust: 350 units)
- Max lateral acceleration: 1.0g
- Thrust-to-weight: 2.28:1
- Altitude budget: 90g (450 units = hover + 20g margin)
- Stabilization budget: 70g (350 units = 100% mass)
- PID output: ±116.67 units/axis (prevents saturation)
- Integral clamp: ±58.33 units (50% of PID limit)

## PiiTune Deep RMSE - PID Auto-Tune System

### Overview
Fully-automated, per-axis PID gain tuning using relay feedback, RMSE error metric, and robust overfitting/stability validation.

### Tuning Process
1. **Initialization:** For each axis, tuning starts at the P stage (D/I zeroed), relay setpoint toggling excites the axis.
2. **Stage Loop (P → D → I):**
  - Increment the relevant gain by a fixed step after each evaluation window.
  - RMSE is computed over a window of samples.
  - If RMSE improves, update best gain and reset patience.
  - If RMSE worsens (beyond a tolerance), increment patience.
  - If patience runs out, revert to best gain and start a stability test.
3. **Stability Test:**
  - Hold the best gain for 10 seconds.
  - If RMSE remains at/below best, stage is complete.
  - If RMSE worsens, increment gain and restart timer (protects against overfitting).
4. **Stage/Axis Completion:**
  - Progress through P → D → I. When all stages are stable, axis is done.
  - When all axes are done, tuning stops and gains are stored.

### Key Features
- Per-axis, staged tuning (P → D → I)
- Relay excitation for system identification
- RMSE-based error evaluation
- Best gain tracking and patience-driven stopping
- Overfitting avoidance and stability validation
- All configuration centralized in macros/arrays

### Performance
- Training Time: ~90-180 seconds total
- Stage Completion: 20% RMSE increase triggers stability test
- Gain Quality: Axis-specific, stable, and flight-ready

## Safety & Robustness
- Reset all PID states and setpoints when FCU inactive
- Constrain motor outputs to physical limits (0-800 units)
- Activate failsafe/landing on packet timeout or power warning
- Abort auto-tune if thrust input changes during ramp/tune
- Independent axis safety management

## Summary
Enables safe, hands-off PID tuning. Delivers reliable axis-specific gains for stable flight. Control authority budget supports robust flight dynamics for the PiiDrone X-314 Quad.

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A9gglund)_method)
- [Root mean square deviation (RMSE)](https://en.wikipedia.org/wiki/Root_mean_square_deviation)
- [Reinforcement learning](https://en.wikipedia.org/wiki/Reinforcement_learning)
- [Machine learning](https://en.wikipedia.org/wiki/Machine_learning)
- [Smoothstep](https://en.wikipedia.org/wiki/Smoothstep)