# PiiDrone X-314 Quad

Ultra-light, agile, stable. 70g including LiPo.

---

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
- **M1:** Front-right (CCW)
- **M2:** Front-left (CW)
- **M3:** Rear-right (CW)
- **M4:** Rear-left (CCW)

---

## Setpoint Response Table

| Axis  | Setpoint Change | Sign | Response        |
|-------|-----------------|------|-----------------|
| Roll  | Increase        | +    | Roll right      |
| Roll  | Decrease        | –    | Roll left       |
| Pitch | Increase        | +    | Pitch forward   |
| Pitch | Decrease        | –    | Pitch backward  |
| Yaw   | Increase        | +    | Yaw right (CW)  |
| Yaw   | Decrease        | –    | Yaw left (CCW)  |

---

## Control Authority Budget

### Physical Capabilities

- Total thrust: 160g (800 units) across 4 motors
- Mass: 70g (hover thrust: 350 units)
- Max lateral acceleration: 1.0g
- Thrust-to-weight: 2.28:1

### Authority Allocation

- Altitude budget: 90g (450 units = hover + 20g margin)
- Stabilization budget: 70g (350 units = 100% mass)

### Performance & Safety

- 1.0g lateral acceleration (strong rejection)
- PID output: ±116.67 units/axis (prevents saturation)
- Integral clamp: ±58.33 units (50% of PID limit)

### Design Advantage

- Mass-equivalent stabilization budget (exceeds typical)
- Conservative thrust allocation (robust in turbulence)
- Balanced control (agility, no saturation)

---

## TrueMin RMSE Relay Autotune

Inspired by the Åström–Hägglund relay auto-tuning method.

### Overview

Auto-tunes roll, pitch, and yaw using 0.5Hz relay excitation and pure RMSE. Finds the true minimum for each stage and stops when RMSE rises by 20% (with patience for 3 samples). No hard-coded gain limits.

### Features

- Smooth thrust ramp to hover (5s)
- 3-stage tuning (P → D → I) with 0.5Hz relay (15° amplitude)
- Track absolute minimum RMSE per stage
- Select true minimum (best gain tracking)
- Stop when RMSE increases by 20% after minimum (with 3-sample patience)
- Constant gain increments: P +2.0, D +1.0, I +0.01 per step
- RMSE evaluation every 0.25s (4Hz, matches code)
- All tuning within safe thrust limits

### Tuning Process

1. Ramp up thrust to hover (smoothstep, 5s)
2. Zero all gains
3. For each axis, tune in 3 stages (P, D, I):
    - Alternate setpoint at 0.5Hz (relay, 15° amplitude)
    - Accumulate squared error at each gain
    - Every 0.25s (4Hz): compute RMSE, increment gain
    - Track absolute minimum RMSE and best gain
    - Stop when RMSE increases by 20% after minimum (3-sample patience)
    - Set gain to best (minimum RMSE)
4. Repeat for roll, pitch, yaw
5. Ramp down thrust to zero (smoothstep)

### RMSE Error Metric

All decisions use:

$RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

where $e_i$ is the error at sample $i$, $N$ is the sample count.

## Results

- Tuning: ~8–32s per axis (depends on response)
- Each stage stops when RMSE increases by 20% after minimum (with patience)
- Gains: optimal, safe, flyable
- Finds true minimum, not just first acceptable
- Minimal code, robust to non-monotonic response

---

## Safety & Robustness

- Reset all PID states and setpoints when FCU inactive
- Constrain motor outputs to physical limits
- Activate failsafe/landing on packet timeout or power warning
- Abort auto-tune and clear state if thrust input changes during ramp/tune

---

## Summary

Enables safe, hands-off PID tuning for drones. Delivers reliable initial gains for stable flight. Further manual tuning is recommended for best performance. Provides a robust starting point for agile, balanced control. Control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---

## References

- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [Smoothstep](https://en.wikipedia.org/wiki/Smoothstep)
- [Root mean square deviation (RMSE)](https://en.wikipedia.org/wiki/Root_mean_square_deviation)

---