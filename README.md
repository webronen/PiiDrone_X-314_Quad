# PiiDrone X-314 Quad

Ultra-light, agile, and stable. 70g including LiPo.

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

| Axis  | Setpoint Change | Sign | Response         |
|-------|-----------------|------|------------------|
| Roll  | Increase        | +    | Rolls right      |
| Roll  | Decrease        | –    | Rolls left       |
| Pitch | Increase        | +    | Pitches forward  |
| Pitch | Decrease        | –    | Pitches backward |
| Yaw   | Increase        | +    | Yaws right (CW)  |
| Yaw   | Decrease        | –    | Yaws left (CCW)  |

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

- 1.0g lateral acceleration for strong rejection
- PID output: ±116.67 units/axis (prevents saturation)
- Integral clamp: ±58.33 units (50% of PID limit)

### Design Advantage

- Mass-equivalent stabilization budget (exceeds typical)
- Conservative thrust allocation (robust in turbulence)
- Balanced control (agility without saturation)

---

## TrueMin RMSE Relay Autotune

Inspired by the Åström–Hägglund relay auto-tuning method.

### Overview

TrueMin RMSE Relay Autotune for roll, pitch, and yaw on a balanced test bench. Uses the Åström–Hägglund relay method with 2Hz relay excitation for system identification. Tracks absolute minimum RMSE for each stage. Stops tuning when RMSE increases by 20% after the minimum. Gain growth is limited only by the RMSE-based stopping rule.

### Features

- Smooth thrust ramp to hover
- 3-stage tuning (P → D → I) with 2Hz relay (square wave setpoint)
- Tracks absolute minimum RMSE per stage
- Always selects the true minimum (best gain tracking)
- Stops when RMSE increases by 20% after minimum
- Constant gain increments (see code)
- 20Hz RMSE evaluation (fast, robust)

### Tuning Process

1. Ramp up thrust to hover (smoothstep)
2. Zero all gains
3. For each axis, tune in 3 stages (P, D, I):
   - Relay: setpoint alternates at 2Hz (square wave)
   - At each gain, accumulate squared error
   - Every 0.05s (20Hz): compute RMSE, increment gain
   - Track absolute minimum RMSE and best gain
   - Stop when RMSE increases by 20% after minimum
   - Set gain to best (minimum RMSE)
4. Repeat for roll, pitch, yaw
5. Ramp down thrust to zero (smoothstep)

### RMS Error Metric

All decisions use:

$RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

where $e_i$ is the error at sample $i$, $N$ is the sample count.

## Results

- Tuning: ~8–32s per axis (depends on response)
- Each stage stops when RMSE increases by 20% after minimum
- Gains: optimal, safe, flyable
- Finds true minimum, not just first acceptable
- Minimal code, robust to non-monotonic response

---

## Safety & Robustness

- All PID states and setpoints reset when FCU inactive
- Motor outputs always constrained to physical limits
- Failsafe/landing logic on packet timeout or power warning
- Auto-tune aborts and state clears if thrust input changes during ramp/tune

---

## Summary

Enables safe, hands-off PID gain estimation for drones. Delivers reliable initial gains for stable flight. Further manual tuning is recommended for best performance. Auto-tune provides a robust starting point for agile, balanced control. Control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---

## 3D Model

- [PiiDrone X-314 Quad (Maker World)](https://makerworld.com/en/models/1153207-piidrone-x-314-quad)

---

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [Smoothstep](https://en.wikipedia.org/wiki/Smoothstep)
- [Root mean square deviation (RMSE)](https://en.wikipedia.org/wiki/Root_mean_square_deviation)

---