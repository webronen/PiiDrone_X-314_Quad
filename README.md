# PiiDrone X-314 Quad

**Ultra-light, agile, and stable powerhouse (70g including LiPo)**

---

## Motor Layout (X Configuration)

```
               Rear
                 |
           | --- | --- |
           | M4  | M3  |
           | --  | --  |
           | M2  | M1  |
           | --  | --  |
                 |
               Front
```
- **M1:** Front-right (CCW)
- **M2:** Front-left (CW)
- **M3:** Rear-right (CW)
- **M4:** Rear-left (CCW)

---

## Setpoint Response Table

| Axis  | Setpoint Change | Sign | Expected Drone Response |
| ----- | --------------- | ---- | ----------------------- |
| Roll  | Increase        | +    | Rolls right             |
| Roll  | Decrease        | –    | Rolls left              |
| Pitch | Increase        | +    | Pitches forward         |
| Pitch | Decrease        | –    | Pitches backward        |
| Yaw   | Increase        | +    | Yaws right (CW)         |
| Yaw   | Decrease        | –    | Yaws left (CCW)         |

---

## Control Authority Budget

### Physical Capabilities

- **Total thrust capacity:** 160g (800 units) across 4 motors  
- **Drone mass:** 70g → Hover thrust: 70g (350 units)  
- **Maximum lateral acceleration:** 1.0g available  
- **Thrust-to-weight ratio:** 2.28:1

### Authority Allocation

- **Altitude control budget:** 90g total (450 units = hover + 20g margin)  
- **Stabilization control budget:** 70g (350 units = 100% of drone mass)  
- **Mixing margin:** 350 units (MOTOR_MAX – THRUST_MAX = 800 – 450)

### Performance & Safety

- **1.0g lateral acceleration** enables aggressive disturbance rejection  
- **PID output limits:** ±116.67 units/axis ensures worst-case (450 + 3×117 = 801) remains within motor saturation  
- **Integral clamp:** ±58.33 units (50% of PID limit) prevents windup from consuming control authority

### Design Advantage

- **Mass-equivalent stabilization budget** exceeds typical commercial allocations  
- **Conservative thrust allocation** ensures robust performance in turbulence  
- **Balanced control strategy** maximizes agility without risking motor saturation

---

## TrueMin RMSE Relay Autotune
Inspired by the Åström–Hägglund Relay Auto-Tuning Method

### Overview

TrueMin RMSE Relay Autotune for roll, pitch, and yaw using a balanced test bench. Based on the Åström–Hägglund relay method, with 2Hz relay excitation to induce controlled oscillations for system identification. Pure RMSE metric tracks the absolute minimum error for each stage. Tuning stops automatically when performance degrades by 20% after the minimum. All gains are safety-bounded.

### Features

- 3-stage tuning (P → D → I) with relay excitation at 2Hz
- Pure RMSE metric: finds absolute minimum error for each stage
- Best gain tracking: always selects the true minimum, not just "good enough"
- Smart stopping: stops when RMSE increases by 20% after the minimum
- Gain limits: maximum boundaries for all terms
- 20Hz evaluation: optimal for aircraft dynamics

### Tuning Process

1. Start from zero gains
2. For each axis, tune in 3 stages (P, D, I):
   - Increment gain in constant steps (see code)
   - At each gain, evaluate RMSE every 0.05s (20Hz)
   - Track the absolute minimum RMSE and best gain
   - Stop the stage when RMSE increases by 20% after the minimum
   - Set gain to the best (minimum RMSE) found
3. Repeat for roll, pitch, and yaw
4. Ramp down thrust after tuning

### RMS Error Metric

All decisions use:

$RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

where $e_i$ is the error at sample $i$, $N$ is the sample count.

## Results

- Tuning time: ~8–32 seconds per axis (depends on system response)
- Each stage stops when RMSE increases by 20% after the minimum
- Gains: Optimal, safe, and flyable
- Finds true minimum, not just first acceptable gain
- Minimal code, robust to non-monotonic response

---

## Safety & Robustness

- All PID states and setpoints are reset when the FCU is inactive
- Final motor outputs are always constrained to physical limits
- Failsafe and landing logic activate on packet timeout or power warning
- Auto-tune is aborted and state cleared if thrust input is modified during ramp or tuning

---

## Summary

This system enables **safe, hands-off PID gain estimation** for drones, delivering reliable initial gains for stable flight. Further manual tuning is recommended for optimal performance. The auto-tune process provides a robust starting point for agile and balanced control. The control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---

## 3D Model

- [PiiDrone X-314 Quad (Maker World)](https://makerworld.com/en/models/1153207-piidrone-x-314-quad)

---

## References
- [Relay Auto-Tuning (Åström–Hägglund) – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [PID Controller – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Root Mean Square Deviation (RMSE) – Wikipedia](https://en.wikipedia.org/wiki/Root_mean_square_deviation)
- [Smoothstep – Wikipedia](https://en.wikipedia.org/wiki/Smoothstep)
- [Quaternion – Wikipedia](https://en.wikipedia.org/wiki/Quaternion)

---