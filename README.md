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

## Astrom-Hägglund Relay Auto-Tuning System

### Overview

Generates safe, flyable PID gains for roll, pitch, and yaw axes using a balanced test bench setup. Relay excitation is applied to induce controlled oscillations, allowing precise measurement of system dynamics and minimizing root mean square error (RMSE). This method provides a reliable baseline for manual tuning, with real-time safety constraints in place to maintain stable flight behavior during initial testing.

### Features

- Smoothstep thrust ramp for gentle, battery-friendly motor activation
- Three-stage PID tuning with relay excitation and staged gain adjustment
- Unified RMSE metric for all stages
- Best-value tracking: each stage keeps the gain with the lowest RMSE
- Automatic stage transitions when RMSE goals are met or gain limits are reached
- Independent tuning for roll, pitch, and yaw
- Safety-constrained gain limits and robust fallback to safe defaults
- Timer overflow protection and static state reset

### Tuning Process

1. Ramp up thrust smoothly to hover.
2. Tune each axis in three stages using relay excitation:
  - **P stage:** 0 → 15.0 (step 0.5), advance if RMSE < 0.12 (~6.9°) or max.
  - **D stage:** 0 → 3.0 (step 0.2), advance if RMSE < 0.08 (~4.6°) or max.
  - **I stage:** 0 → 2.0 (step 0.1), finish if RMSE < 0.06 (~3.4°) or max.
3. At the end of each stage, set the gain to the best value found (lowest RMSE).
4. Repeat for roll, pitch, and yaw axes.
5. Ramp down thrust smoothly after tuning.

## Results

- Tuning time: ~8–32 seconds per axis (depends on system response).
- Stage progression: Ends when RMSE target or gain limit is reached.
- Gains: Performance-based selection for a safe, flyable starting point.
- Robustness: RMSE-based approach is immune to noise and zero-crossing issues.
- Early completion: Can finish in as few as 8 relay flips if performance targets are met quickly.
- Actual: Each stage ends when RMSE goal or gain limit is reached.

### Fallback Behavior

If tuning exceeds maximum gain limits, fallback gains are applied:

- `P = 0.0`
- `I = 0.0`
- `D = 0.0`

---

## Safety & Robustness

- **All PID states and setpoints are reset** when the FCU is inactive  
- **Final motor outputs are always constrained** to physical limits  
- **Failsafe and landing logic** activate on packet timeout or power warning  
- **Auto-tune is aborted and state cleared** if thrust input is modified during thrust ramp or tuning

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