# PiiDrone X-314 Quad

**Ultra-light, agile, and stable powerhouse (70g including LiPo)**

---

## Motor Layout (X Configuration)

```
   Rear
     |
| -- | -- |
| M4 | M3 |
| -- | -- |
| M2 | M1 |
| -- | -- |
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
- **Altitude control:** 90g total (450 units = hover + 20g margin)
- **Stabilization control:** 70g (350 units = 100% of drone mass)
- **Mixing budget:** (MOTOR_MAX - THRUST_MAX) = 800 - 450 = 350 units

### Performance & Safety
- **1.0g lateral acceleration** enables aggressive disturbance rejection
- **PID limits:** ±116.67 units/axis ensures worst-case (450 + 3×117 = 801) stays within motor saturation limits
- **Integral clamp:** ±58.33 units (half of PID limits) prevents windup consuming control authority

### Design Advantage
- **100% mass-equivalent stabilization** exceeds typical commercial budgets
- **Conservative allocation** provides robust performance in turbulence
- **Balanced approach** optimizes control without risking motor saturation

---

## Ziegler-Nichols Auto-Tune System

### Key Features

- **Battery-friendly smoothstep ramp-up** to hover thrust before tuning for gentle motor activation
- **Single-phase PID tuning** using the classic Ziegler-Nichols oscillation method
- **Controlled oscillation induction** in each axis using 0.5 Hz square wave setpoints
- **Adaptive hysteresis-based zero-crossing detection** using a threshold set to one-third of the square wave amplitude
- **Oscillation analysis** to measure ultimate gain (Ku) and oscillation period (Tu)
- **PID gain calculation** using Ziegler-Nichols formulas:
  - `P = 0.6 × Ku`
  - `I = 1.2 × Ku / Tu`
  - `D = 0.075 × Ku × Tu`
- **Sequential axis tuning** with independent state tracking for roll, pitch, and yaw
- **Fast tuning completion** in approximately 8 seconds per axis
- **Safety-constrained gain limits** to prevent instability during or after tuning
- **Fallback logic** to apply conservative default gains if oscillations are not detected

### Tuning Process

1. **Thrust ramp-up** to stable hover
2. **Axis selection** (roll → pitch → yaw)
3. **Oscillation induction** via square wave excitation
4. **Gain calculation** using Ziegler-Nichols method
5. **Parameter application** and progression to next axis

---

## Safety & Robustness

- **All PID states, setpoints, and gains are reset** on abort or disarm
- **Final motor outputs are always constrained** to physical limits
- **Failsafe and landing logic** on packet timeout or power warning
- **Auto-tune aborts** if thrust is changed during ramp-up or tuning, resetting all tuning state

## Source
- [Ziegler–Nichols method (Wikipedia)](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
---

## Summary

## Summary

This system enables **safe, hands-off PID tuning** for drones, delivering reliable initial gain estimates for stable flight.<br><br>While further fine-tuning **is needed** for optimal performance, the auto-tune process provides a strong starting point for agile and balanced control.<br><br>The control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---