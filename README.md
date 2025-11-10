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

## Ziegler-Nichols Auto-Tune System

### Key Features

- **Battery-friendly smoothstep ramp-up** to hover thrust before tuning for gentle motor activation
- **Single-phase PID tuning** using the classic Ziegler-Nichols oscillation method
- **Controlled oscillation induction** in each axis using 0.5 Hz square wave setpoints
- **Adaptive zero-crossing detection with hysteresis** based on the square wave amplitude
- **Oscillation analysis** to measure ultimate gain (Ku) and oscillation period (Tu)
- **PID gain calculation** using Ziegler-Nichols formulas:
  - `P = 0.6 × Ku`
  - `I = 1.2 × Ku / Tu`
  - `D = 0.075 × Ku × Tu`
- **Sequential axis tuning** with independent state tracking for roll, pitch, and yaw
- **Fast tuning completion** in approximately 8 seconds per axis (2.5 periods at 0.5 Hz)
- **Safety-constrained gain limits** to prevent instability during or after tuning
- **Fallback logic** to apply conservative default gains if oscillations are not detected

### Tuning Process

1. **Thrust ramp-up** to stable hover  
2. **Axis selection** (roll → pitch → yaw)  
3. **Oscillation triggering** using 0.5 Hz square wave setpoints  
4. **Gain calculation** using Ziegler-Nichols method  
5. **Apply calculated gains** and continue to the next axis

---

## Safety & Robustness

- **All PID states, setpoints, and gains are reset** when the FCU is inactive  
- **Final motor outputs are always constrained** to physical limits  
- **Failsafe and landing logic** activate on packet timeout or power warning  
- **Auto-tune is aborted and state cleared** if thrust input is modified during ramp-up or tuning

---

## Summary

This system enables **safe, hands-off PID tuning** for drones, delivering reliable initial gain estimates for stable flight. While further fine-tuning **is needed** for optimal performance, the auto-tune process provides a strong starting point for agile and balanced control. The control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---

## 3D Model
- [PiiDrone X-314 Quad (Maker World)](https://makerworld.com/en/models/1153207-piidrone-x-314-quad)

---

## Source
- [Quaternion (Wikipedia)](https://en.wikipedia.org/wiki/Quaternion)
- [Smoothstep (Wikipedia)](https://en.wikipedia.org/wiki/Smoothstep)
- [Ziegler–Nichols method (Wikipedia)](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)

---