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
|-------|-----------------|------|------------------------|
| Roll  | Increase        | +    | Rolls right            |
| Roll  | Decrease        | –    | Rolls left             |
| Pitch | Increase        | +    | Pitches forward        |
| Pitch | Decrease        | –    | Pitches backward       |
| Yaw   | Increase        | +    | Yaws right (CW)        |
| Yaw   | Decrease        | –    | Yaws left (CCW)        |

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

Automates the Ziegler-Nichols PID tuning method using relay feedback for embedded flight controllers.

### Features

- **Smoothstep thrust ramp** for battery-friendly, gentle motor activation
- **Single-phase PID tuning** via induced oscillation (relay/relay-feedback)
- **0.5 Hz square wave excitation** with adaptive hysteresis for robust zero-crossing detection
- **Sequential axis tuning:** roll → pitch → yaw, each with independent state
- **Safety-constrained gain limits** and fallback to conservative defaults if oscillation fails
- **Robust zero-crossing detection** with bias removal and hysteresis
- **Timer overflow and division protection** for reliable operation
- **Static state reset** for safe re-tuning or abort

### Tuning Process

1. **Smoothstep thrust ramp** to hover using `pid_thrust_ramp()`
2. **Axis excitation** with 0.5 Hz square wave setpoint (relay method)
3. **Gain scheduling:** P gain increases every 0.5s until oscillation is detected
4. **Oscillation analysis:**
   - 6 zero-crossings (2.5 periods) are measured
   - `Tu = (now - first_cross) * 4e-7f` (for 2.5 periods, timer in microseconds)
5. **Gain calculation:**
   - `P = 0.6 × Ku` (constrained)
   - `I = 1.2 × Ku / Tu` (constrained)
   - `D = 0.075 × Ku × Tu` (constrained)
6. **Repeat** for each axis (roll, pitch, yaw)
7. **Ramp down thrust** after tuning

### Performance

- ~8 seconds per axis (6 zero-crossings, 2.5 periods)
- Adaptive noise rejection via hysteresis
- Safe fallback and abort at any time

### Fallback Behavior

If oscillation fails (e.g., fewer than 6 zero-crossings or excessive gain), fallback gains are applied:

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

This system enables **safe, hands-off PID tuning** for drones, delivering reliable initial gain estimates for stable flight. While further fine-tuning **is needed** for optimal performance, the auto-tune process provides a strong starting point for agile and balanced control. The control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---

## 3D Model

- [PiiDrone X-314 Quad (Maker World)](https://makerworld.com/en/models/1153207-piidrone-x-314-quad)

---

## References

- [Quaternion – Wikipedia](https://en.wikipedia.org/wiki/Quaternion)  
- [Smoothstep – Wikipedia](https://en.wikipedia.org/wiki/Smoothstep)  
- [PID Controller – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)  
- [Relay Auto-Tuning (Åström–Hägglund) – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)  
- [Ziegler–Nichols Tuning Method – Wikipedia](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)

---