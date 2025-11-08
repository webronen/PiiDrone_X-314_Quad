# PiiDrone X-314 Quad

**Ultra-light, agile, and stable powerhouse (70g including LiPo)**

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

| Axis  | Setpoint Change | Sign | Expected Drone Response   |
|-------|-----------------|------|--------------------------|
| Roll  | Increase        |  +   | Rolls right              |
| Roll  | Decrease        |  –   | Rolls left               |
| Pitch | Increase        |  +   | Pitches forward          |
| Pitch | Decrease        |  –   | Pitches backward         |
| Yaw   | Increase        |  +   | Yaws right (CW)          |
| Yaw   | Decrease        |  –   | Yaws left (CCW)          |

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
- **Battery-friendly ramp-up** to hover thrust before tuning begins
- **Automatically induces controlled oscillations** in a selected axis by generating square wave setpoint changes
- **Detects sustained oscillations** to measure the ultimate gain (Ku) and oscillation period (Tu) for that axis
- **Calculates optimal PID gains** using the classic Ziegler-Nichols tuning rules, with axis-specific scaling if needed
- **Applies the new gains and repeats** for each axis (roll, pitch, yaw) in sequence
- **Tuning stops automatically** after the required number of oscillations or if a safety fallback is triggered

---

## Safety & Robustness

- **All PID states, setpoints, and gains are reset** on abort or disarm
- **Final motor outputs are always constrained** to physical limits
- **Failsafe and landing logic** on packet timeout or power warning
- **Auto-tune aborts** if thrust is changed during ramp-up or tuning, resetting all tuning state

---

## Summary

This system enables **safe, hands-off PID tuning** for drones, ensuring robust and balanced flight control.  
The control authority budget and safety features support agile and stable flight for the PiiDrone X-314 Quad.

---