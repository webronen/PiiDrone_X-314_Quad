# PiiDrone X-314 Quad
Ultra-light, agile, and stable. 70g including LiPo.

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
- M1: Front-right (CCW)
- M2: Front-left (CW)
- M3: Rear-right (CW)
- M4: Rear-left (CCW)

## Setpoint Response
| Axis  | Setpoint Change | Sign | Response       |
| ----- | --------------- | ---- | -------------- |
| Roll  | Increase        | +    | Roll right     |
| Roll  | Decrease        | –    | Roll left      |
| Pitch | Increase        | +    | Pitch forward  |
| Pitch | Decrease        | –    | Pitch backward |
| Yaw   | Increase        | +    | Yaw right (CW) |
| Yaw   | Decrease        | –    | Yaw left (CCW) |

## Control Authority

- Thrust: 160g (800 units) across 4 motors
- Mass: 70g (hover: 350 units)
- Max lateral accel: 1.0g
- Thrust-to-weight: 2.28:1
- Altitude budget: 90g (450 units)
- Stabilization budget: 70g (350 units)
- PID output: ±116.67 units/axis
- Integral clamp: ±58.33 units

---

## PiiTune StepSync – Adaptive PID Tuning System

### Mission
Find PID gains that deliver exact performance specs—no compromises, no fallbacks. Only accepts gains that meet both settling time and overshoot targets to deliver stable, precise, and reliable flight control.

### Core Algorithm
- Relay excitation: ±15° steps at 0.5Hz
- Step response analysis: Measures actual settling time and overshoot
- Strict targets check: Must achieve ≤200ms settling and ≤1.0° overshoot (roll/pitch)
- Sequential tuning: P → D → I stages, each must meet targets

### Performance Specs
- Roll/Pitch: 200ms settling, 1.0° max overshoot
- Yaw: 250ms settling, 0.5° max overshoot
- Conservative increments: P=0.3, D=0.15, I=0.001

### Decision Criteria

A new gain is accepted if it improves at least one metric (settling time or overshoot) and does not worsen the other:

$$(T_s^{\text{new}} < T_s^{\text{best}} \land O^{\text{new}} \leq O^{\text{best}}) \quad \text{or} \quad (O^{\text{new}} < O^{\text{best}} \land T_s^{\text{new}} \leq T_s^{\text{best}})$$

Where:
- $T_s$ = Settling time
- $O$ = Overshoot

### Search Behavior
- Targets met → Progress to next stage immediately
- Targets not met → Keep incrementing gains indefinitely
- Never settles → Continue searching with higher gains
- Manual stop → Pilot decides when to abort search

### Safety
- Gain clamping: Prevents runaway with PID_GAIN_MAX limits
- Stage isolation: Clean P→D→I progression
- Multi-axis: Independent roll, pitch, yaw tuning

### Guarantee
Either finds gains that deliver exactly 200ms/1.0° performance or keeps searching forever. No middle ground, no "good enough" compromises.

Perfect for anyone needing rock-solid hover and precise, reliable flight performance.

---

## Results
- Tuning: Strict, target-based, and uncompromising
- Gains: Only accepted if they meet exact specs
- Robust: No fallback—always searching for perfection

## Safety
- Resets setpoints when done
- Stage isolation and bounds checks
- Handles edge cases gracefully

## Flight Performance Targets
- Roll/Pitch: ≤200ms settling, ≤1.0° overshoot
- Yaw: ≤250ms settling, ≤0.5° overshoot
- Stable, accurate, hands-off hover

---

## Summary
StepSync autotune finds gains that meet exact flight targets for each axis—no guesswork, no compromises. Delivers rock-solid hover and crisp stick response, ready for demanding aerial work.

---

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A9gglund)_method)
- [Pareto efficiency](https://en.wikipedia.org/wiki/Pareto_efficiency)
---