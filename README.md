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

Inspired by relay auto-tuning and Pareto optimization. Per-axis PID tuning uses relay excitation and step response analysis, tuning P, D, I gains to meet settling time and overshoot targets.

### Features
- Per-axis, staged tuning (P → D → I)
- Relay excitation (0.5Hz, 15°)
- Step response: settling time + overshoot
- Pareto optimization: accepts only gains that improve at least one metric
- Immediate stop: ends stage when no further improvement or targets are met
- Target-based: stops when axis-specific targets are reached
- Smart fallback: uses best gains if targets not met
- Fast, simple, robust

### Training Process
- Excite axis with relay
- Measure step response
- Tune each gain until no further Pareto improvement or targets met

### Decision Formula
A new gain is accepted if:
$$(T_s^{\text{new}} < T_s^{\text{best}} \land O^{\text{new}} \leq O^{\text{best}}) \quad \text{or} \quad (O^{\text{new}} < O^{\text{best}} \land T_s^{\text{new}} \leq T_s^{\text{best}})$$
Where $T_s$ = settling time, $O$ = overshoot.

### Target-Based Stopping
- Roll/Pitch: ≤200ms settling, ≤1.0° overshoot
- Yaw: ≤250ms settling, ≤0.5° overshoot
- If targets not met, use best gains found

---

## Results
- Tuning: ~30–90s per axis
- Gains: Stable, responsive, and flight-ready
- Robust to noise and non-ideal responses

## Safety
- Resets setpoints when done
- Stage isolation and bounds checks
- Handles edge cases gracefully

## Flight Performance Targets
- Roll/Pitch: <200ms settling, <1.0° overshoot
- Yaw: <250ms settling, <0.5° overshoot
- Stable, accurate, hands-off hover

---

## Summary
StepSync autotune finds gains that meet exact flight targets for each axis—no guesswork, no endless searching. Delivers rock-solid hover and crisp stick response.

---

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A9gglund)_method)
- [Pareto efficiency](https://en.wikipedia.org/wiki/Pareto_efficiency)
---