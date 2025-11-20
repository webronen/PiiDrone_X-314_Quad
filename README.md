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

**Physical Capabilities**
- Total thrust: 160g (800 units) across 4 motors
- Mass: 70g (hover thrust: 350 units)
- Max lateral acceleration: 1.0g
- Thrust-to-weight: 2.28:1

**Authority Allocation**
- Altitude budget: 90g (450 units = hover + 20g margin)
- Stabilization budget: 70g (350 units = 100% mass)

**Performance & Safety**
- 1.0g lateral acceleration (strong rejection)
- PID output: ±116.67 units/axis (prevents saturation)
- Integral clamp: ±58.33 units (50% of PID limit)

**Design Advantage**
- Mass-equivalent stabilization budget (exceeds typical)
- Conservative thrust allocation (robust in turbulence)
- Balanced control (agility, no saturation)

## PiiTune StepSync – Adaptive PID Tuning System
Inspired by relay auto-tuning methods and Pareto multi-objective optimization, PiiTune StepSync performs per-axis PID tuning using relay excitation and Pareto-optimized step response analysis, sequentially tuning P, D, and I gains while balancing settling time against overshoot.

### What It Is
- Adaptive PID Tuning: Learns optimal gains through step response analysis
- Pareto Optimization: Minimizes settling time and overshoot together
- Multi-Objective Learning: Balances speed and stability by direct measurement
- System-Specific Adaptation: Discovers unique drone dynamics and optimal parameters

### Key Features
- Step response optimization: Directly measures and optimizes flight performance
- Pareto frontier exploration: Finds best trade-off between settling time and overshoot
- Staged learning: Systematic progression through P → D → I
- Immediate convergence: Stops each stage as soon as no Pareto improvement is found
- Fast tuning: No waiting for confirmation samples—finds optimal gains faster
- Clean, simple logic: Less state to track, easier to understand
- Real flight correlation: Tunes for actual pilot experience, not just math

### Training Process
- System excitation: Relay oscillations (0.5Hz, 15° amplitude) with half-cycle timing
- Performance measurement: Direct step response analysis (settling time + overshoot)
- Pareto optimization: Multi-objective improvement tracking
- Staged learning: Isolated parameter tuning (P → D → I)
- Immediate completion: Each stage stops as soon as no further Pareto improvement is detected

### Technical Foundation
- Pareto optimization: Multi-objective improvement, no artificial weighting
- Step response analysis: Direct measurement of flight-relevant performance
- Staged parameter isolation: Clean separation of P, D, I effects
- Real-time adaptation: Continuous performance feedback and adjustment

### Performance Metrics
All optimization decisions use direct flight performance measurements:
- Settling time: Time to reach and stay within 0.5° of target
- Overshoot: Maximum angle exceeded beyond target
- Pareto improvement: Better in one metric, equal or better in the other

#### Pareto Optimization Decision Formula
A new gain is accepted if it is not worse in either metric and better in at least one:

$$(T_s^{\text{new}} < T_s^{\text{best}} \land O^{\text{new}} \leq O^{\text{best}}) \quad \text{or} \quad (O^{\text{new}} < O^{\text{best}} \land T_s^{\text{new}} \leq T_s^{\text{best}})$$

Where:
- $T_s$ = Settling time
- $O$ = Overshoot

## Results
- Tuning time: ~30–90 seconds per axis (convergence-based)
- Performance: Optimal balance of speed and stability for flight
- Gains: Flight-ready, stable, responsive
- Convergence: Finds true performance boundary, not arbitrary stopping point
- Robustness: Handles non-ideal responses and measurement noise

## Safety & Robustness
- Automatic setpoint reset: Zero commands when tuning complete
- Stage isolation: Clean parameter separation prevents interference
- Bounds protection: Array bounds checking and stage overflow protection
- Timing safety: Half-cycle offset ensures proper measurement timing
- Graceful degradation: Handles unsettled systems and edge cases

## Flight Performance Targets
Rock-solid hover profile:
- Roll/Pitch: <200ms settling, <1.0° overshoot
- Yaw: <250ms settling, <0.5° overshoot

Stable, accurate, and fully autonomous hover.

## Summary
Enables safe, hands-off PID tuning for drones. Delivers reliable, flight-ready gains for stable flight. Further manual tuning is recommended for best performance. Provides a robust starting point for agile, balanced control. Control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A9gglund)_method)
- [Pareto efficiency](https://en.wikipedia.org/wiki/Pareto_efficiency)
---