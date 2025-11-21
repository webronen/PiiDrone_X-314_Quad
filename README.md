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
- Max lateral acceleration: 1.0g
- Thrust-to-weight: 2.28:1
- Altitude budget: 90g (450 units)
- Stabilization budget: 70g (350 units)
- PID output: ±116.67 units/axis
- Integral clamp: ±58.33 units

---

## PiiTune StepSync – Adaptive PID Tuning System
Where Reinforcement Learning meets Control Theory to deliver professional results without professional expertise.

### Mission
Discover hypervolume-optimal PID gains for direct hover stabilization through systematic exploration, balancing settling performance against overshoot without artificial compromises. Uses hypervolume convergence to find the best possible tradeoff for each axis.

### Core Algorithm
- Relay excitation: ±11.5° at 0.5Hz (2s period)
- Multi-objective optimization: Simultaneously optimizes settling time and overshoot via hypervolume
- Hypervolume frontier: Only accepts gains that improve the combined performance metric
- Sequential tuning: P → D → I stages with hypervolume convergence
- Reinforcement learning: Pure exploration with automatic performance evaluation

### Performance Metrics
- Settle performance: Inverse of settling time (higher = better)
- Overshoot penalty: Inverse of maximum overshoot (higher = better)
- Hypervolume: Product of settle performance and overshoot penalty (balanced multi-objective measure)

### Decision Criteria

A new gain is accepted if it improves the hypervolume metric:

$`\text{Hypervolume}^{\text{new}} > \text{Hypervolume}^{\text{best}}`$

Where:

$`\text{Hypervolume} = \frac{1}{T_s} \times \frac{1}{O}`$

$`T_s`$ = Settling time (lower is better, faster)  
$`O`$ = Overshoot (lower is better, smaller)

Higher hypervolume means both faster settling and smaller overshoot.

### Convergence Behavior
- 99% hypervolume convergence: Progresses to next stage when improvements are less than 1%
- No artificial limits: Gains can grow indefinitely if beneficial
- Pure exploration: Systematic gain incrementing with fixed steps
- Stage preservation: Best gains carried forward through P→D→I progression

### Tuning Parameters
- Relay frequency: 0.5Hz (2s period, optimized for control loop)
- Gain increments: P=0.1, D=0.01, I=0.001
- Settling threshold: 0.01 radians precision (~0.57°)
- Convergence: 1% hypervolume improvement threshold
- Exploration: No maximum gain limits

### Safety & Robustness
- Numerical stability: Protected divisions
- Multi-axis independent: Parallel tuning across roll, pitch, yaw
- Automatic completion: Resets setpoints when all axes complete

### Guarantee
Aims to find the hypervolume-optimal balance between speed and stability for your hardware. Either converges to 99% of optimal hypervolume within each stage, or continues exploring indefinitely. Provides directly tuned control suitable for stable flight.

---

## Expected Performance
- Per axis: ~2.5 minutes
- Total system: ~7.5 minutes
- Provides: Directly tuned control for stable flight

---

## Summary
StepSync uses hypervolume optimization to discover PID gains that balance speed and stability. The reinforcement learning approach systematically explores the gain space while ensuring continuous improvements, delivering tuned control suitable for stable flight performance.

Suitable for developers who want automated tuning without system identification or manual compromise.

---

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A9gglund)_method)
- [Hypervolume indicator](https://en.wikipedia.org/wiki/Hypervolume_indicator)
- [Multi-objective optimization](https://en.wikipedia.org/wiki/Multi-objective_optimization)
- [Reinforcement learning](https://en.wikipedia.org/wiki/Reinforcement_learning)
---