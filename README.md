License: CC BY-NC 4.0
- https://creativecommons.org/licenses/by-nc/4.0/legalcode

# PiiDrone X-314 Quad
Ultra-light, agile, and stable. 70g including LiPo.

## Motor Layout (X Configuration)
```
   Rear
     o
| -- | -- |
| M4 | M3 |
| -- | -- |
| M2 | M1 |
| -- | -- |
     v
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

## PiiTune RL‑314 PID – Hypervolume Performance Explorer
Where Reinforcement Learning meets Control Theory to deliver professional results without professional expertise.

### Mission
Discover hypervolume-optimal PID gains for direct hover stabilization through systematic exploration, balancing settling performance against overshoot without artificial compromises. Uses pure relative metrics and hypervolume convergence to find the best possible tradeoff for each axis.

### Core Algorithm
The optimizer uses relay excitation and staged PID gain tuning for each axis. Gains are incremented systematically, and each candidate is evaluated using settling time and overshoot. All decisions are made using pure relative comparisons with a single 10% threshold. The combined performance is measured by the hypervolume metric, and only gains that improve this metric are accepted. Convergence is determined by relative hypervolume improvement, with best gains preserved and tuning proceeding automatically across axes.

### Performance Metrics
- Settle performance: Inverse of relative settling time (higher = better)
- Overshoot penalty: Inverse of relative maximum overshoot (higher = better)
- Hypervolume: Product of relative settle performance and overshoot penalty (balanced multi-objective measure)

### Decision Criteria
A new gain is accepted if it improves the hypervolume metric by at least 10%:

$`\frac{|\text{Hypervolume}^{\text{new}} - \text{Hypervolume}^{\text{prev}}|}{\text{Hypervolume}^{\text{prev}}} \geq 0.10`$

Where:

$`\text{Hypervolume} = (1 - \min(\text{settling time} / \text{relay period}, 1)) \times (1 - \min(\text{overshoot} / \text{relay amplitude}, 1))`$

Settling detection is performed using pure relative error change:

$`\frac{|\text{err} - \text{prev\_err}|}{|\text{prev\_err}|} \leq 0.10`$

Non-responsive systems are penalized if hypervolume $< 0.10$.

No absolute thresholds are used, everything is relative to the previous state.

### Convergence Behavior
- 90% hypervolume convergence: Progresses to next stage when improvements are less than or equal to 10%
- No artificial limits: Gains can grow indefinitely if beneficial
- Pure exploration: Systematic gain incrementing with fixed steps
- Stage preservation: Best gains carried forward through P→D→I progression

### Tuning Parameters
- Relay frequency: 0.5Hz (2s period, optimized for control loop)
- Gain increments: P=0.1, D=0.01, I=0.001
- All thresholds: 10% relative change
- Exploration: No maximum gain limits

### Safety & Robustness
- Numerical stability: Protected divisions
- Multi-axis independent: Sequential tuning across roll, pitch, yaw
- Automatic completion: Resets setpoint when axis completes
- Oscillation validation:
  - Uses zero-crossing counting (≥2 crossings required) to confirm true oscillation
  - Applies a uniform penalty to non-responsive systems to prevent false convergence

### Guarantee
Aims to find the hypervolume-optimal balance between speed and stability for your hardware. Either converges to 90% of optimal hypervolume within each stage, or continues exploring indefinitely. Provides directly tuned control suitable for stable flight.

---

## Expected Performance
- Each stage (P, D, I): ~30 seconds
- Each axis: 3 stages × 30 seconds = 90 seconds
- Entire system: 3 axes × 90 seconds = 270 seconds (4.5 minutes)
- Output: Directly tuned control for stable flight

Timing may vary based on system dynamics and hardware characteristics.  
Typical range: 2–6 minutes total depending on plant complexity and convergence.

---

## Summary
PiiTune RL‑314 PID uses pure relative hypervolume optimization to discover PID gains that balance speed and stability. The reinforcement learning approach systematically explores the gain space while ensuring continuous improvements, delivering tuned control suitable for stable flight performance.

Suitable for developers who want automated tuning without system identification or manual compromise.

---

## 3D Model
- [PiiDrone X-314 Quad](https://makerworld.com/en/models/1153207-piidrone-x-314-quad)

---

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [Multi-objective optimization](https://en.wikipedia.org/wiki/Multi-objective_optimization)
- [Reinforcement learning](https://en.wikipedia.org/wiki/Reinforcement_learning)
- [Lebesgue measure](https://en.wikipedia.org/wiki/Lebesgue_measure)

---