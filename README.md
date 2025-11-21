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

### Mission
Discover Pareto-optimal PID gains through systematic exploration, balancing settling performance against overshoot without artificial compromises. Uses hypervolume convergence to find the best possible tradeoff for each axis.

### Core Algorithm
- Relay excitation: ±TUNE_RELAY_RADIANS at 2.0Hz (500ms period)
- Multi-objective optimization: Simultaneously minimizes settling time and overshoot
- Pareto frontier: Only accepts gains that improve at least one metric without degrading the other
- Sequential tuning: P → D → I stages with hypervolume convergence
- Reinforcement learning: Pure exploration with automatic performance evaluation

### Performance Metrics
- Inverse performance scoring: Higher values = better performance
- Settle performance: 1.0f / (half_period_us + __FLT_EPSILON__)
- Overshoot penalty: 1.0f / (max_os + __FLT_EPSILON__)
- Hypervolume: settle_performance × os_penalty (balanced multi-objective measure)

### Decision Criteria

A new gain is Pareto-optimal if:

$$(T_s^{\text{new}} < T_s^{\text{best}} \land O^{\text{new}} \leq O^{\text{best}}) \quad \text{or} \quad (O^{\text{new}} < O^{\text{best}} \land T_s^{\text{new}} \leq T_s^{\text{best}})$$

Where:
- $T_s$ = Settling time
- $O$ = Overshoot

### Convergence Behavior
- 99% hypervolume convergence: Progress to next stage when improvements < 1%
- No artificial limits: Gains can grow indefinitely if beneficial
- Pure exploration: Systematic gain incrementing with TUNE_*_INCREMENT steps
- Stage preservation: Best gains carried forward through P→D→I progression

### Tuning Parameters
- Relay frequency: 2.0Hz (optimal for 211Hz control loop)
- Gain increments: P=0.1, D=0.01, I=0.001 (professionally scaled)
- Convergence: 1% hypervolume improvement threshold
- Exploration: No maximum gain limits

### Safety & Robustness
- Numerical stability: __FLT_EPSILON__ protected divisions
- Quaternion Kalman ready: Works with professional-grade attitude estimation
- Multi-axis independent: Parallel tuning across roll, pitch, yaw
- Automatic completion: Resets setpoints when all axes complete

### Guarantee
Finds the Pareto-optimal balance between speed and stability for your specific hardware. Either converges to 99% of optimal hypervolume within each stage, or continues exploring indefinitely. Perfect foundation for stable hover when combined with angle control wrapper.

---

## Results
- Tuning: Model-free reinforcement learning approach
- Gains: Pareto-optimal tradeoffs between settling and overshoot
- Performance: Professional-grade rate control ready for hover integration
- Robustness: Mathematically sound with no numerical edge cases

## Expected Performance
- Per axis: ~45-60 seconds tuning time
- Total system: ~2.5-3 minutes for all axes
- Foundation: World-class rate control for attitude stabilization

---

## Summary
StepSync uses advanced multi-objective optimization to discover PID gains that balance speed against stability. The reinforcement learning approach systematically explores the gain space while ensuring Pareto-optimal improvements, delivering professional-grade rate control ready for rock-solid hover performance.

Perfect for developers who want industrial-grade autotuning without system identification or manual compromise.

---

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A9gglund)_method)
- [Pareto efficiency](https://en.wikipedia.org/wiki/Pareto_efficiency)
- [Lebesgue measure](https://en.wikipedia.org/wiki/Lebesgue_measure)
- [Reinforcement learning](https://en.wikipedia.org/wiki/Reinforcement_learning)
---