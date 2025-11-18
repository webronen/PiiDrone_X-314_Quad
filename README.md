# PiiDrone X-314 Quad

Ultra-light, agile, stable. 70g including LiPo.

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

| Axis  | Setpoint Change | Sign | Response        |
|-------|-----------------|------|-----------------|
| Roll  | Increase        | +    | Roll right      |
| Roll  | Decrease        | –    | Roll left       |
| Pitch | Increase        | +    | Pitch forward   |
| Pitch | Decrease        | –    | Pitch backward  |
| Yaw   | Increase        | +    | Yaw right (CW)  |
| Yaw   | Decrease        | –    | Yaw left (CCW)  |

---

## Control Authority Budget

### Physical Capabilities

- Total thrust: 160g (800 units) across 4 motors
- Mass: 70g (hover thrust: 350 units)
- Max lateral acceleration: 1.0g
- Thrust-to-weight: 2.28:1

### Authority Allocation

- Altitude budget: 90g (450 units = hover + 20g margin)
- Stabilization budget: 70g (350 units = 100% mass)

### Performance & Safety

- 1.0g lateral acceleration (strong rejection)
- PID output: ±116.67 units/axis (prevents saturation)
- Integral clamp: ±58.33 units (50% of PID limit)

### Design Advantage

- Mass-equivalent stabilization budget (exceeds typical)
- Conservative thrust allocation (robust in turbulence)
- Balanced control (agility, no saturation)

---

## PiiTune Deep RMSE - PID Training System

Inspired by the Åström–Hägglund relay auto-tuning method and modern machine learning.

**What It Is:**
- PID Training System: Learns optimal gains through system interaction
- Deep RMSE Optimization: High-frequency error analysis drives learning
- Reinforcement Learning: Environment interaction with performance-based updates
- Adaptive Control: Discovers system-specific optimal parameters

### Key Features

- Learns optimal PID gains through relay excitation
- Trains using RMSE as performance metric
- Prevents overfitting with 20% tolerance and patience
- Finds global optimum through multi-stage exploration (P → D → I)
- Adapts to specific drone dynamics

### Training Process

1. System excitation via relay oscillations (0.5Hz, 15° amplitude)
2. Performance measurement through deep RMSE analysis (4Hz)
3. Parameter updates based on performance feedback
4. Multi-stage learning (P → D → I)
5. Validation through overfitting prevention (20% tolerance, 3-sample patience)

### Technical Foundation

- Reinforcement Learning: Environment + Agent + Reward system
- Online Learning: Real-time parameter updates
- Gradient-Free Optimization: Robust to non-convex landscapes
- System Identification: Learns drone dynamics through interaction

### RMSE Error Metric

All decisions use:

$RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

where $e_i$ is the error at sample $i$, $N$ is the sample count.

## Results

- Tuning: ~8–32s per axis (depends on response)
- Each stage stops when RMSE increases by 20% after minimum (with patience)
- Gains: optimal, safe, flyable
- Finds true minimum, not just first acceptable
- Minimal code, robust to non-monotonic response

---

## Safety & Robustness

- Reset all PID states and setpoints when FCU inactive
- Constrain motor outputs to physical limits
- Activate failsafe/landing on packet timeout or power warning
- Abort auto-tune and clear state if thrust input changes during ramp/tune

---

## Summary

Enables safe, hands-off PID tuning for drones. Delivers reliable initial gains for stable flight. Further manual tuning is recommended for best performance. Provides a robust starting point for agile, balanced control. Control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---

## References

- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Smoothstep](https://en.wikipedia.org/wiki/Smoothstep)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [Root mean square deviation (RMSE)](https://en.wikipedia.org/wiki/Root_mean_square_deviation)
- [Machine learning](https://en.wikipedia.org/wiki/Machine_learning)

---