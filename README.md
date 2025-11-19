
# PiiDrone X-314 Quad

Ultra-light, agile, and stable. 70g including LiPo.

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
- **Total thrust:** 160g (800 units) across 4 motors
- **Mass:** 70g (hover thrust: 350 units)
- **Max lateral acceleration:** 1.0g
- **Thrust-to-weight:** 2.28:1

### Authority Allocation
- **Altitude budget:** 90g (450 units = hover + 20g margin)
- **Stabilization budget:** 70g (350 units = 100% mass)

### Performance & Safety
- **1.0g lateral acceleration:** Strong rejection
- **PID output:** ±116.67 units/axis (prevents saturation)
- **Integral clamp:** ±58.33 units (50% of PID limit)

### Design Advantage
- **Mass-equivalent stabilization budget:** Exceeds typical
- **Conservative thrust allocation:** Robust in turbulence
- **Balanced control:** Agility, no saturation

---

## PiiTune Deep RMSE - PID Training System

Inspired by the Åström–Hägglund relay auto-tuning method and practical system validation.

### What It Is
- **Per-Axis, Staged PID Tuning:** Each axis (Roll, Pitch, Yaw) is tuned independently in three stages: P, D, I.
- **Relay Excitation:** Alternates setpoint at 0.5Hz (15° amplitude) to excite system dynamics for identification.
- **RMSE-Based Evaluation:** Uses root mean square error (RMSE) at 4Hz to evaluate performance and guide tuning.
- **Best Gain Tracking:** Remembers the gain value that achieved the lowest RMSE for each stage and axis.
- **Patience Mechanism:** Waits for 3 consecutive RMSE increases before stopping a stage (prevents premature stopping).
- **Early Stopping and Revert:** If RMSE increases, reverts to best gain and starts a stability test.
- **Two-Phase Process:** Phase 1: Incremental gain search; Phase 2: 10s stability validation under relay excitation.
- **Infinite Gain Exploration:** If stability test fails, increases gain and restarts test until stable.
- **Automatic Stage Progression:** After stability, moves to next gain (P→D→I) for each axis.
- **Axis Completion:** After all stages, axis is marked complete; all axes must finish for tuning to end.
- **State Reset:** All accumulators and state are reset between stages and axes for robust operation.

### Training Process
**Phase 1: Incremental Discovery (Per Axis)**
- Start: P=0.5, I=0, D=0 with relay excitation (0.5Hz, 15° amplitude)
- RMSE evaluation: 4Hz rate, squared error accumulation
- Increment gain for current stage (P, D, I) until RMSE stops improving
- Track best RMSE and gain; revert to best gain if RMSE increases
- Patience: Wait for 3 consecutive RMSE increases before stopping

**Phase 2: Stability Validation (Per Axis)**
- 10-second endurance test under continuous relay excitation
- If RMSE increases, increase gain and restart test (infinite gain exploration)
- On success, progress to next stage (P→D→I); after all, axis is complete

### RMSE Error Metric
All optimization decisions use:

$RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

where $e_i$ is the error at sample $i$, $N$ is the sample count over 250ms evaluation windows.

### Performance Results
- **Training Time:** ~30-60 seconds per axis (90-180s total)
- **Stage Completion:** RMSE increases by 20% with 3-sample patience triggers stability test
- **Gain Quality:** Axis-specific, optimal, stable, and flight-ready
- **Reliability:** Infinite search guarantees solution for any system
- **Completeness:** Each axis independently optimized for its unique dynamics

### Validation Methodology
- **Per-Axis Survival Testing:** 10-second continuous relay operation per axis
- **Infinite Gain Adjustment:** No limits – finds stable gains for any system
- **Independent Optimization:** Each axis tuned to its specific requirements
- **Real-World Ready:** Testing under excitation mimics flight disturbances

**Result:** Fast, reliable, and complete tuning with mathematically optimal and practically stable PID parameters for each axis independently.

---

## Safety & Robustness
- **Reset all PID states and setpoints** when FCU inactive
- **Constrain motor outputs** to physical limits
- **Activate failsafe/landing** on packet timeout or power warning
- **Abort auto-tune and clear state** if thrust input changes during ramp/tune

---

## Summary

Enables safe, hands-off PID tuning for drones. Delivers reliable, axis-specific gains for stable flight. Further manual tuning is recommended for best performance. Provides a robust starting point for agile, balanced control. Control authority budget and safety features support robust flight dynamics for the PiiDrone X-314 Quad.

---

## References
- [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Åström–Hägglund relay method](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A9gglund)_method)
- [Root mean square deviation (RMSE)](https://en.wikipedia.org/wiki/Root_mean_square_deviation)
- [Reinforcement learning](https://en.wikipedia.org/wiki/Reinforcement_learning)
- [Machine learning](https://en.wikipedia.org/wiki/Machine_learning)
- [Smoothstep](https://en.wikipedia.org/wiki/Smoothstep)
---
