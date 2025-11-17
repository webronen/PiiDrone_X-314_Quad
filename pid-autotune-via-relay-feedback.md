# Åström–Hägglund Relay Auto-Tuning (Embedded)

## Overview

Automatically finds safe, flyable PID gains for roll, pitch, and yaw using a balanced test bench. Relay excitation creates controlled oscillations, enabling precise measurement of system response and minimizing RMSE. The result is a reliable starting point for manual tuning, with real-time safety limits for stable initial tests.

## Tuning Algorithm

- Each axis is tuned in 3 stages: P, D, I.
- Relay excitation (setpoint alternates every 2s) induces oscillation.
- For each gain, collect squared error samples for 2s and compute RMSE.
- The best gain (lowest RMSE) is tracked for each stage.

### Stages and Parameters

- **P:** 0 → 15.0 (step 0.5), next if RMSE < 0.12 (~6.9°) or max.
- **D:** 0 → 3.0 (step 0.2), next if RMSE < 0.08 (~4.6°) or max.
- **I:** 0 → 2.0 (step 0.1), done if RMSE < 0.06 (~3.4°) or max.

### RMS Error Metric

All stages use:

$RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

where $e_i$ is the error at sample $i$, $N$ is the sample count.

## Results

- Tuning time: ~8–32 seconds per axis (depends on system response)
- Stage ends when RMSE target or gain limit is reached
- Gains: Safe, flyable starting point
- Robust: Immune to noise and zero-crossing issues
- Can finish in as few as 8 relay flips if targets are met quickly

## References

- [Relay Auto-Tuning (Åström–Hägglund) – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [PID Controller – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Root Mean Square Deviation (RMSE) – Wikipedia](https://en.wikipedia.org/wiki/Root_mean_square_deviation)