# Åström–Hägglund Relay Auto-Tuning (Embedded)

## Overview

Generates safe, flyable PID gains for roll, pitch, and yaw axes using a balanced test bench setup. Relay excitation is applied to induce controlled oscillations, allowing precise measurement of system dynamics and minimizing root mean square error (RMSE). This method provides a reliable baseline for manual tuning, with real-time safety constraints in place to maintain stable flight behavior during initial testing.

## Tuning Algorithm

- For each axis, tuning runs in 3 stages: P, D, I.
- Each stage uses relay excitation (setpoint alternates every 2s).
- At each gain value, collect squared error samples for 2s, compute RMS error.
- Track the gain with the lowest RMS error for each stage.

### Stages and Parameters

- **P stage:** 0 → 15.0 (step 0.5), advance if RMSE < 0.12 (~6.9°) or max.
- **D stage:** 0 → 3.0 (step 0.2), advance if RMSE < 0.08 (~4.6°) or max.
- **I stage:** 0 → 2.0 (step 0.1), finish if RMSE < 0.06 (~3.4°) or max.

### RMS Error Metric


All stages use the same RMSE:

$RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

where $e_i$ is the error at sample $i$, $N$ is the sample count.

### Implementation Notes

- Static structs per axis track stage, best values, and timing.
- Hardware timer ensures precise intervals and relay switching.
- Gains are bounded; fallback to zero if tuning fails or limits are hit.
- Each axis is tuned independently.
- No dynamic memory allocation.


## Results

- Tuning time: ~8–32 seconds per axis (depends on system response).
- Stage progression: Advances when either the RMSE target is met or the gain reaches its maximum (whichever comes first).
- Gains: Performance-based selection for a safe, flyable starting point.
- Robustness: RMSE-based approach is immune to noise and zero-crossing issues.
- Early completion: Can finish in as few as 8 relay flips if performance targets are met quickly.
- Actual behavior: The auto-tuner completes each stage when either the RMSE goal is reached or the gain limit is hit, ensuring safe termination regardless of system response.

## References

- [Relay Auto-Tuning (Åström–Hägglund) – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [PID Controller – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Root Mean Square Deviation (RMSE) – Wikipedia](https://en.wikipedia.org/wiki/Root_mean_square_deviation)