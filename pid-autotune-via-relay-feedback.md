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

- Tuning time: ~32s per axis (24 relay flips, 3 stages).
- Gains are conservative, flyable, and safe for further tuning.
- No zero-crossing detection needed; robust to noise.

## References

- [Relay Auto-Tuning (Åström–Hägglund) – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [PID Controller – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Root Mean Square Deviation (RMSE) – Wikipedia](https://en.wikipedia.org/wiki/Root_mean_square_deviation)