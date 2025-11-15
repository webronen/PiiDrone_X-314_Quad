# Åström–Hägglund Relay Auto-Tuning (Embedded)

## Overview
Auto-tunes PID gains for roll, pitch, and yaw on a 70g quadcopter (nRF MCU). No manual tuning needed. Real-time, safe, and efficient.

## Tuning Algorithm

- For each axis, tuning runs in 3 stages: P, D, I.
- Each stage uses relay excitation (setpoint alternates every 2s).
- At each gain value, collect squared error samples for 2s, compute RMS error.
- Track the gain with the lowest RMS error for each stage.

### Stages and Parameters

- **P stage:** Start at 0, increment by 0.5 up to 15.0. Advance if RMS < 0.12 or max.
- **D stage:** Start at 0, increment by 0.2 up to 3.0. Advance if RMS < 0.08 or max.
- **I stage:** Start at 0, increment by 0.1 up to 2.0. Finish if RMS < 0.06 or max.

### RMS Error Metric

All stages use the same RMS error:

$RMS = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$

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
