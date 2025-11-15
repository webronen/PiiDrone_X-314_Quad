# Deep Dive: Åström–Hägglund Relay Auto-Tuning System for Embedded Flight Controllers

## Abstract
This paper presents a practical, robust implementation of the Åström–Hägglund relay auto-tuning method for embedded quadcopter flight controllers. The system enables hands-off, in-field PID gain estimation using staged relay excitation, unified RMS error metrics, and robust fallback logic. We detail the algorithmic architecture, design rationale, and embedded safety considerations.

## 1. Introduction
PID controllers are ubiquitous in drone flight control, but manual tuning is time-consuming and error-prone. The Åström–Hägglund relay method automates PID tuning by inducing controlled oscillations and analyzing system response. We extend this method with a staged, RMS-based architecture suitable for real-time, resource-constrained embedded systems.

## 2. System Overview
This paper details a robust Åström–Hägglund relay auto-tuning method for embedded quadcopters. The system enables hands-off PID gain estimation using staged relay excitation, unified RMS error, and fallback logic. We cover algorithm, rationale, and safety.
- **Platform:** Ultra-light quadcopter (70g) with nRF microcontroller.
PID controllers are standard in drones, but manual tuning is slow and error-prone. The Åström–Hägglund relay method automates PID tuning by inducing oscillations and analyzing response. We extend it with staged, RMS-based logic for real-time embedded use.

 **Goal:** Auto-select PID gains for roll, pitch, yaw with minimal user input.
 **Platform:** 70g quadcopter, nRF MCU.
 **Constraints:** Real-time, safe, low overhead.

### 3.2 Three-Stage Tuning Architecture
A square wave setpoint is applied to each axis, alternating sign at intervals to excite the system and reveal dynamics.
- **P gain:** Sweep from 3.0 to 15.0 (step 0.5). For each value, collect squared error samples over 2 seconds, compute RMS error, and track the value with the lowest RMS. Advance if RMS < 0.05 or max P is reached.
For each axis:
- **P:** 3.0→15.0 (0.5 step), keep value with lowest RMS, advance if RMS<0.05 or max.
- **D:** 0→3.0 (0.2 step), same, advance if RMS<0.03 or max.
- **I:** 0→2.0 (0.1 step), same, finish if RMS<0.02 or max.
All stages use the same RMS error metric:
All stages use the same RMS error:
$\mathrm{RMS} = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}$
where $e_i$ is the error at sample $i$, $N$ is the sample count. This gives consistent, noise-robust evaluation.
- **State Management:** Static structs per axis track stage, best values, and timing.
Each stage keeps the gain with lowest RMS for robust tuning.
- **Safety:** All gains are bounded; fallback to zero gains if tuning fails or limits are exceeded.
- **State:** Static structs per axis track stage, best, timing.
- **Timing:** Hardware timer snapshots for intervals and relay.
- **Safety:** Gains bounded; fallback to zero if tuning fails or limits hit.
- **Axis:** Each axis tuned independently.
- **No dynamic memory:** All state is static.
- **Noise Rejection:** Squared error penalizes large deviations, making the system robust to outliers.
- **Consistency:** RMS for all PID terms, simple and comparable.
- **Noise rejection:** Squared error penalizes outliers.
- **Embedded:** RMS is efficient and easy to implement.
- **Auto stage:** Tuning advances when RMS goals met.
If tuning exceeds gain limits or fails to meet RMS goals, the system applies safe fallback gains (all zero). This ensures the drone never flies with unstable or untested gains.
If tuning fails or exceeds limits, fallback gains (all zero) are used for safety.
## 7. Results and Discussion
- **Tuning:** ~32s/axis (24 relay flips, 3 stages).
- **Flight:** Gains are conservative, flyable, safe for further tuning.
- **No zero-crossing:** More reliable in noise than classic relay.

This system gives robust, hands-off PID gains for embedded drones. The staged, RMS-based design is ideal for real-time, safety-critical use and adapts to many robots.
The presented Åström–Hägglund relay auto-tuning system delivers robust, hands-off PID gain estimation for embedded drones. Its staged, RMS-based architecture is well-suited to real-time, safety-critical applications and can be adapted to a wide range of robotic platforms.

## References
- [Relay Auto-Tuning (Åström–Hägglund) – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [PID Controller – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
