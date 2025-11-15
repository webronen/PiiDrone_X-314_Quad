# Deep Dive: Åström–Hägglund Relay Auto-Tuning System for Embedded Flight Controllers

## Abstract
This paper presents a practical, robust implementation of the Åström–Hägglund relay auto-tuning method for embedded quadcopter flight controllers. The system enables hands-off, in-field PID gain estimation using staged relay excitation, unified RMS error metrics, and robust fallback logic. We detail the algorithmic architecture, design rationale, and embedded safety considerations.

## 1. Introduction
PID controllers are ubiquitous in drone flight control, but manual tuning is time-consuming and error-prone. The Åström–Hägglund relay method automates PID tuning by inducing controlled oscillations and analyzing system response. We extend this method with a staged, RMS-based architecture suitable for real-time, resource-constrained embedded systems.

## 2. System Overview
- **Objective:** Automate PID gain selection for roll, pitch, and yaw axes with minimal user intervention.
- **Platform:** Ultra-light quadcopter (70g) with nRF microcontroller.
- **Constraints:** Real-time operation, safety, and minimal computational overhead.

## 3. Relay Excitation and Staged Tuning
### 3.1 Relay Excitation
A square wave setpoint (relay) is applied to each axis, alternating sign at fixed intervals. This excites the system and reveals its dynamic response.

### 3.2 Three-Stage Tuning Architecture
Tuning proceeds in three sequential stages for each axis:
- **P gain:** Sweep from 3.0 to 15.0 (step 0.5). For each value, collect squared error samples over 2 seconds, compute RMS error, and track the value with the lowest RMS. Advance if RMS < 0.05 or max P is reached.
- **D gain:** Sweep from 0 to 3.0 (step 0.2). Same RMS process, advance if RMS < 0.03 or max D is reached.
- **I gain:** Sweep from 0 to 2.0 (step 0.1). Same RMS process, finish if RMS < 0.02 or max I is reached.

### 3.3 Unified RMS Metric
All stages use the same RMS error metric:
$$
\text{RMS} = \sqrt{\frac{1}{N} \sum_{i=1}^N (e_i)^2}
$$
where $e_i$ is the instantaneous control error and $N$ is the number of samples in the interval.

### 3.4 Best-Value Tracking
At each stage, the gain value yielding the lowest RMS is retained. This ensures the most effective, noise-robust tuning for each PID term.

## 4. Embedded Implementation
- **State Management:** Static structs per axis track stage, best values, and timing.
- **Timing:** Hardware timer snapshots ensure precise measurement intervals and relay switching.
- **Safety:** All gains are bounded; fallback to zero gains if tuning fails or limits are exceeded.
- **Axis Independence:** Each axis is tuned independently, supporting sequential or parallel operation.
- **No Dynamic Memory:** All state is statically allocated for real-time safety.

## 5. Advantages of RMS-Based Unified Architecture
- **Consistency:** RMS error is used for all PID terms, simplifying logic and improving comparability.
- **Noise Rejection:** Squared error penalizes large deviations, making the system robust to outliers.
- **Embedded Suitability:** RMS is computationally efficient and easy to implement on microcontrollers.
- **Automatic Stage Transition:** Tuning advances automatically when RMS goals are met, reducing user intervention.

## 6. Fallback and Robustness
If tuning exceeds gain limits or fails to meet RMS goals, the system applies safe fallback gains (all zero). This ensures the drone never flies with unstable or untested gains.

## 7. Results and Discussion
- **Tuning Time:** ~32 seconds per axis (24 relay flips across 3 stages).
- **Flight Performance:** Gains are conservative but flyable, providing a safe baseline for further manual refinement.
- **No Zero-Crossing Dependency:** Unlike classic relay methods, this approach does not require zero-crossing detection, improving reliability in noisy environments.

## 8. Conclusion
The presented Åström–Hägglund relay auto-tuning system delivers robust, hands-off PID gain estimation for embedded drones. Its staged, RMS-based architecture is well-suited to real-time, safety-critical applications and can be adapted to a wide range of robotic platforms.

## References
- Åström, K.J., & Hägglund, T. (1984). Automatic tuning of simple regulators with specifications on phase and amplitude margins. Automatica, 20(5), 645-651.
- [Relay Auto-Tuning (Åström–Hägglund) – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Relay_(%C3%85str%C3%B6m%E2%80%93H%C3%A4gglund)_method)
- [PID Controller – Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
