# PID Autotune via Relay Feedback on Drone Bench

## Abstract  
This paper explains why hover thrust is not required when performing a Relay Feedback Test to determine the ultimate gain (`K_u`) and ultimate period (`T_u`) for Ziegler-Nichols PID tuning on a balanced drone test bench. It outlines the principles of the relay method, the mechanical isolation of rotational dynamics, and practical considerations for effective testing.

## 1. Introduction  
PID tuning is a critical step in achieving stable and responsive control in drone systems. The Ziegler-Nichols (ZN) method, particularly its closed-loop variant using Relay Feedback Testing, is a widely used approach to determine the optimal PID parameters. This paper discusses how the test can be conducted without requiring the drone motor to operate at hover thrust.

## 2. Relay Feedback Test Focuses on Oscillations  
The Relay Feedback Test replaces the proportional controller with a relay (or sign function) that toggles the control output between high and low values when the process variable crosses the setpoint. This setup induces sustained, bounded oscillations in the system.

- **Ultimate Period (`T_u`)**: The period of the sustained oscillation.  
- **Ultimate Gain (`K_u`)**: Calculated from the oscillation amplitude and the relay output magnitude.

These parameters are then used to derive the PID gains using Ziegler-Nichols formulas.

## 3. The Balanced Test Bench Isolates Rotational Dynamics  
A balanced test bench mechanically restricts the drone’s movement to rotation around a single axis (e.g., pitch or roll), preventing vertical translation or lift.

- This isolation decouples rotational dynamics from thrust dynamics.  
- The PID controller for rotational axes can be tuned independently of hover conditions.  
- The motor thrust only needs to be sufficient to generate rotational forces—not to achieve lift.

## 4. Practical Considerations  
Although hover thrust is unnecessary, the motor must produce enough force to:

- Overcome friction in the test bench bearings.  
- Generate measurable oscillations in response to relay switching.

Typically, a low-to-moderate fixed throttle is used—adequate to excite the system but well below the hover point.

## 5. Conclusion  
Relay Feedback Testing on a balanced test bench allows safe and effective PID tuning without requiring hover thrust. This approach simplifies the tuning process and reduces risk, making it ideal for laboratory and development environments.