# Autonomous Clamp-Lift Robot

An autonomous mobile mechatronics platform featuring real-time target acquisition, dynamic payload clamping, and Ackermann steering. Built on non-blocking C++ firmware with integrated power management and digital signal filtering.

---

## Technical Highlights

* **Non-Blocking C++ Firmware:** Asynchronous control using `micros()` / `millis()` delta-timing loops for concurrent multi-motor execution and IR signal decoding.
* **Signal Conditioning:** 3-sample median filter algorithm on 4x HC-SR04 ultrasonic sensors to discard noise spikes and false sonar reflections.
* **Custom Kinematics & CAD:** Designed in Autodesk Inventor and 3D printed. Features single drag-link Ackermann steering with algorithmic backlash compensation (+25°/-15° overshoot profile).
* **Multi-Actuator Power Architecture:** Drives a NEMA 17 stepper (DRV8825 current driver), 28BYJ-48 clamping stepper, and MG90S high-torque steering servo without logic rail brownouts.

---

## Hardware & Pinout

| Component | Function | Pins |
| :--- | :--- | :--- |
| **ATmega328P** | Core MCU | — |
| **NEMA 17 + DRV8825** | Drive Motor | Step: 8, Dir: 9, Enable: 10 |
| **28BYJ-48 Stepper** | Clamp Actuator | 2, 3, 4, 5 |
| **MG90S Servo** | Ackermann Steering | 7 |
| **HC-SR04 Array** | Distance Telemetry | Trig: 11 | Echo: A2 (Center), A3 (Left), A4 (Top), A5 (Right) |
| **IR Receiver** | Remote Control | 6 |

---

## System Capabilities

1. **Autonomous Search & Align (`executeForkliftAssist`):** Evaluates distance thresholds and triggers 45° reverse-arc sweep maneuvers to lock side-detected targets onto the center clamp line.
2. **RC Auto-Avoidance Mode:** Round-robin sonar scanning triggering auto-braking (<250mm), side-nudging (<350mm), and emergency reversing (<50mm).

---

## Demo
* 🎬 **Video Demonstration:** [Paste your YouTube link here]
