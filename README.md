# 6-DOF Autonomous Object Tracking Robotic Arm

An autonomous eye-to-hand robotic pick-and-place platform combining computer vision object localization with custom embedded kinematics.

---

## 🛠️ Project Overview

* **What it does:** This project is a full-stack, 6-DOF autonomous robotic arm that utilizes computer vision to localize objects in a physical workspace and translate those visual data points into real-world coordinate targets. 

* **System Evolution:** The platform was engineered iteratively across three distinct design phases:
  * **Version 1 (Joint Control):** Built the baseline forward-kinematic engine, mapping analog potentiometer inputs directly to manual multi-servo positioning.
  * **Version 2 (Coordinate Control):** Developed a custom geometric Inverse-Kinematic (IK) engine, allowing the robot to receive any absolute $(X, Y, Z)$ Cartesian coordinate and solve the required joint angles natively on-chip.
  * **Version 3 (AI Integration):** Combined the inverse-kinematic engine with an overhead camera pipeline running a custom-trained YOLOv8 object detection model, enabling the platform to autonomously locate, calculate spatial trajectories for, and pick up target objects.

* **The Core Stack:** C++, Python, OpenCV, YOLOv8, PySerial, ESP32, MG996R Servos, Autodesk Fusion 360 (CAD)

---

## 🦾 Kinematics & Firmware Architecture

This layer manages hardware actuation, real-time signal processing, and multi-joint coordination. By handling calculation pipelines natively on the microcontroller, the system avoids relying on resource-heavy operating middleware (like ROS) and ensures deterministic response times.

### 1. Embedded Control Stack (ESP32)
The low-level electronics architecture is built around an **ESP32 microcontroller** configured to manage real-time multi-servo communication and asynchronous host commands.

* **Multi-Servo PWM Generation:** To drive the physical linkages under continuous load without signal jitter, I utilized the ESP32's hardware-based LEDC peripheral. The stack configures **5 independent PWM channels** running at a base frequency of **50 Hz** with a high-resolution **12-bit depth** (mapping to a discrete duty cycle scale of 0 to 4095). This resolution allows for fractional joint adjustments. Actuators are mapped to dedicated GPIO targets across the bicep, forearm, base shoulder, wrist hand, and gripper assemblies.
* **Non-Blocking Serial Architecture:** The main execution routine processes incoming coordinate updates over the hardware serial line at a baud rate of **115,200**. Instead of using blocking execution methods like standard `delay()` calls which halt the CPU, the firmware handles inputs dynamically within the `loop()` cycle. 
* **Dynamic Servo Smoothing:** When a coordinate packet arrives, the script compares the new duty cycle target against the current positions (`prevDutyBicep != dutyBicep`). If a change is detected, the micro stepped-iterates through the duty array smoothly, applying discrete incremental adjustments. This approach prevents massive instantaneous voltage spikes from dropping the power rail, eliminates abrupt mechanical tearing, and generates smooth, predictable 3D paths.

### 2. Custom Inverse Kinematics (IK)
* Explain the math engine you wrote. Instead of relying on heavy third-party software overhead (like ROS), detail how your native C++ geometry/trigonometry scripts break down 3D space $(X, Y, Z)$ into raw joint angles.
* **Self-Leveling & Boundaries:** Mention how your firmware handles vertical tracking (Z-axis shifts) while keeping the gripper parallel to the workspace.
* 
---

## 👁️ Computer Vision & Coordinate Mapping
*Explain how your robot transforms what it "sees" into where it "moves". Describe the hurdles you faced getting the coordinates to match reality.*

### 1. Object Detection (YOLOv8)
* Describe how you trained or set up your model (epochs, datasets, what object you targeted, and the typical confidence scores you achieved).

### 2. The Pixels-to-Millimeters Transformation (Homography)
* Explain how you mapped a 2D image canvas to a physical desk workspace.
* **The Calibration Trap:** *Write in your own words about the sequence bug we discovered—how calculating the transformation matrix before or after image transformations (like clockwise rotation and resizing to 1280x720) warps the Y-axis projection scaling.*

### 3. Static Camera Angle Calibration
* Explain why switching from a hand-held viewpoint to a completely rigid, static overhead view was necessary for math stability.
* Detail how you calculated your permanent linear scaling multiplier (multiplying by `0.90476`) to compress the camera's perspective tilt along the Y-axis.



---

## 📐 Mechanical & Structural Design
*Give a quick nod to your CAD and manufacturing choices.*

* **Linkage Dimensions:** Bicep = `13.5 cm` | Forearm = `16.5 cm`
* **Stability:** Describe your physical build setup, including your custom 3D-printed brackets and T-slot slider tracks used to manage the base rotation.
* **Design Files:** Mention that the completeparametric assembly is preserved in the repository as a universal `.STEP` model for easy cross-platform evaluation.

---

## 📊 Experimental Validation (The Test Clips)
*Prove that your machine actually works across the entire board. Break down the logic behind why you picked your demo shots.*

### Version 1.0: Hardware & Motion Baselines
* **Symmetrical Parity:** Mirroring movements across the center line (e.g., `15, 15` to `-15, 15`) to check trigonometric balancing.
* **The Geometric Workspace Sweep:** Tracing fluid paths through multiple quadrants to verify multi-joint synchronization.

### Version 2.0: Full Vision Integration (The Multi-Cam Layout)
*Explain your 4-panel split-screen layout (Raw View, YOLOv8 Overlay, and Dual Hardware Profile Angles).*

* **Clip 1 (Wide Left Sweep):** `(-21.7, 8.7, 10.0)` — Validating wide base rotations and sign-polarity swaps.
* **Clip 2 (Maximum Extension):** `(0.0, 23.5, 10.0)` — Verifying forward depth scaling at the extreme boundary.
* **Clip 3 (Close-In Crowding):** `(14.0, 12.0, 10.0)` — Checking linkage folding near the base without joint singularities.
* **Clip 4 (Fixed Quadrant Calibration):** `(18.7, 11.1, 10.0)` — Proving the accuracy of the fixed-angle camera scaling modifier.

---

## 📁 Repository Structure
```text
├── cad/         # Parametric .STEP models and 3D print mesh files (.3mf / .stl)
├── firmware/    # C++ ESP32 firmware (Serial parsing, servo PWM, geometric IK)
├── vision/      # Python scripts (OpenCV processing, Homography matrix, YOLOv8 inference)
└── docs/        # Mathematical derivations and calibration notes
