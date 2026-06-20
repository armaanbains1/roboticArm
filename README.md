# roboticArm

# 6-DOF Autonomous Object Tracking Robotic Arm

An autonomous eye-to-hand robotic pick-and-place platform combining computer vision object localization with custom embedded kinematics.

## 🛠️ Project Overview
*Write a 2-3 sentence high-level summary of what your machine does. Imagine explaining it to someone who has never seen it before.*
* **What it does:** * **The Core Stack:** (e.g., Python, OpenCV, YOLOv8, Serial, ESP32, C++, MG996R Servos)

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

## 🦾 Kinematics & Firmware Architecture
*This is your systems engineering section. Detail how the micro-controller processes data without breaking a sweat.*

### 1. Embedded Control Stack (ESP32)
* Describe your low-level setup. Mention managing multi-servo PWM signals and how you handle incoming data over the Serial line (COM3 at 115200 baud) asynchronously without blocking the execution loops.

### 2. Custom Inverse Kinematics (IK)
* Explain the math engine you wrote. Instead of relying on heavy third-party software overhead (like ROS), detail how your native C++ geometry/trigonometry scripts break down 3D space $(X, Y, Z)$ into raw joint angles.
* **Self-Leveling & Boundaries:** Mention how your firmware handles vertical tracking (Z-axis shifts) while keeping the gripper parallel to the workspace.

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
