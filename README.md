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

### 2. Custom Inverse Kinematics (IK) Engine
Rather than relying on pre-built robotics toolkits, I designed and coded a custom math engine from scratch to bridge the gap between abstract 3D spatial coordinates and raw actuator joint angles.

* **Actuator Range Mapping & Stall Prevention:** The first step required profiling the physical boundaries of each linkage. For instance, the base shoulder servo operates across a 180-degree physical sweep, which I explicitly mapped to a safe hardware duty cycle range between **105 and 520**. Establishing these strict software-level constraints ensured the actuators would never push past mechanical end-stops, preventing servo stall conditions, minimizing high current spikes, and protecting the power rail from thermal tripping.

* **The Kinematics Baseline (Geometric Shadowing):**
  To develop an intuition for the physical workspace, I built a baseline Forward Kinematics engine driven by analog potentiometer inputs. I utilized these manual joint positions to calculate how much physical length each arm segment projected down onto the 2D surface of the desk—essentially treating the bicep and forearm linkages as hypotenuses of moving triangles. By continuously resolving these dynamic variables (`bicepLength` and `forearmLength`), the firmware calculated the combined radius of the arm extension. Multiplying this cumulative distance by the sine and cosine of the base shoulder angle provided a real-time, mathematical estimate of exactly where the end-effector was in space.

* **Transitioning to 3D Inverse Kinematics (IK):**
  Once the forward geometry was verified, I inverted the mathematical pipeline to implement true target-driven Inverse Kinematics. The core objective was to feed an absolute Cartesian vector `(X, Y, Z)` from the computer vision pipeline into the ESP32 and force the microcontroller to solve the required actuator positions backwards. 
  
  The engine first parses the targeted position, isolates the base rotation matrix to aim the shoulder, and then breaks down the required vertical depth profile. By evaluating the necessary linear extension across the $Z$-axis, the script determines the exact physical posture the bicep must take. Once the required bicep posture is locked in, the firmware uses the geometric properties established during the forward phase to run a reverse lookup—mapping the spatial requirements directly back into a discrete 12-bit hardware duty cycle. This exact same triangulation process is executed simultaneously across the forearm, bicep, and base shoulder joints to align the entire linkage assembly in a single clock cycle.

* **Dynamic Z-Axis Leveling & End-Effector Stabilization:**
  The wrist/hand assembly required a completely separate tracking system to manage physical picks cleanly. If the hand remained fixed relative to the forearm, moving the bicep or forearm would cause the gripper to tilt wildly, scraping the table surface or approaching the object at an unusable angle. To achieve true parallel Z-axis self-leveling, the firmware actively monitors the real-time changes in both the bicep and forearm duty cycles. It then dynamically calculates a counteracting offset and applies it directly to the wrist hand servo. This continuous, real-time stabilization loop counteracts any intermediate joint elevation shifts, keeping the gripper locked in a stable, perfectly vertical downward-facing orientation throughout the entire 3D travel arc.

---

## 👁️ Computer Vision & Coordinate Mapping

This module bridges the gap between what the overhead camera sees and where the physical arm moves. Transforming raw digital pixel coordinates from a video stream into absolute physical destinations requires balancing deep learning inference with geometric transformation pipelines.

### 1. Object Detection (YOLOv8)
To allow the robot to identify targets autonomously, I trained a custom **YOLOv8** object detection model (`best.pt`) optimized specifically to localize a green desk eraser. 
* **The Dataset & Training:** I generated a custom image dataset capturing the object under varying lighting conditions, baseline shifts, and shadows. The model was trained locally over **50 epochs**.
* **Performance:** The resulting weights are highly optimized for local inference, consistently outputting bounding boxes with a localized confidence threshold **above 83%**. The center point of this bounding box provides the raw `(pixel_x, pixel_y)` anchor for the rest of the mapping stack.

---

### 2. Pixels-to-Millimeters Transformation (Homography)
To translate a raw 2D pixel coordinate into an absolute physical location, I implemented a $3 \times 3$ projective **Homography Matrix** using OpenCV (`cv2.getPerspectiveTransform`). This maps the four clicked pixel corners of the camera view directly to a rigid $600\text{ mm} \times 380\text{ mm}$ physical whiteboard grid boundary on my desk.

* **The Calibration Trap (The Sequence Bug):** During integration, I hit a massive roadblock where the calculated $X$-axis coordinates were dead-on, but the $Y$-axis distances consistently overstretched reality by nearly $2\text{ cm}$. 
  
  The culprit was an order-of-operations bug in my vision script: I was generating the `homography_matrix` using corner coordinates captured from a pre-processed canvas, but the script was calculating the matrix *before* actually performing the 90-degree clockwise frame rotation and $1280 \times 720$ canvas resizing on the live image feed. This mismatch caused OpenCV's backend math to scale coordinates using uninitialized canvas dimensions, distorting the aspect ratio projection map. Moving the image rotation and resizing logic to the absolute top of the pipeline—before any transformation matrices are built—completely resolved the core spatial warp.

---

### 3. Static Camera Angle Calibration
Initially, running tests using a hand-held smartphone camera introduced too many variables; minor shifts in hand posture or lens tilt completely invalidated the perspective map between test runs. Moving to a **rigid, completely static overhead camera mount** was critical to lock down a permanent mathematical model of the workspace.

* **Fixing Lens Projection Tilt:**
  Even with a fixed mount, a camera lens positioned at a slight angle introduces a predictable perspective distortion, stretching the calculated $Y$-values further away from the lens. Because the view is completely static, I was able to isolate and resolve this using linear scaling.
  
  By placing the target at a known physical distance ($15.2\text{ cm}$) and measuring the script's raw homography output ($16.8\text{ cm}$), I calculated a permanent corrective multiplier:
  
  $$\text{Scaling Factor} = \frac{15.2}{16.8} \approx 0.90476$$
  
  Multiplying the raw centimeter outputs by `0.90476` across the entire $Y$-axis compresses the perspective tilt perfectly, creating dead-accurate spatial tracking across all quadrants of the workspace.


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
