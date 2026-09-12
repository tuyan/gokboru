# Gökbörü

**Gökbörü** is a C++ autonomous drone targeting and delivery engine for ArduPilot ecosystems. It seamlessly combines GPS macro-navigation with CUDA-accelerated YOLOv8 vision tracking to execute secure search-and-drop missions, featuring real-time OS scheduling and signed MAVLink v2 protocols for zero-latency, tamper-proof tactical flight control.

## System Architecture

Gökbörü is designed to run on companion computers (like the NVIDIA Jetson series) mounted on multirotor drones. It bridges the gap between static waypoint navigation and dynamic, AI-driven flight by taking direct control of the drone's velocity vectors.

*   **CUDA-Accelerated Edge Vision:** Bypasses CPU bottlenecks by routing OpenCV DNN YOLOv8 inference directly to NVIDIA GPU cores using FP16 (16-bit float) quantization.
*   **Asynchronous Telemetry:** Decouples the MAVLink UDP packet receiver from the computer vision loop using thread-safe shared states (`std::mutex` and `std::atomic`), preventing network latency from dropping camera frames.
*   **Real-Time POSIX Scheduling:** Leverages Linux `SCHED_FIFO` to elevate the tracking threads above standard OS background tasks, ensuring the flight control loop never stutters.
*   **Cryptographically Signed Links:** Implements MAVLink v2 SHA-256 packet signing with monotonic 10-microsecond timestamps to completely immunize the drone against network spoofing and command replay attacks.
*   **Tri-Axial PID Flight Dynamics:** Translates 2D bounding boxes into fluid 3D flight using custom Proportional-Integral-Derivative controllers for Yaw (centering), Altitude (Z-axis alignment), and Distance (scale-based forward velocity).

## The Autonomous State Machine

The mission logic operates in a strict, self-contained loop:
1.  **Transit:** Dispatches the drone to a global GPS coordinate (`SET_POSITION_TARGET_GLOBAL_INT`) and calculates Haversine distance until arrival.
2.  **YOLO Search:** Once within a 3-meter radius of the target zone, the drone holds GPS position while executing a slow 360-degree yaw rotation to scan the environment.
3.  **Tracking & Lock:** Upon detecting the assigned YOLO class, the system arrests the spin and feeds bounding box coordinates into the PID controllers to close the distance.
4.  **Payload Deployment:** When the bounding box reaches the target pixel scale and is perfectly centered, Gökbörü fires a PWM servo command to release the payload.

---

## Prerequisites

*   **Hardware:** NVIDIA Jetson (Nano, Xavier, Orin) or an x86 companion computer with a dedicated GPU.
*   **Flight Controller:** Pixhawk or equivalent running ArduPilot (Copter).
*   **Software:** 
    *   Linux OS (Ubuntu recommended).
    *   OpenCV compiled from source with `WITH_CUDA=ON`, `WITH_CUDNN=ON`, and `OPENCV_DNN_CUDA=ON`.
    *   Official MAVLink v2 C library.

## Installation & Build

1. Clone the repository and fetch the MAVLink headers:
   ```bash
   git clone [https://github.com/yourusername/gokboru.git](https://github.com/yourusername/gokboru.git)
   cd gokboru
   git clone [https://github.com/mavlink/c_library_v2.git](https://github.com/mavlink/c_library_v2.git) mavlink


## Pre-flight Checklist

1. **Power & Peripherals:** Ensure the companion computer is receiving clean power and has uninterrupted camera access.
2. **Telemetry Link:** Verify ArduPilot is routing MAVLink telemetry to UDP port `14550` on the companion computer.
3. **GPS Lock:** Ensure the flight controller has a solid 3D GPS lock with a low HDOP before proceeding.
4. **Airborne Initialization:** Arm the drone and take off manually (or via GCS) to a safe altitude.
5. **Launch Engine:** Execute the `./ai_drone_tracker` binary. The drone will immediately take over velocity control and begin the transit phase to the designated coordinates.

---

## Safe PID Tuning Guide

Every drone frame is different in weight, motor size, and aerodynamics. You **must** tune the Proportional (`Kp`), Integral (`Ki`), and Derivative (`Kd`) values in `main.cpp` for your specific hardware to prevent extreme oscillations or crashes. 

> **CRITICAL SAFETY WARNING:** Never tune AI flight loops without holding the physical RC transmitter. Map a dedicated hardware switch to immediately change the flight mode to `LOITER` or `STABILIZE` to instantly sever the AI control if the drone accelerates unexpectedly.

1. **Zero Out Constants:** Set `Ki = 0.0` and `Kd = 0.0`. Set `Kp` to a highly conservative baseline (e.g., `0.001`). 
2. **Tune Proportional (Kp):** Incrementally increase `Kp` by small margins. Have a spotter move a physical target back and forth in front of the camera while the drone hovers. Keep increasing `Kp` until the drone starts overshooting the target and oscillating (wobbling).
3. **Tune Derivative (Kd):** Leave `Kp` at the oscillating value. Slowly increase `Kd` (start around `0.005`). The derivative term predicts the error rate and applies the brakes, stopping the wobble and locking the drone onto the target smoothly.
4. **Tune Integral (Ki):** Only add `Ki` if the drone consistently stops slightly short of the exact center pixel due to physical drag, battery sag, or wind resistance. Start extremely low (e.g., `0.0001`), as too much integral gain will cause slow, wide, and dangerous looping orbits.

## Usage

Gökbörü requires three arguments to initiate a mission: Target Latitude, Target Longitude, and the file path to the reference image it needs to search for. 

> **Note:** The reference image should be a clear, well-lit, and tightly cropped picture of the specific object you want the drone to track.

```bash
# Syntax
./ai_drone_tracker <Target_Lat> <Target_Lon> <Reference_Image_Path>

