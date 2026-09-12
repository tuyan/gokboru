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

