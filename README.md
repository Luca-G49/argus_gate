# Argus Gate: ROS 2 Airsoft Sentry Turret

**Argus Gate** is a custom robotic project for an autonomous **Airsoft Sentry Turret** (Pitch & Yaw). The system uses **ROS 2 Humble** (running on Ubuntu 22.04) for high-level logic and vision, while a **Siemens S7-1214C PLC** handles the physical motion.

---

## 🏗️ System Overview

The system is designed as a **decoupled ROS 2 Graph**, where functional levels are separated to ensure safety, modularity, and low-latency hardware communication.

<img src="./docs/system_diagram.svg" width="600">

### 1. GUI Level
*   **`gui_node` (Planned)**: Web dashboard developed with **Flet**.
    *   **Interaction**: **Subscribes** to `PlcStatus` for real-time telemetry and the camera stream. It **publishes** mode requests, manual target overrides, and system commands.

### 2. Logic Level
*   **`manager_node`**: The "brain" of the turret.
    **Interaction**: It acts as a message multiplexer. It **subscribes** to `TeleopCommand` and perception data.
    *   **Logic**: Based on the active state (Manual, Semi-Auto, Full-Auto), it **publishes** the final `PlcCommand` to the Driver Level. It also acts as a **Safety Watchdog**, monitoring node heartbeats to force an IDLE state upon link loss.

### 3. Teleop Level
*   **`joystick_node`**: The user teleop interface.
    *   **Interaction**: It **subscribes** to raw controller inputs from the hardware layer. It maps axes and buttons and **publishes** a `TeleopCommand`.

### 4. Perception Level
*   **`target_processor_node` (Planned)**: The vision and coordinate transformation engine.
    *   **Interaction**: It **subscribes** to 3D spatial results from the `depthai_ros_driver`.
    *   **Logic**: Uses **TF2 (Transform Library)** to convert 3D camera coordinates into Pitch/Yaw angles relative to the turret base, then **publishes** target setpoints to the Manager.

### 5. Drivers Level
This level handles the physical hardware:
*   **`plc_bridge_node`**: Manages the high-speed **100Hz UDP Cyclic link** (MSG200/201) with the Siemens PLC.
*   **`joystick_bridge_node`**: Interfaces with the physical game controller (PS4/Xbox) using the **SFML** library.
*   **`depthai_ros_driver`**: The official driver for the **Luxonis OAK-D**, providing 3D spatial detections.

---

## ⚙️ Hardware Details

*   **Controller:** Siemens S7-1214C PLC.
*   **Motors:** 2x **Nema Stepper Motors** (Pitch & Yaw).
*   **Drivers:** 2x **DM556** digital stepper drivers.
*   **Control Method:** The PLC manages the motors in **Open Loop** using high-speed pulse outputs (PTO).
*   **Vision:** Luxonis OAK-D S2 Depth Camera.

---

## 📐 Mechanical Design & CAD

*   **CAD Model:** Available in the [**`/cad`**](./cad) folder in **.STEP** format.
*   **Range of Motion:** +-120° Yaw rotation / -20° to +45° Pitch.

---

## 🚀 System Capabilities

- [x] **ROS 2 Workspace**: Standardized modular package structure.
- [x] **PLC Bridge**: Reliable 100Hz UDP communication with Siemens PLC.
- [x] **Manual Teleoperation**: Full control with Joystick interface.
- [ ] **Rviz Digital Twin**: Real-time 3D simulation and visualization.
- [ ] **TF2 Integration**: Dynamic coordinate transformation for targeting.
- [ ] **Flet Dashboard**: Modern web GUI for telemetry and remote control.
- [ ] **Autonomous Tracking**: Full-Auto mode using AI vision.

---

