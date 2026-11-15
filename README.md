[![Rover core tests](https://github.com/sksharma903231/robotics_phase2/actions/workflows/core-tests.yml/badge.svg)](https://github.com/sksharma903231/robotics_phase2/actions/workflows/core-tests.yml)

# Enterprise Rover (Phase 2): Autonomous Drivetrain Control, Perception & Telemetry

A modular robotics simulation project demonstrating C++, Python, ROS 2, URDF modeling, Gazebo integration, and real-time telemetry teleoperations.

---

## 🏗️ Architecture Overview

The system bridges low-level actuator control, 2D LiDAR range perception, Gazebo physics simulation, and a web-based operator control room:

```mermaid
flowchart TD
    subgraph CoreCpp ["C++ Core Perception & Control (src/ & include/)"]
        A[MotorPIDController] -->|Velocity Commands| B[4WD Differential Drivetrain]
        C[LidarProcessor] -->|Scan Filtering| D{Obstacle Hazard Logic}
        D -->|Distance < 3.0m| E[Emergency Stop / Brakes Applied]
        D -->|Distance 3.0m - 6.0m| F[Caution Speed Reduction]
        D -->|Distance > 6.0m| G[Path Clear]
    end

    subgraph Sim ["Gazebo & URDF Simulation (ros2_ws/)"]
        H[rover.urdf Model] -->|Physics & Mass| I[Gazebo Sim / Harmonic]
        I -->|gz::sim DiffDrive| J[/cmd_vel & /odom]
        I -->|gz::sim JointStatePublisher| K[/joint_states]
        I -->|gz::sim Sensors GPU LiDAR| L[/scan]
    end

    subgraph Bridge ["ROS 2 Telemetry Bridge (ros2_ws/)"]
        K -->|Joint Angular Velocity| M[telemetry_bridge.py]
        M -->|Atomic Write rad/s -> RPM| N[("/tmp/rover_telemetry.json")]
    end

    subgraph TelemetryDashboard ["Operator Telemetry Control Room (dashboard/)"]
        N -->|Polling Read & Staleness Check| O[Flask Web Server (app.py)]
        O -->|JSON REST API /api/telemetry| P[HTML5 / CSS / JS Real-Time UI]
    end
```

---

## 📁 Repository Structure

```text
.
├── CMakeLists.txt                  # Root CMake build configuration (C++17, CTest)
├── README.md                       # Architecture, reproduction guide, and audit report
├── requirements.txt                # Python dependencies (Flask, Pytest)
├── run_tests.sh                    # Automated unified test runner (CTest + Pytest)
├── pytest.ini                      # Pytest runner configuration
│
├── include/                        # Modular C++ headers
│   ├── lidar_processor.hpp         # Noise filtering, closest obstacle detection & hazard status
│   └── motor_controller.hpp        # Discrete PID controller with anti-windup & safety bounds
│
├── src/                            # Standalone C++ node implementations
│   ├── lidar_node.cpp              # LiDAR range processing execution demo
│   └── motor_controller.cpp        # 4WD rover turn maneuver PID execution demo
│
├── tests/                          # Independent unit tests (No Gazebo required)
│   ├── test_rover_logic.cpp        # C++ unit tests: PID convergence, bounds, LiDAR filters
│   └── test_dashboard.py           # Python unit tests: rad/s to RPM, Flask API endpoints
│
├── dashboard/                      # Web-based Telemetry Control Room
│   ├── app.py                      # Flask REST API server with staleness & mock support
│   └── templates/
│       └── index.html              # Responsive dark-theme real-time teleoperations UI
│
└── ros2_ws/                        # ROS 2 Colcon Workspace
    └── src/
        └── rover_sim_pkg/          # Rover ROS 2 simulation package
            ├── CMakeLists.txt      # ament_cmake build rules
            ├── package.xml         # Package metadata and ROS 2 dependencies
            ├── launch/
            │   └── sim_launch.py   # Gazebo sim, URDF spawner & bridge launch script
            ├── scripts/
            │   └── telemetry_bridge.py # JointState subscriber & atomic telemetry cache
            └── urdf/
                └── rover.urdf      # Rover URDF with dual casters, LiDAR & Gazebo plugins
```

---

## ⚡ Engineering Highlights

### 1. C++ Motor PID Controller (`include/motor_controller.hpp`)
- Implements discrete-time PID velocity loop:
  $$u(t) = K_p e(t) + K_i \int e(t)\,dt + K_d \frac{de(t)}{dt}$$
- **Anti-Windup Protection**: Integrator sum is clamped to configurable bounds ($\pm 50.0$), preventing integral runaway during sustained error.
- **Actuator Rate Limiting**: Clamps maximum acceleration/deceleration step ($\pm 30.0$ RPM/step).
- **Runaway Loop Guards**: Guarded iteration limits prevent infinite execution if tuning parameters induce oscillation or divergence.

### 2. C++ LiDAR Range Perception (`include/lidar_processor.hpp`)
- Filters invalid sensor data: discards `NaN`, `Inf`, and out-of-sensor-bound noise readings ($r < 0.1\,\text{m}$ or $r > 50.0\,\text{m}$).
- Classifies obstacle hazards into deterministic states:
  - `EMERGENCY_STOP`: Obstacle distance $< 3.0\,\text{m}$ (critical brake override).
  - `CAUTION`: Obstacle distance between $3.0\,\text{m}$ and $6.0\,\text{m}$.
  - `CLEAR`: Safe clearance $> 6.0\,\text{m}$.
  - `NO_DATA`: Handles empty or completely corrupted sensor arrays safely.

### 3. URDF Rover Model & Gazebo Harmonic Simulation
- **Chassis Stability**: Differential drive robot with dual front and rear low-friction caster wheels ($\mu = 0.001$) preventing pitch oscillation and tipping.
- **Sensor Linkage**: Integrated `lidar_link` rigidly attached to the chassis with realistic visual, collision, and inertial properties.
- **Gazebo Plugins**:
  - `gz::sim::systems::DiffDrive`: Handles `/cmd_vel` wheel kinematics and publishes `/odom`.
  - `gz::sim::systems::JointStatePublisher`: Emits wheel angular velocities to `/joint_states`.
  - `gz::sim::systems::Sensors`: Generates simulated 360° GPU LiDAR point streams.

### 4. Telemetry Teleoperations Dashboard
- **Robust Exception Handling**: Replaced buggy legacy file reads with atomic file updates (`.tmp` + `os.replace`), eliminating race conditions between the ROS 2 bridge and web server.
- **Data Freshness / Staleness**: Dynamically flags status as `ONLINE`, `STALE` (if stream stops $> 5\,\text{s}$), or `OFFLINE`.
- **Standalone Mock Demonstration**: Supports running without ROS 2 or Gazebo installed via `MOCK_MODE=1` or manual REST API telemetry injection (`POST /api/telemetry`).

---

## 🚀 Reproduction & Execution Guide

### Prerequisites
- **Compiler**: GCC / G++ (C++17 supported)
- **Build System**: CMake $\ge 3.16$
- **Python**: Python $\ge 3.10$

### 1. Build and Run Standalone C++ Nodes
```bash
# Configure and build
cmake -B build -S .
cmake --build build

# Run LiDAR obstacle detection demo
./build/lidar_node

# Run 4WD differential drivetrain PID turn maneuver demo
./build/motor_controller
```

### 2. Run Automated Test Suite (CTest + Pytest)
A single command verifies all C++ controller and perception logic, as well as Python telemetry API endpoints:
```bash
./run_tests.sh
```

Or run individually:
```bash
# Run C++ unit tests
ctest --test-dir build --output-on-failure

# Run Python unit tests
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pytest -v tests/test_dashboard.py
```

### 3. Launch the Telemetry Dashboard
```bash
# Activate virtual environment
source .venv/bin/activate

# Option A: Run with simulated live telemetry
MOCK_MODE=1 python3 dashboard/app.py

# Option B: Run in live listener mode (awaits telemetry bridge)
python3 dashboard/app.py
```
Open [http://localhost:5000](http://localhost:5000) in your browser to view the real-time rover telemetry control room.

### 4. Launch Full ROS 2 / Gazebo Simulation (When ROS 2 is installed)
In an environment with ROS 2 (Humble or Jazzy) and Gazebo Harmonic:
```bash
# Source ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Build the ROS 2 workspace
cd ros2_ws
colcon build --packages-select rover_sim_pkg
source install/setup.bash

# Launch URDF, Gazebo simulation, ROS bridge, and telemetry bridge
ros2 launch rover_sim_pkg sim_launch.py
```

---

## 🔍 Verification & Test Results

| Test Target | Test Case | Status | Verified Behavior |
| :--- | :--- | :---: | :--- |
| **C++ PID** | `test_pid_convergence` | **PASSED** | Converges smoothly to target speed within 0.1 RPM tolerance |
| **C++ PID** | `test_pid_deceleration` | **PASSED** | Handles negative error and deceleration without overshoot runaway |
| **C++ PID** | `test_pid_anti_windup` | **PASSED** | Integrator sum is clamped to configured bound during sustained error |
| **C++ PID** | `test_pid_safety_limit` | **PASSED** | Unstable gains abort at max iterations without hanging |
| **C++ LiDAR** | `test_lidar_noise_filtering`| **PASSED** | Ignores negative values, sub-minimum noise (< 0.1m) and out-of-range |
| **C++ LiDAR** | `test_lidar_emergency_stop` | **PASSED** | Distance < 3.0m triggers `EMERGENCY_STOP` |
| **C++ LiDAR** | `test_lidar_caution_zone` | **PASSED** | Distance between 3.0m - 6.0m triggers `CAUTION` |
| **C++ LiDAR** | `test_lidar_no_data` | **PASSED** | Empty or invalid scans return `NO_DATA` safely |
| **Python** | `test_rad_to_rpm_conversion`| **PASSED** | Validates angular velocity conversion factor ($60 / 2\pi$) |
| **Python** | `test_extract_wheel_rpms` | **PASSED** | Accurately extracts wheel joint speeds by name and fallback index |
| **Python** | `test_dashboard_home` | **PASSED** | Web interface renders cleanly with status badges |
| **Python** | `test_dashboard_telemetry` | **PASSED** | Validates JSON schemas for online, offline, and stale telemetry |

---

## ⚠️ Honest Limitations & System Notes

1. **Simulation Scope**: This project is entirely software simulation and teleoperations logic; it does not claim to run directly on physical embedded microcontrollers (STM32/ESP32).
2. **IPC Mechanism**: The telemetry bridge communicates with Flask via an atomic JSON state file (`/tmp/rover_telemetry.json`). While lightweight and decoupled for this project's scope, an enterprise production robot typically adopts ROS 2 Web Bridge (`rosbridge_suite`), Zenoh, or direct WebSocket telemetry streaming.
3. **Host Environment Note**: On systems running pre-release or rolling Linux distributions with Python 3.14 default interpreters, system binary packages built for Python 3.12 (`/opt/ros/jazzy`) encounter C-ABI extension mismatches (`_rclpy_pybind11`). To ensure full reproducibility across any machine, all core C++ control/perception algorithms, dashboard teleoperations, and unit tests have been engineered to run independently of system ROS Python bindings.
