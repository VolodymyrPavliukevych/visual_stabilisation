
# Ardupilot Optical Flow Drone Stabilization

This project implements an **optical flow-based stabilization system** for Ardupilot-compatible drones. The system enhances flight stability by using optical flow data to adjust drone position and orientation in real-time. The architecture follows a modular class-based design, enabling easy maintenance, expansion, and testing.

![Simulation screen cast]("dron_stabilisation.gif" Dron Stabilisation screen cast)

---

## **Features**
- Optical flow-based stabilization for precise position control  
- PID-based control loops for smooth adjustments  
- Modular design with clear class separation  
- Real-time feedback using sensor fusion  

---

## **Architecture**
The project consists of three core modules:

### 1. `FlyController` (`fly_controller.py`)
The `FlyController` class manages the low-level control of the drone using PID controllers. It computes the required motor adjustments based on optical flow data and other sensor inputs.

**Key components:**
- `__init__()` – Initializes the PID controllers and control parameters.
- `update_control()` – Processes sensor data and adjusts control outputs.
- `compute_pid()` – Implements PID logic for each control axis (roll, pitch, yaw).

---

### 2. `ReactionControlSystems` (`reaction_control_systems.py`)
This module implements the reaction control system (RCS) logic, managing how the drone responds to disturbances and stabilizes itself.

**Key components:**
- `ReactionControlSystems` class:
  - `apply_corrections()` – Applies corrections based on control errors.
  - `set_target()` – Sets target orientation and position.
  - `update_state()` – Updates the internal state based on sensor feedback.

---

### 3. `AutoPilot` (`auto_pilot.py`)
The `AutoPilot` class integrates the `FlyController` and `ReactionControlSystems` to form a high-level flight control system. It acts as the main decision-making unit for autonomous flight.

**Key components:**
- `__init__()` – Initializes submodules and state variables.
- `run()` – Main control loop for real-time flight control.
- `adjust_course()` – Adjusts flight path based on optical flow and external disturbances.
- `safety_check()` – Ensures that all flight parameters are within safe limits.

---

## **File structure**
```
├── fly_controller.py
├── reaction_control_systems.py
├── auto_pilot.py
├── README.md
└── requirements.txt
```

---

## **Installation**
1. Clone the repository:
```bash
git clone https://github.com/yourusername/ardupilot-optical-flow.git
```
2. Install dependencies:
```bash
pip install -r requirements.txt
```
3. Run the autopilot:
```bash
python auto_pilot.py
```

---

## **How it works**
1. The drone receives real-time optical flow data from onboard sensors.
2. The `FlyController` computes necessary adjustments using PID logic.
3. The `ReactionControlSystems` module applies corrections to stabilize the drone.
4. The `AutoPilot` module manages high-level flight objectives and decision-making.

---

## **Future improvements**
- Add support for different drone models.
- Improve performance with adaptive PID tuning.
- Integrate with GPS for long-distance navigation.  

---

# Ardupilot simulator | Setup
### Gudes and usefull links

[Short course](https://www.youtube.com/watch?v=m7hPyJJmWmU)
[Official documentation](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)

## 1. Install prerequisites

### Install gz
```bash
brew tap osrf/simulation
brew install gz-harmonic
```

## 2. Install ArduPilot

### Clone and setup
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

### Build ArduPilot
```bash
./waf configure --board sitl
./waf copter
```

### Run Ardupilot and QGroundControl
```
cd Tools/autotest
python3.12 sim_vehicle.py -v ArduCopter 
```

### Build Ardopilot plugin
```
cd ~/Projects/sources/
mkdir -p gz_ws/src && cd gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo

cd ardupilot_gazebo

export GZ_VERSION=harmonic
If you need to have qt@5 first in your PATH, run:
  echo 'export PATH="/opt/homebrew/opt/qt@5/bin:$PATH"' >> ~/.zshrc

For compilers to find qt@5 you may need to set:
  export LDFLAGS="-L/opt/homebrew/opt/qt@5/lib"
  export CPPFLAGS="-I/opt/homebrew/opt/qt@5/include"
  
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
make install 
```

### Add plugin data to environmant 
In the same terminal with `Gazebo Server` 
```
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/ardupilot_gazebo
export GZ_SIM_RESOURCE_PATH=/usr/local/share/ardupilot_gazebo/worlds:/usr/local/share/ardupilot_gazebo/models
```

### Iris quadcopter

### Run Gazebo Server 

```
gz sim -v4 -r iris_runway.sdf -s
```

### Run Gazebo GUI

```
gz sim -v4 -r iris_runway.sdf -g
```

### Run SITL

```
cd /Users/roaming/Projects/sources/ardupilot/Tools/autotest
python3.12 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
Arm and takeoff
```

### Some control
```
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5

```