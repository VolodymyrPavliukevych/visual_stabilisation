# Visual stabilisation
Optical Flow navigation, stabilisation


# Ardupilot Simulator 
### Gudes and usefull links

[Short course](https://www.youtube.com/watch?v=m7hPyJJmWmU)
[Official documentation](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)

## 1. Install Prerequisites

### Install gz
```bash
brew tap osrf/simulation
brew install gz-harmonic
```

## 2. Install ArduPilot

### Clone and Setup
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
cd ~/Projects/BFO/
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
cd /Users/roaming/Projects/BFO/ardupilot/Tools/autotest
python3.12 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
Arm and takeoff
```

### Some control
```
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5

```