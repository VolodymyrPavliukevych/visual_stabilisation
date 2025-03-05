## Copter setup

### Modify default params

```
# At default_params/gazebo-iris.parm
# Custom params
GPS_TYPE 0              # Disable GPS
ARMING_CHECK 0          # Disable arming checks
EK3_SRC1_POSXY 6        # Use Optical Flow instead of GPS
EK3_SRC1_VELXY 6        # Use Optical Flow velocity
EK3_SRC1_POSZ 1         # Use Barometer for altitude
EK3_SRC1_VELZ 3         # Use Accelerometer for vertical velocity
AHRS_EKF_TYPE 3         # Use EKF3 instead of EKF2
EK3_ENABLE 1            # Enable EKF3
FLOW_TYPE 1

```

### Add one more port to mavlink
```
cd /Users/roaming/Projects/BFO/ardupilot/Tools/autotest && \
python3.12 sim_vehicle.py -v ArduCopter -I0 -M "--out=127.0.0.1:14551"

cd /Users/roaming/Projects/BFO/ardupilot/Tools/autotest && \
python3.12 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0 -M "--out=127.0.0.1:14551"

# Run simulator
gz sim -v4 -r -s iris_runway.sdf
gz sim -v4 -r -g iris_runway.sdf


gz sim -v4 -r iris_warehouse.sdf -s
gz sim -v4 -r iris_warehouse.sdf -g

```

By default, **GUIDED mode requires GPS**, but you can **start GUIDED mode without GPS** by making some parameter changes. Here’s how:  


### Allow GUIDED Mode Without GPS
Set the following parameters to **allow GUIDED mode without GPS**:  
```bash
param set GPS_TYPE 0              # Disable GPS
param set ARMING_CHECK 0          # Disable arming checks
param set EK3_SRC1_POSXY 6        # Use Optical Flow instead of GPS
param set EK3_SRC1_VELXY 6        # Use Optical Flow velocity
param set EK3_SRC1_POSZ 1         # Use Barometer for altitude
param set EK3_SRC1_VELZ 3         # Use Accelerometer for vertical velocity
param set AHRS_EKF_TYPE 3         # Use EKF3 instead of EKF2
param set EK3_ENABLE 1            # Enable EKF3
```
Then, **reboot**:  
```bash
reboot
```


### Force Enable GUIDED Mode (No GPS)
After disabling GPS, try switching to GUIDED mode:  
```bash
mode guided
```
If you get an error, set:  
```bash
param set GUIDED_MINSPD 0
```