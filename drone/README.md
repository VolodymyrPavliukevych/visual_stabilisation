## Copter setup

### Modify default params

```
# At default_params/gazebo-iris.parm
# Custom params
GPS_TYPE 0              # Disable GPS
ARMING_CHECK 0          # Disable arming checks
EK3_SRC1_POSXY 0        
EK3_SRC1_VELXY 5        
EK3_SRC1_POSZ 1         # Use Barometer for altitude
EK3_SRC1_VELZ 0         # Use Accelerometer for vertical velocity
AHRS_EKF_TYPE 3         # Use EKF3 instead of EKF2
EK3_ENABLE 1            # Enable EKF3
FLOW_TYPE 10
FLOW_FXSCALER 0.0
FLOW_FYSCALER 0.0
FLOW_ORIENT_YAW 0
SIM_FLOW_ENABLE 1

```

### Add one more port to mavlink
```
#cd ~/Projects/BFO/ardupilot/Tools/autotest && \
#python3.12 sim_vehicle.py -v ArduCopter -I0 -M "--out=127.0.0.1:14551"

cd ~/Projects/BFO/ardupilot/Tools/autotest && \
python3.12 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0 -M "--out=127.0.0.1:14551"

export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/ardupilot_gazebo
export GZ_SIM_RESOURCE_PATH=/usr/local/share/ardupilot_gazebo/worlds:/usr/local/share/ardupilot_gazebo/models
export GZ_VERSION harmonic

#LD_TRACE_LOADED_OBJECTS=1


# Run runway simulator Server and GUI
gz sim -v4 -r -s iris_runway.sdf
gz sim -v4 -r -g iris_runway.sdf

# Or warehouse environment
gz sim -v4 -r iris_warehouse.sdf -s
gz sim -v4 -r iris_warehouse.sdf -g

```

By default, **GUIDED mode requires GPS**, but you can **start GUIDED mode without GPS** by making some parameter changes. Here’s how:  


### Allow GUIDED Mode Without GPS
Set the following parameters to **allow GUIDED mode without GPS**:  
```bash
param set GPS_TYPE 0              # Disable GPS
param set ARMING_CHECK 0          # Disable arming checks
param set EK3_SRC1_POSXY 0        
param set EK3_SRC1_VELXY 5        
param set EK3_SRC1_POSZ 1         # Use Barometer for altitude
param set EK3_SRC1_VELZ 0         
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


### Troubleshoot Optical Flow issue

Check parameters

```
EK3_SRC1_POSXY  0
EK3_SRC1_VELXY  5
EK3_SRC1_VELZ   0
FLOW_TYPE       10
SIM_FLOW_ENABLE 1
```