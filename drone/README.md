## To run drone

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

---

### ✅ **1. Allow GUIDED Mode Without GPS**  
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

---

### **2. Force Enable GUIDED Mode (No GPS)**
After disabling GPS, try switching to GUIDED mode:  
```bash
mode guided
```
If you get an error, set:  
```bash
param set GUIDED_MINSPD 0
```
Then try again.


### Turn camera on
```
param set CAM_TRIGG_TYPE 1   # 1 for servo, 4 for mavlink
param set CAM_VID_TYPE 2      # 2 for UDP streaming
param set CAM_VID_FMT 1       # 1 for H264 format

```


The **MAVLink Optical Flow message (`OPTICAL_FLOW`)** provides data from an **optical flow sensor** mounted on a drone. Optical flow sensors help estimate **relative movement** by analyzing **pixel shifts** in images captured by the sensor. This is particularly useful for **positioning** when **GPS is unavailable**.

---

## 📌 **MAVLink `OPTICAL_FLOW` Message Fields**:
| **Field**            | **Type**   | **Description** |
|----------------------|-----------|---------------|
| **`time_usec`**     | `uint64_t` | Timestamp (microseconds since boot or UNIX epoch). |
| **`sensor_id`**     | `uint8_t`  | ID of the optical flow sensor (useful when multiple sensors are used). |
| **`flow_x`**        | `int16_t`  | Raw **optical flow measurement** in **pixels** (X-axis). |
| **`flow_y`**        | `int16_t`  | Raw **optical flow measurement** in **pixels** (Y-axis). |
| **`flow_comp_m_x`** | `float`    | **Compensated optical flow** in **meters** per second (X-axis). |
| **`flow_comp_m_y`** | `float`    | **Compensated optical flow** in **meters** per second (Y-axis). |
| **`quality`**       | `uint8_t`  | Quality of the optical flow measurement (0 = bad, 255 = perfect). |
| **`ground_distance`** | `float`    | Distance to ground in **meters** (measured via a rangefinder, if available). |
| **`flow_rate_x`**   | `float`    | Angular flow rate in **radians/second** (X-axis). |
| **`flow_rate_y`**   | `float`    | Angular flow rate in **radians/second** (Y-axis). |

---

## 🔍 **Detailed Explanation of Each Field:**
### **1️ `time_usec` (Timestamp)**
- Time in **microseconds** when the measurement was taken.
- Helps **synchronize** data across multiple sensors.

### **2️ `sensor_id` (Sensor ID)**
- Identifies **which sensor** the data came from (useful if multiple sensors are used).

### **3️ `flow_x` and `flow_y` (Raw Optical Flow)**
- Measures how **pixels shift** in the camera image due to drone movement.
- **Units**: Pixels
- **Example**: If the drone moves forward, objects in the image move **backward**, creating a shift in pixel position.

### **4️ `flow_comp_m_x` and `flow_comp_m_y` (Compensated Optical Flow)**
- Converts **raw pixel movement** into **real-world movement** in meters per second.
- Compensation is done using the **ground distance** (to account for scale).
- **Units**: Meters per second (**m/s**)
- **Example**: A drone moving at **1 m/s forward** would have `flow_comp_m_x = 1.0`.

### **5️ `quality` (Measurement Quality)**
- **0** = No reliable data (bad lighting, too much motion blur).
- **255** = Best possible quality.
- If **quality is low**, ignore or discard the measurement.

### **6️ `ground_distance` (Height Above Ground)**
- Distance between the sensor and the ground.
- **Measured by a rangefinder (e.g., LiDAR, ultrasonic sensor)**.
- **Units**: Meters (**m**).
- Used for **scale correction** (converting pixel shifts to real-world motion).

### **7️ `flow_rate_x` and `flow_rate_y` (Angular Flow Rate)**
- Represents the **rate of change of flow** in **radians per second**.
- Helps estimate **angular motion** (rotation).
- **Useful for yaw drift correction** in **GPS-denied environments**.

---

##  **How It’s Used in GPS-Denied Navigation**
1. **Read Optical Flow (`flow_comp_m_x`, `flow_comp_m_y`)** to estimate **velocity**.
2. **Use `ground_distance`** to scale movement properly.
3. **Use `quality` to filter out bad data**.
4. **Integrate velocities** over time to estimate **position**.
5. **Combine with IMU (gyroscope + accelerometer) for better accuracy**.

---

##  **Example: Using Optical Flow for Drone Stabilization**
- If `flow_comp_m_x = 0.5` and `flow_comp_m_y = 0`, the drone is moving **forward at 0.5 m/s**.
- If the drone should hover, the **PID controller** adjusts motor speeds to bring `flow_comp_m_x = 0`.

---

Let me know if you need an example **Python script** to process optical flow for stabilization! 