# SLAM_with_TRIK_and_LIDAR
Simultaneously localization and mapping with TRIK and LIDAR

## Autonomous SLAM using TRIK Controller + LiDAR (Direct Wi-Fi Bridge)
In this phase, we bypassed the Raspberry Pi entirely and connected the LiDAR directly to the **TRIK Controller's UART port**. We created a custom, zero-lag, full-duplex TCP/IP bridge to stream Raw LiDAR, Wheel Encoders, and IMU (Gyroscope) data directly over Wi-Fi to the laptop for **Simultaneous Localization and Mapping (SLAM)**.

### 🔌 1. Hardware & Wiring Setup
We utilized the TRIK controller as the sole hardware abstraction layer.
* **LiDAR to TRIK (UART):** 
  * `5V` ➔ `5V`
  * `GND` ➔ `GND`
  * `TX` ➔ TRIK `RX` (Receive)
  * `RX` ➔ TRIK `TX` (Transmit)
* **Motors & Encoders:** Left Motor = `M3` / `E3`, Right Motor = `M4` / `E4`.
* **Configuration:** In the TRIK Web Configurator, the **UART** dropdown was set to `lidar` to utilize the TRIK's internal high-speed C++ driver.

---

### 🧠 2. The Edge Server (Run on TRIK Controller)
Since the TRIK does not run ROS 2 natively, we wrote a Python socket server to stream the raw sensor data at 20Hz and receive driving commands asynchronously.

**1. Create the script on the TRIK via SSH:**
```bash
nano scripts/direct_stream.py
```
(Save and exit. Run this script directly from the physical TRIK LCD Screen under Files -> scripts).

___________________________________________________________________________________________________

## 3. The ROS 2 Base Station (Run on Ubuntu Laptop)

The laptop catches the Wi-Fi stream, performs Sensor Fusion (combining Wheel Odometry with IMU Gyroscope data to prevent wheel-slip errors), and translates the data into official ROS 2 formats (LaserScan, Odometry, TF).

1. Create the Bridge Node on the Laptop:
```bash
nano ~/ros2_bridge.py
```

code for both "direct_stream.py" and "ros2_bridge.py" are provided.
___________________________________________________________________________________________________

## 4. SLAM Launch Sequence

With the scripts in place, we launch the ROS 2 environment using 5 separate terminals on the Ubuntu Laptop.
(Note: We compiled slam_toolbox from source in a local ~/lidar_ws workspace due to ROS 2 Lyrical missing apt binaries).

Terminal 1: Start the Wi-Fi Bridge:

```bash
source /opt/ros/*/setup.bash
python3 ~/ros2_bridge.py
```

Terminal 2: Publish LiDAR Static Transform:

```bash
source /opt/ros/*/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.15 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_footprint --child-frame-id laser

```

Terminal 3: Launch SLAM Toolbox:

```bash
source ~/lidar_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

Terminal 4: Keyboard Teleoperation:

```bash
sudo apt install ros-*-teleop-twist-keyboard -y
source /opt/ros/*/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 5: Launch RViz2:
```bash
source /opt/ros/*/setup.bash
rviz2
```

## 5. Mapping the Room

* In RViz2, set Fixed Frame to map.
* Add the Map display (By topic -> /map -> Map). Change Durability to Transient Local to prevent yellow warning triangles.
* Add the LaserScan display (By topic -> /scan -> LaserScan).
* Click onto Terminal 4 and use the i, j, l, , keys to drive the robot around the room.

As the robot drives, the Sensor Fusion (IMU + Wheels) ensures perfectly straight walls, and slam_toolbox dynamically generates a 2D floorplan of the environment!
