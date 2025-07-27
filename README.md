# ROS2-Sphero-Mini

## ROS 2 Sphero Mini - Vision Tracking & Teleoperation
This project uses ROS 2 with a Tkinter GUI and OpenCV color‑tracking to control a Sphero Mini to autonomously track a target. 

## Features
**GUI**: On‑screen joystick & D‑pad publish speed/heading; buttons toggle autonomy, stabilization, and reset heading; RGB sliders set the Sphero LED.

**Autonomous tracking**: OpenCV detects blue (robot) and red (target) objects, computes bearing & distance, and publishes desired yaw/velocity to the robot.

**BLE driver**: bluepy wrapper sends roll, stabilization, resetHeading, setLEDColor, queries battery/firmware, and IMU streaming.

## Systems Used
**Robotics middleware**: ROS 2 nodes, topic publisher/subscribers, launch files, namespaces.

**Computer Vision and Control**: HSV masking, contour extraction, centroiding, simple P‑controller, and real‑time video logging.

**UI**: Tkinter event‑driven GUI integrated with rclpy via a background spin thread.

## High Level Architecture
+--------------------+                   +--------------------+
|  usb_cam (driver)  |  /camera1/image   |   tracker_node     |
|  - MJPG/YUYV       +------------------->   (OpenCV HSV)     |
|  - 30 FPS          |                   | - blob detect      |
+---------+----------+                   | - centroid/bearing |
          |                              | - P ctrl (velocity)|
          |                              +----+-----+---------+
          |                                   |     |
          |                                   |     |
          |                                   v     v
          |                               /sphero/desired_yaw       (Int64)
          |                               /sphero/desired_distance  (Int64)
          |                               /sphero/desired_velocity  (Int64)
          |                               /sphero/move_flag         (Int64)
          v
+---------------------+                 +---------------------------+
|       gui           |  /GUI/*         |        sphero_node        |
| (Tkinter + rclpy)   +----------------->  (ROS2 ↔ BLE bridge)      |
| - joystick/D-pad    |                 | - autonomy/teleop modes   |
| - RGB sliders       |                 | - roll/stab/reset/LED     |
| - track/teleop btns |                 | - battery/firmware query  |
+----------+----------+                 | - optional IMU stream*    |
           |                            +-------------+-------------+
           |                                          |
           |                                          v
           |                                     BLE (bluepy)
           |                                Sphero Mini hardware
           |
           +--(background thread runs rclpy.spin; Tk mainloop on main thread)

## Quick Setup

**System (example for Jazzy)**
sudo apt-get install ros-jazzy-usb-cam ros-jazzy-cv-bridge python3-tk python3-venv python3-dev

**BLE venv (bluepy)**
mkdir -p ~/ros2_ble_ws && cd ~/ros2_ble_ws
python3 -m venv venv && source venv/bin/activate
pip install --upgrade pip bluepy opencv-python "numpy<2"
touch venv/COLCON_IGNORE

**Build package**
cd ~/ros2_ws && colcon build --packages-select ros2_sphero_mini
source install/setup.bash

## References
ROS1 Sphero Mini: cedricgoubard/sphero_mini
