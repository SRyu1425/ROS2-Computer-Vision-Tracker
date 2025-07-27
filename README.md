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
Camera -> Vision: A USB camera publishes frames; a vision node (OpenCV) detects robot/target colors, computes bearing + distance, and outputs simple motion intents.

Operator UI -> Control: A Tkinter GUI publishes joystick/D‑pad commands, LED color, and mode toggles (Autonomy/Teleop, Stabilization, Reset Heading).

Control Bridge: A ROS 2 node takes either the GUI commands (Teleop) or the tracking flag (Autonomy), applies control algorithms (e.g., heading offset, velocity cap), and decides what the robot should do next.

Robot I/O (BLE): A thin driver encodes Sphero API packets over BLE to execute actions (roll, stabilization on/off, reset heading, LED) and fetch device status (battery, firmware).

Launch & Namespacing: One launch file starts camera, vision, GUI, and control under a single namespace for topic organization.

**Data Flow**
Camera -> OpenCV -> Control -> BLE -> Sphero
GUI -> Control -> BLE -> Sphero

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

## Demo
The attched mp4 video files demonstrates both the tracking functionality with the GUI in view, as well as the manual teleoperation mode.

## References
ROS1 Sphero Mini BLE Driver: cedricgoubard/sphero_mini
