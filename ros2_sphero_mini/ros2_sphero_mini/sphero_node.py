#!/usr/bin/env python3

import sys
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg       import Int64, String, Bool

from ros2_sphero_mini.core import SpheroMini
from ros2_sphero_mini.simple_moves import forward, initSphero
from rclpy.qos import QoSPresetProfiles

class SpheroNode(Node):
    def __init__(self):
        super().__init__('sphero_node')

        # State variables 
        self.move = 0
        self.auto = False
        self.last_sent_yaw = None
        self.last_heading = 0
        self.last_vel = 0
        self.heading = 0
        self.velocity = 0
        self.distance = 0

        # Declare & read the MAC_ADDRESS parameter
        # self.declare_parameter('MAC_ADDRESS', 'E0:D2:31:6B:74:3C')
        # mac = self.get_parameter('MAC_ADDRESS').value
        self.declare_parameter('mac_address', 'E0:D2:31:6B:74:3C')
        self.declare_parameter('conf_file_path', '')

        mac = self.get_parameter('mac_address').get_parameter_value().string_value
        cfg = self.get_parameter('conf_file_path').get_parameter_value().string_value
        self.get_logger().info(f'Connecting to Sphero {mac}, with config file {cfg}')

        # Connect to the Sphero
        self.sphero = SpheroMini(mac, verbosity=4)
        # initSphero(self.sphero)

        self.sphero.getBatteryVoltage()
        self.get_logger().info(f'Battery voltage: {self.sphero.v_batt} V')
        self.sphero.returnMainApplicationVersion()
        fw = ".".join(str(x) for x in self.sphero.firmware_version)
        self.get_logger().info(f'Firmware version: {fw}')

        # # Configure IMU streaming
        # self.sphero.configureSensorMask(
        #     IMU_yaw=True, IMU_pitch=True, IMU_roll=True,
        #     IMU_acc_x=True, IMU_acc_y=True, IMU_acc_z=True,
        #     IMU_gyro_x=True, IMU_gyro_y=True
        # )
        # self.sphero.configureSensorStream()

        # Publisher and timer
        # self.imu_pub = self.create_publisher(Imu, '/imu', 5)
        # self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz


        #  Subscribers (tracker topics) 
        self.create_subscription(Int64,'/sphero/desired_yaw', self.yaw_callback, 5)
        self.create_subscription(Int64,'/sphero/desired_distance', self.dist_callback, 5)
        self.create_subscription(Int64,'/sphero/move_flag', self.move_flag_callback, 5)
        self.create_subscription(Int64,'/sphero/desired_velocity', self.velocity_callback,  5)

        #  Subscribers (GUI topics) 
        self.create_subscription(String, '/GUI/dpad',  self.dpad_callback, 5)
        self.create_subscription(Vector3,'/GUI/joystick', self.joystick_callback, 5)
        self.create_subscription(Vector3,'/GUI/slider', self.slider_callback, 5)
        self.create_subscription(Bool,'/GUI/autonomy', self.autonomy_callback, 5)
        self.create_subscription(Bool,'/GUI/stabilization', self.stab_callback, 5)
        self.create_subscription(Bool,'/GUI/reset_heading', self.reset_callback, 5)
        
    # for publishing imu data
    def timer_callback(self):
        
        data = {
            "roll": self.sphero.IMU_roll,
            "pitch": self.sphero.IMU_pitch,
            "yaw": self.sphero.IMU_yaw,
            "groll": self.sphero.IMU_gyro_x,
            "gpitch": self.sphero.IMU_gyro_y,
            "xacc":self.sphero.IMU_acc_x,
            "yacc":self.sphero.IMU_acc_y,
            "zacc":self.sphero.IMU_acc_z,
        }

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = self._to_quaternion(
            data["roll"], data["pitch"], data["yaw"]
        )
        msg.angular_velocity = self._to_vector3(
            data["groll"], data["gpitch"], 0.0
        )
        msg.linear_acceleration = self._to_vector3(
            data["xacc"], data["yacc"], data["zacc"]
        )

        self.imu_pub.publish(msg)
        self.sphero.setLEDColor(red=0, green=255, blue=0)

    def _to_quaternion(roll, pitch, yaw):
        q = Quaternion()
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def _to_vector3(x, y, z):
        v = Vector3()
        v.x, v.y, v.z = x, y, z
        return v 
    
    
    def yaw_callback(self, msg: Int64):
        # sphero_heading = (bearing_deg + 90) % 360
        self.heading = (msg.data + 90) % 360
        self.get_logger().info(f"[yaw_callback] data={self.heading}")

        if self.auto: # if tracking mode
            if self.heading < 0:
                self.heading += 360
            if self.move == 1: # far enough from target
                self.sphero.roll(self.velocity, self.heading) # roll command
                self.last_sent_yaw = self.heading # update last yaw
            else:
                self.get_logger().info("[yaw_callback] Not Moving")
    
    def dist_callback(self, msg: Int64):
        # self.get_logger().info(f"[dist_callback] data={msg.data}")
        self.distance = msg.data

    def move_flag_callback(self, msg: Int64):
        # self.get_logger().info(f"[move_flag_callback] data={msg.data}")
        self.move = msg.data

    def velocity_callback(self, msg: Int64):
        # self.get_logger().info(f"[velocity_callback] data={msg.data}")
        self.velocity = msg.data

    def dpad_callback(self, msg: String):
        self.get_logger().info(f"[dpad_callback] data='{msg.data}'")
        if not self.auto: # teleop mode
            cmd = msg.data 
            # dict for dpad dir mapping
            mapping = {
                "Up": (100,   0),
                "Down": (100, 180),
                "Left":(100, 270),
                "Right":(100,  90),
                "None": (  0, self.last_heading),
            }
            speed, head = mapping.get(cmd, (0, self.last_heading))
            self.sphero.roll(speed, head)
            self.sphero.wait(0.2) # only roll for a small amount
            self.sphero.roll(0, head)
            self.last_heading = head

    def joystick_callback(self, msg: Vector3):
        self.get_logger().info(f"[joystick_callback] x={msg.x}, y={msg.y}, z={msg.z}")
        speed = int(msg.x)
        head  = int(msg.y)
        if not self.auto: # teleop mode
            if head < 0:
                head += 360
            if abs(speed - self.last_vel) >= 8:
                self.sphero.roll(speed, head)
                self.last_vel = speed

    def slider_callback(self, msg: Vector3):
        self.get_logger().info(f"[slider_callback] r={msg.x}, g={msg.y}, b={msg.z}")
        self.sphero.setLEDColor(red=int(msg.x), green=int(msg.y), blue=int(msg.z))

    def autonomy_callback(self, msg: Bool):
        self.get_logger().info(f"[autonomy_callback] auto={msg.data}")
        self.auto = msg.data
    def stab_callback(self, msg: Bool):
        self.get_logger().info(f"[stab_callback] stab={msg.data}")
        self.sphero.setLEDColor(red = 255, green = 0, blue = 0) # Turn main LED red
        self.sphero.setBackLEDIntensity(255) # turn back LED on
        self.sphero.stabilization(msg.data)
    def reset_callback(self, msg: Bool):
        self.get_logger().info(f"[reset_callback] reset={msg.data}")
        self.sphero.setBackLEDIntensity(0) # turn back LED on
        self.sphero.resetHeading()
    
    
    def destroy(self):
        # self.timer.cancel()
        self.sphero.sleep()
        self.sphero.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpheroNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Caught Ctrl-C, shutting down…")

    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



