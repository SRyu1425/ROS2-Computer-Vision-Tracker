# tracker_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg    import Int64
from geometry_msgs.msg import Point

class TrackerNode(Node):
    def __init__(self):
        super().__init__('sphero_tracker')

        # Parameters
        self.declare_parameter('camera_topic', '/camera1/image_raw')
        cam_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        # CV Bridge & Video writer
        self.bridge = CvBridge() # instantiate bridge as CvBridge object
        fourcc = cv2.VideoWriter_fourcc(*'MJPG') # Create a file capable of holding video frames
        self.out = cv2.VideoWriter('Tracking.avi', fourcc, 30.0, (320,240))

        # each img is appended to this video file

        # Publishers (created once)
        # editing QoS profile
        # Reliability: send data once, but no need to retry (ok if message is occasionally dropped)
        # History: only buffer last (5) messages for lagging subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        self.pub_yaw   = self.create_publisher(Int64,   '/sphero/desired_yaw',     5)
        self.pub_dist  = self.create_publisher(Int64,   '/sphero/desired_distance', 5)
        self.pub_move  = self.create_publisher(Int64,   '/sphero/move_flag',        5)
        self.pub_vel   = self.create_publisher(Int64,   '/sphero/desired_velocity', 5)
        self.pub_robot = self.create_publisher(Point,   '/sphero/robot_pose',       5)
        self.pub_target= self.create_publisher(Point,   '/sphero/target_pose',      5)

        #  Subscription
        sub_qos = QoSProfile(depth=5)
        self.create_subscription(Image, cam_topic, self.image_callback, sub_qos)

        self.get_logger().info(f"Subscribed to {cam_topic}")

    def euclid(self, c1, c2): # calculate dist and bearing between 2 center points
        dist = np.hypot(c1[0]-c2[0], c1[1]-c2[1])
        bearing = np.arctan2(c2[1]-c1[1], c2[0]-c1[0])
        return dist, bearing

    def image_callback(self, msg: Image):
        # Convert
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # convert ROS image to BGR using CvBridge
        except Exception as e:
            self.get_logger().error(f"Bridge error: {e}")
            return

        # Resize & HSV
        # interlinear is bilinear interpolation to compute resized image (default)
        img = cv2.resize(img, (320,240), interpolation=cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert img from bgr to hsv

        # 2 masks because red spans 170-10 deg approx, so you have to go past 0 deg
        red1 = cv2.inRange(hsv, (0,70,50), (10,255,255))
        red2 = cv2.inRange(hsv, (170,70,50),(180,255,255))
        red_mask = cv2.bitwise_or(red1, red2)
        blue_mask= cv2.inRange(hsv, (90,0,125),(179,255,255))
        yellow_mask= cv2.inRange(hsv, (19,176,164),(53,255,255)) # yellow color mask
        
        # pixels inside the range are white (255) and all else is black (0)

        def find_center(mask):
            # find contours extracts all white regions 
            # cnts is a list of contours (each element is an array for the coords of the points along contour boundary)
            # retr_external: returns outermost contour (no contours inside other contours)
            # chain_approx: how pixels are stored. in between simple and none.
            # chain_approx_simple collapses straight runs of pixels into end poitns, and non conserves all.
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
            if not cnts: return None, None # if no contours return No center
            # max function takes a key arg (function applied to each elem in cnts)
            m = max(cnts, key=cv2.contourArea) # pick largest contour by area
            if cv2.contourArea(m) < 10: return None, None # if the largest contour is too small, discard it
            # 10 pixels limit
            x,y,w,h = cv2.boundingRect(m) # otherwise, get the bounding box and then calc the center
            return (x+w//2, y+h//2), m

        red_c, m_red  = find_center(red_mask)
        blue_c, m_blue = find_center(blue_mask)

        def _draw_detection(img, center, contour, color, label):
            # draw bounding boxes over the circle
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(img, (x,y), (x+w,y+h), color, 2)
            cx,cy = center
            cv2.circle(img, (cx,cy), 5, color, -1)
            cv2.putText(img, label, (x+10,y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        
        if red_c is not None and m_red is not None:
            _draw_detection(img, red_c, m_red, (0,0,255), "Red Sphero")
        if blue_c is not None and m_blue is not None:
            _draw_detection(img, blue_c, m_blue, (255,0,0), "Blue Sphero")

        if red_c and blue_c:
            # compute dist
            dist, theta = self.euclid(blue_c, red_c)
            dx = red_c[0] - blue_c[0]
            dy = red_c[1] - blue_c[1]
            # self.get_logger().info(f"[DEBUG] dx={dx}, dy={dy}, dist={dist:.1f}")
            # self.get_logger().info(f"[DEBUG] theta_rad={theta:.3f}")
            move_flag = 0 if dist <= 5 else 1 # if alr closer than 70 pixels, don't move
            vel = min(250, int(1.1*abs(dist-5))) # p controller, capped at 250

            bearing_deg = (np.degrees(theta) + 360) % 360
            # self.get_logger().info(f"[DEBUG] bearing_deg={bearing_deg:.1f}°")

            rob = Point(x=float(blue_c[0]), y=float(blue_c[1]), z=0.0)
            tgt = Point(x=float(red_c[0]),  y=float(red_c[1]),  z=0.0)
            yaw = Int64(data=int(bearing_deg))
            d   = Int64(data=int(dist))
            mv  = Int64(data=move_flag)
            vv  = Int64(data=vel)

            self.pub_robot.publish(rob)
            self.pub_target.publish(tgt)
            self.pub_dist.publish(d)
            self.pub_move.publish(mv)
            self.pub_vel.publish(vv)
            self.pub_yaw.publish(yaw)

        self.out.write(img) # out is the video file
        # each frame is appended to Tracking.avi file
        cv2.imshow("Robot Tracking", img) # displays live tracking window. technically queues but doesn't show until next line
        cv2.waitKey(1) # need this line for waiting 1 ms. Updates all imshow windows, handles clicks, key presses.

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.out.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()

