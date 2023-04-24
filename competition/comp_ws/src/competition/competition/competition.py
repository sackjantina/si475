import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, BatteryState
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String 


import cv2
import numpy as np


class Competition(Node):
    def __init__(self):
        super().__init__("Competition")
        
        # subscriber to read the wheel ticks 
        # self.subscriber = self.create_subscription(WheelTicks, '/wheel_ticks', self.tick_callback, qos_profile_sensor_data)

        # ros2 topic pub --once /angle std_msgs/Float32 "data: 0.785"
        # self.turn_sub = self.create_subscription(Float32, '/angle', self.turn, qos_profile_sensor_data)
        
        self.create_subscription(Image, '/kinect_image', self.process_image, qos_profile_system_default)

        # Create subscription to get battery percentage
        self.create_subscription(BatteryState, '/battery_state', self.get_battery, qos_profile_sensor_data)
        self.battery_percent = 0.0

        self.time_interval = 0.5
        self.create_timer(self.time_interval, self.state_machine)

        # State Functions
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)


        self.bridge = CvBridge() # create a CvBridge object for later use

        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 100
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.5

        self.detector = cv2.SimpleBlobDetector_create(params)

        self.blobs = []

        self.state = "Searching"

        self.return_home = False
        self.home = self.get_PoseStamped(0.0, 0.0, 0.0)

        self.navigator = BasicNavigator() 
        self.navigator.waitUntilNav2Active()

        self.create_subscription(String, '/start_navigation', self.navigate, qos_profile_system_default) 


    def navigate(self, msg):
        self.return_home = True

    def get_PoseStamped(self, x, y, angle):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        x_q, y_q, z_q, w_q = quaternion_from_euler(0, 0, angle)
        pose.pose.orientation.x = x_q
        pose.pose.orientation.y = y_q
        pose.pose.orientation.z = z_q
        pose.pose.orientation.w = w_q
        return pose

    def process_image(self, msg):
        # self.get_logger().error("Processing Image")

        img = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hue, saturation, value = cv2.split(hsv[:,:])
        ret, thresh = cv2.threshold(saturation, np.average(saturation)+ 3*np.std(saturation), 255, cv2.THRESH_BINARY)

        mask = cv2.erode(thresh, None, iterations=4)
        mask = cv2.dilate(mask, None, iterations=4)
        mask = 255-mask

        keypoints = self.detector.detect(mask)
        blob_img = cv2.drawKeypoints(mask, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        h,w,c = blob_img.shape
        middle = w/2

        if len(keypoints) > 0:
            for k in keypoints:
                print(f"{k.pt}: {k.size}")
                cmd = Twist()
                #if k.pt < middle: #left side of middle
                #   cmd.angular.z = 
                #if k.pt > middle: #right side of middle
                #   cmd.angular.z = 
                #else: # middle
                #    cmd.angular.z = 0.0
                #self.publisher.publish(cmd)
        cv2.imshow('original', img)
        # cv2.imshow('mask', mask)
        # cv2.imshow('thresholded image', thresh)
        cv2.imshow('blobbed image', blob_img)
        cv2.waitKey(100)

        self.blobs = keypoints

    def get_battery(self, msg):
        self.battery_percent = msg.percentage

    def state_machine(self):
        self.get_logger().error("running state machine")

        # check battery level
        if self.battery_percent < 0.2 or self.return_home:
            self.state = "Return"
            self.get_logger().error(self.state)
            self.navigator.goToPose(self.home)
            return

        # check if heighest weighted particle is near boundary

        # check if any balloons are in view
        if len(self.blobs) > 0:
            self.state = "Hunting"
            self.get_logger().error(self.state)
        else:
            self.state = "Searching"
            self.get_logger().error(self.state)
        


def main(args=None):
    rclpy.init(args=args)
    node = Competition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
