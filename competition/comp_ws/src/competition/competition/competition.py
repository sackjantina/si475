import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_system_default
import cv2
import numpy as np


class Competition(Node):
    def __init__(self):
        super().__init__("Competition")
        
        self.create_subscription(Image, '/kinect_image', self.process_image, qos_profile_system_default)

        self.bridge = CvBridge() # create a CvBridge object for later use

        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 240;
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 200
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.8
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.1
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.008

        self.detector = cv2.SimpleBlobDetector_create(params)

    def process_image(self, msg):
        # self.get_logger().error("Processing Image")

        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # thresh_val = np.average(img)
        # ret, thresh = cv2.threshold(img, thresh_val, 255, cv2.THRESH_BINARY)

        keypoints = self.detector.detect(img)
        blob_img = cv2.drawKeypoints(img, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow('img', img)
        cv2.imshow('hue', img[:,:,0])
        cv2.imshow('saturation', img[:,:,1])
        cv2.imshow('value', img[:,:,2])
        # cv2.imshow('thresholded image', thresh)
        cv2.imshow('blobbed image', blob_img)
        cv2.waitKey(100) 



def main(args=None):
    rclpy.init(args=args)
    node = Competition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
