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
        params.minThreshold = 0
        params.maxThreshold = 255
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 400
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.01
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        self.detector = cv2.SimpleBlobDetector_create(params)

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
        
        if len(keypoints) > 0:
            for k in keypoints:
                print(f"({k.pt}): {k.size}")


        cv2.imshow('original', img)
        # cv2.imshow('mask', mask)
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
