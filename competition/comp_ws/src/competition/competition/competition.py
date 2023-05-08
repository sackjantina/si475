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

        self.initialized = False
        
        # subscriber to read the wheel ticks 
        # self.subscriber = self.create_subscription(WheelTicks, '/wheel_ticks', self.tick_callback, qos_profile_sensor_data)

        # ros2 topic pub --once /angle std_msgs/Float32 "data: 0.785"
        # self.turn_sub = self.create_subscription(Float32, '/angle', self.turn, qos_profile_sensor_data)
        
        self.create_subscription(Image, '/kinect_image', self.process_image, qos_profile_system_default)

        # Create subscription to get battery percentage
        self.create_subscription(BatteryState, '/battery_state', self.get_battery, qos_profile_sensor_data)
        self.battery_percent = 0.0

        # After setting the initial pose, from the command line run: ros2 topic pub -1 /start_navigation std_msgs/String
        self.create_subscription(String, '/start_navigation', self.navigate, qos_profile_system_default) 
        
        self.create_subscription(String, '/return_home', self.go_home, qos_profile_system_default) 
        self.create_subscription(String, '/leave_home', self.go_home, qos_profile_system_default) 


        self.time_interval = 5
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
        params.minArea = 500
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.2
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.5

        self.detector = cv2.SimpleBlobDetector_create(params)

        self.blobs = []

        self.state = "Searching"

        self.navigator = BasicNavigator() 
        self.navigator.waitUntilNav2Active()

        self.return_home = False
        self.home = self.get_PoseStamped(-3.25, -0.405, -1.285)


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

        # img = self.bridge.imgmsg_to_cv2(msg)

        # # path to the weights and model files
        # weights = "ssd_mobilenet/frozen_inference_graph.pb"
        # model = "ssd_mobilenet/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
        # # load the MobileNet SSD model trained  on the COCO dataset
        # net = cv2.dnn.readNetFromTensorflow(weights, model)

        # # load the class labels the model was trained on
        # class_names = []
        # with open("ssd_mobilenet/coco_names.txt", "r") as f:
        #     class_names = f.read().strip().split("\n")

        # # create a list of random colors to represent each class
        # np.random.seed(42)
        # colors = np.random.randint(0, 255, size=(len(class_names), 3))

        # # loop over the frames
        # while True:
        #     # starter time to computer the fps
        #     start = datetime.datetime.now()

        #     # create a blob from the frame
        #     blob = cv2.dnn.blobFromImage(
        #         img, 1.0/127.5, (320, 320), [127.5, 127.5, 127.5])
        #     # pass the blog through our network and get the output predictions
        #     net.setInput(blob)
        #     output = net.forward() # shape: (1, 1, 100, 7)

        #     # loop over the number of detected objects
        #     for detection in output[0, 0, :, :]: # output[0, 0, :, :] has a shape of: (100, 7)
        #         # the confidence of the model regarding the detected object
        #         probability = detection[2]
        #         # if the confidence of the model is lower than 50%,
        #         # we do nothing (continue looping)
        #         if probability < 0.5:
        #             continue

        #         # extract the ID of the detected object to get
        #         # its name and the color associated with it
        #         class_id = int(detection[1])
        #         label = class_names[class_id - 1].upper()
        #         color = colors[class_id]
        #         B, G, R = int(color[0]), int(color[1]), int(color[2])
        #         # perform element-wise multiplication to get
        #         # the (x, y) coordinates of the bounding box
        #         box = [int(a * b) for a, b in zip(detection[3:7], [w, h, w, h])]
        #         box = tuple(box)
        #         # draw the bounding box of the object
        #         cv2.rectangle(frame, box[:2], box[2:], (B, G, R), thickness=2)

        #         # draw the name of the predicted object along with the probability
        #         text = f"{label} {probability * 100:.2f}%"
        #         cv2.putText(frame, text, (box[0], box[1]),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        #     # end time to compute the fps
        #     end = datetime.datetime.now()
        #     # calculate the frame per second and draw it on the frame
        #     fps = f"FPS: {1 / (end - start).total_seconds():.2f}"
        #     cv2.putText(frame, fps, (50, 50),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 8)
        #     cv2.imshow("Output", frame)
        #     # write the frame to disk
        #     writer.write(frame)
        #     if cv2.waitKey(10) == ord("q"):
        #         break

        # # release the video capture, video writer, and close all windows
        # video_cap.release()
        # writer.release()
        # cv2.destroyAllWindows()

        

        # # Show image
        # cv2.imshow("Blob detection", frame)
        # cv2.waitKey(100)

        img = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hue, saturation, value = cv2.split(hsv[:,:])
        ret, thresh = cv2.threshold(saturation, np.average(saturation)+ 2*np.std(saturation), 255, cv2.THRESH_BINARY)

        mask = cv2.erode(thresh, None, iterations=4)
        mask = cv2.dilate(mask, None, iterations=4)
        mask = 255-mask

        keypoints = self.detector.detect(mask)
        blob_img = cv2.drawKeypoints(mask, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        self.imheight, self.imwidth, _ = blob_img.shape

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

    def navigate(self, msg):
        # self.return_home = True
        self.initialized = True

    def go_home(self, msg):
        self.return_home = True

    def leave_home(self, msg):
        self.return_home = False

    def patrol(self):
        path = []
        poses = [ 
            [0.25, -3.54, 0.23],
            [3.46, -2.85, 1.75],
            [3.07, -0.57, -2.87],
            [-0.295, -1.03, -1.34]
         ]
        
        for pose in poses:
            p = self.get_PoseStamped(pose[0], pose[1], pose[2])
            path.append(p)

        # Start navigation
        self.navigator.goThroughPoses(path)

    def state_machine(self):
        if not self.initialized:
            return

        # self.get_logger().error("running state machine")
        # print("return home:", self.return_home)

        # check battery level
        if self.battery_percent < 0.2 or self.return_home:
            self.state = "Return"
            self.get_logger().error(self.state)

            # cancel patrol task
            if not self.navigator.isTaskComplete():
                self.navigator.cancelTask()

            self.navigator.goToPose(self.home)
            return

        # check if heighest weighted particle is near boundary

        # check if any balloons are in view
        if len(self.blobs) > 0:
            self.state = "Hunting"
            self.get_logger().error(self.state)

            # cancel patrol task
            if not self.navigator.isTaskComplete():
                self.navigator.cancelTask()

            balloon = self.blobs[0]
            offset = -1*(balloon.pt[0] - (self.imwidth//2)) / (self.imwidth*1.5)

            cmd = Twist()
            cmd.linear.x = 0.1
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = offset
            self.publisher.publish(cmd)

        else:
            self.state = "Searching"
            self.get_logger().error(self.state)

            if not self.navigator.isTaskComplete():
                    self.navigator.cancelTask()

            self.patrol()

            # cmd = Twist()
            # cmd.linear.x = 0.0
            # cmd.linear.y = 0.0
            # cmd.linear.z = 0.0
            # self.publisher.publish(cmd)
        


def main(args=None):
    rclpy.init(args=args)
    node = Competition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
