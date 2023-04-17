from action_msgs.msg import GoalStatus 
from copy import deepcopy 
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import String 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy 
from rclpy.node import Node 
from rclpy.action import ActionClient 
from rclpy.duration import Duration 
from rclpy.qos import qos_profile_system_default

from tf_transformations import quaternion_from_euler

class Navigate(Node): 
    def __init__(self): 
        super().__init__("Navigate") 

        # Since we need to set the initial pose with Rviz, we will use a topic to start the navigation process.  
        # After setting the initial pose, from the command line run: ros2 topic pub -1 /start_navigation std_msgs/String
        # We don't care about the content of the message, only that a message was received.  
        self.create_subscription(String, '/start_navigation', self.navigate, qos_profile_system_default) 

        # this is how we interact with the Simple Commander API 
        self.navigator = BasicNavigator() 

        # wait to ensure that the Nav2 stack is running. 
        self.navigator.waitUntilNav2Active() 

    # here's where all the magic happens.  
    def navigate(self, msg): 
        # Set initial pose in RViz before starting 
        
        # This function will start once anything is published to /start_navigation topic
        self.get_logger().error("-------------- Starting Navigation --------------")

        # All poses we want to navigate to must be stamped with the correct header information.  Follow this 
        # example: 
        goal_pose = []

        # To create a path, you need a sequence of poses to follow. Create the object you'll pass to
        # self.navigator as follows: 
        poses = [ 
            [0.25, -3.54, 0.23],
            [3.46, -2.85, 1.75],
            [3.07, -0.57, -2.87],
            [-0.295, -1.03, -1.34]
         ] 
        # where each point is the X and Y position.  Note that all the values must be floating point.  The 
        # Python function reverse will come in handy for repeating your path.  

        for pose in poses:
            p = self.get_PoseStamped(pose[0], pose[1], pose[2])
            goal_pose.append(p)

        # Start navigation
        self.navigator.goThroughPoses(goal_pose)

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


    
def main(args=None):
    rclpy.init(args=args)
    node = Navigate() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown()
     
if __name__ == '__main__':
    main()

