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

class Navigate(Node): 
    def __init__(self): 
        super().__init__("Navigate") 

        # Since we need to set the initial pose with Rviz, we will use a topic to start the navigation process.  
        # After setting the initial pose, from the command line run: ros2 topic pub -1 std_msgs/String  
        # We don't care about the content of the message, only that a message was received.  
        self.create_subscription(String, '/start_navigation', self.navigate, qos_profile_system_default) 

        # this is how we interact with the Simple Commander API 
        self.navigator = BasicNavigator() 

        # wait to ensure that the Nav2 stack is running. 
        self.navigator.waitUntilNav2Active() 

    # here's where all the magic happens.  
    def navigate(self, msg): 
        # All poses we want to navigate to must be stamped with the correct header information.  Follow this 
        # example: 
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        #goal_pose.pose.position =  
        #goal_pose.pose.orientation =  

        # To create a path, you need a sequence of poses to follow. Create the object you'll pass to
        # self.navigator as follows: 
        poses = [ 
            [1.2, 3.0], 
            [3.4, 1.34]
         ] 
        # where each point is the X and Y position.  Note that all the values must be floating point.  The 
        # Python function reverse will come in handy for repeating your path.  

        for pose in poses:
            goal_pose.append(navigator.getPoseStamped(pose, TurtleBot4Directions.North))

    
def main(args=None):
    rclpy.init(args=args)
    node = Navigate() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown()
     
if __name__ == '__main__':
    main()

