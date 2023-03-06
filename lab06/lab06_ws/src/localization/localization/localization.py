import rclpy, time, random
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data

from lifecycle_msgs.srv import ChangeState 
from lifecycle_msgs.msg import Transition

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid 
from std_msgs.msg import Header 
from builtin_interfaces.msg import Time 
from nav2_msgs.msg import Particle, ParticleCloud


class Locate(Node):

    def __init__(self):
        super().__init__('localization')

        # these two boolean values are to ensure that steps happen in the appropriate order 
        self.initialized = False 
        self.has_map = False 

        # the following block of code sets up the service call to the map server 
        # First, it creates the client of the service (the server of the service is 
        # created via the launch file).  Then it waits until the service is 
        # available (since we don't know in what order the ROS nodes were launched, 
        # nor how long it takes for them to become ready).  Then a service message is 
        # created, with the ID of 1 corresponding to configure, and ID of 3 to activate.  
        # As mentioned on the lab webpage, the map server only publishes the map one time, 
        # and since both this code and Rviz need the map, we need to wait a bit between 
        # configuring and activating to allow Rviz to finish initialization.   Also note 
        # that we need to create the map subscriber prior to calling the map server service.  
        # If the subscription was created after the service call, then the subscriber would 
        # miss the single publication of the map.  
        self.client = self.create_client(ChangeState, "/map/change_state") 
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.get_map, qos_profile_sensor_data) 
        while not self.client.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info("Waiting for change_state service")
        req = ChangeState.Request() 
        req.transition = Transition() 
        req.transition.id = 1 
        future = self.client.call_async(req) 
        rclpy.spin_until_future_complete(self, future) 
        time.sleep(1) 
        req.transition.id = 3 
        future = self.client.call_async(req) 
        rclpy.spin_until_future_complete(self, future) 

        # set up some publishers and subscribers 
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.locate, qos_profile_sensor_data) 
        self.cloud_pub = self.create_publisher(ParticleCloud, '/particle_cloud', 10) 

        # this is the set of particles.  Conviently, nav2_msgs has a Particle and 
        # ParticleCloud message already.  
        self.particle_cloud = ParticleCloud()  

        # the best estimate of the robot's pose.  Useful for 
        # debugging
        self.robot_estimate = Pose() 

        # the map received from the map server 
        self.map = OccupancyGrid() 

        # number of particles.  Probably need to adjust this number 
        self.num_particles = 10000 

        # intiialize the particle cloud 
        self.init_particle_cloud() 
    
        # now that we have our particles, we can proceed with the 
        # localization algorithm 
        self.initialized = True 

    # this function should set the self.map variable, and also set other information 
    # about the map such as width, height, resolution, etc.  See the OccupancyGrid message 
    # documentation on-line.  
    def get_map(self, msg): 
        # TODO 
        print('getting map...')

    # initialize the particles with random values.  You will need to determine the appropriate 
    # values for x and y that are consistent with the map.  Using the map resolution, and the map
    # origin will be required.  Also, to create a random Quaternion, you can pass four random 
    # numbers in [0, 1] to the constructor.  For initial weights, set them all to 0.5  
    def init_particle_cloud(self): 
        # TODO 
        pass


    # heres where you'll implement the particle filter.  So, your localization algorithm will 
    # run everytime a laser scan comes in.  Use the self.initialized variable to ensure everything 
    # is ready prior to starting your algorithm.  You should also check if the robot has moved some 
    # minimum amount prior to running the particle filter.  The /odom topic is helpful here.  
    def locate(self, msg): 
        # implement the complete particle filter here 
        pass


    # Helper function to publish the current set of particles, 
    # so rviz can visualize them 
    def publish_particle_cloud(self):
        self.particle_cloud.header =  Header(stamp=self.get_clock().now().to_msg(), frame_id='/map') 
        self.cloud_pub.publish(self.particle_cloud)

    # Helper function that publishes the current estimate of the 
    # robot pose, for rviz and debugging.  
    def publish_estimated_robot_pose(self):
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='/map') 
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


# the usual boilerplate to get python ROS running 
def main(args=None):
    rclpy.init(args=args)
    node = Locate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

