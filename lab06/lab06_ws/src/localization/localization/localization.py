import rclpy, time, random
import numpy as np
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSReliabilityPolicy , QoSProfile

from lifecycle_msgs.srv import ChangeState 
from lifecycle_msgs.msg import Transition

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav2_msgs.msg import Particle, ParticleCloud
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler

from .likelihood import LikelihoodField


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
        #self.laser_sub = self.create_subscription(LaserScan, "/scan", self.locate, qos_profile_sensor_data) 
        qos = QoSProfile(depth=5)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.laser_pub = self.create_subscription(LaserScan, '/scan', self.locate, qos)
        
        self.cloud_pub = self.create_publisher(ParticleCloud, '/particle_cloud', 10) 

        self.command_sub = self.create_subscription(Twist, '/cmd_vel', self.get_command, qos_profile_sensor_data)
        self.command = Twist()

        # this is the set of particles.  Conviently, nav2_msgs has a Particle and 
        # ParticleCloud message already.  
        self.particle_cloud = ParticleCloud()  

        # the best estimate of the robot's pose.  Useful for 
        # debugging
        self.robot_estimate = Pose() 

        # the map received from the map server 
        self.map = OccupancyGrid() 

        # number of particles. Probably need to adjust this number 
        self.num_particles = 1000 

        # initialize the particle cloud 
        self.init_particle_cloud() 
    
        # now that we have our particles, we can proceed with the 
        # localization algorithm 
        self.initialized = True 
        self.get_logger().error("INITIALIZED")

    # this function should set the self.map variable, and also set other information 
    # about the map such as width, height, resolution, etc.  See the OccupancyGrid message 
    # documentation on-line.  
    def get_map(self, msg): 
        self.get_logger().error("GET MAP")
        self.map = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
        print(self.width, self.height, self.resolution, self.origin)

        self.has_map = True

        self.likelihood = LikelihoodField(self.map)

    # this function sets the self.command variable from the Twist message published 
    # on the /cmd_vel topic
    def get_command(self, msg):
        self.command.linear = msg.linear
        self.command.angular = msg.angular
        self.get_logger().error("Command: Linear = " + str(self.command.linear.x) + ", Angular = " + str(self.command.angular.z))

    # initialize the particles with random values.  You will need to determine the appropriate 
    # values for x and y that are consistent with the map.  Using the map resolution, and the map
    # origin will be required.  Also, to create a random Quaternion, you can pass four random 
    # numbers in [0, 1] to the constructor.  For initial weights, set them all to 0.5  
    def init_particle_cloud(self): 
        while not self.has_map:
            pass

        self.get_logger().error("Initializing Particle Cloud...")
        for i in range(self.num_particles):
            pose = Pose()
            pose.position = Point()
            pose.position.x = random.random()*self.width*self.resolution + self.origin.x
            pose.position.y = random.random()*self.height*self.resolution + self.origin.y
            pose.position.z = 0.0
            # self.get_logger().error("initialized position")

            pose.orientation = Quaternion()
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            # self.get_logger().error("initialized quaternion")

            p = Particle()
            p.pose = pose
            p.weight = 0.5
            # self.get_logger().error("initialized particle " + str(i))
          
            self.particle_cloud.particles.append(p)

        self.get_logger().error("Finished Initializing Particle Cloud")


    # heres where you'll implement the particle filter.  So, your localization algorithm will 
    # run everytime a laser scan comes in.  Use the self.initialized variable to ensure everything 
    # is ready prior to starting your algorithm.  You should also check if the robot has moved some 
    # minimum amount prior to running the particle filter.  The /odom topic is helpful here.  
    def locate(self, msg): 
        # implement the complete particle filter here 
        while not self.initialized:
            pass
        
        # if there is no command, don't do anything
        if self.command.linear.x == 0 and self.command.angular.z == 0:
            self.publish_particle_cloud()
            return

        for particle in self.particle_cloud.particles:
            particle.pose = self.sample_motion_model(self.command, particle.pose)
            particle.weight = self.measurement_model(msg, particle.pose, self.map)

        # select particle based off weights associated with each particle
        new_pc = ParticleCloud()
        r = np.random.uniform(self.num_particles)
        c = self.particle_cloud.particles[0].weight
        i = 0
        for m in range(len(self.particle_cloud.particles)):
            u = r + (m-1)/len(self.particle_cloud.particles)
            while u > c:
                i = i+1
                c = c + self.particle_cloud.particles[i].weight
            new_pc.particles.append(self.particle_cloud.particles[i])

        self.particle_cloud = new_pc

        self.get_logger().error("publishing particle cloud")
        self.publish_particle_cloud()
        # reset self.command variable
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0

    def sample_motion_model(self, command, pose):
        delta_t = 0.5
        alpha = 0.01
        a1 = alpha
        a2 = alpha
        a3 = alpha
        a4 = alpha
        a5 = alpha
        a6 = alpha

        v = command.linear.x + sample(a1*abs(command.linear.x) + a2*abs(command.angular.z))
        w = command.angular.z + sample(a3*command.linear.x + a4*abs(command.angular.z))
        g = sample(a5*command.linear.x + a6*abs(command.angular.z))

        quaternion_list = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
        
        x = pose.position.x - (v/w)*np.sin(yaw) + (v/w)*np.sin(yaw + w*delta_t)
        y = pose.position.y + (v/w)*np.cos(yaw) - (v/w)*np.cos(yaw + w*delta_t)
        theta = yaw + w*delta_t + g*delta_t

        new_pose = Pose()
        new_pose.position.x = x
        new_pose.position.y = y
        new_pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        new_pose.orientation = Quaternion()
        new_pose.orientation.x = q[0]
        new_pose.orientation.y = q[1]
        new_pose.orientation.z = q[2]
        new_pose.orientation.w = q[3]
        return new_pose

    def measurement_model(self, msg, pose, map):
        _, _, theta_pose = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        lasers = msg.ranges
        q = 1
        theta_sense = msg.angle_min
        for laser in lasers:
            if laser < msg.range_max:
                x_beam = pose.position.x + -0.04*np.cos(theta_pose) + laser*np.cos(theta_pose + theta_sense)
                y_beam = pose.position.y + -0.04*np.sin(theta_pose) + laser*np.sin(theta_pose + theta_sense)
                
                # convert x and y from pixel coordinantes to meters
                
                dist = self.likelihood.get_closest_obstacle_distance(2,2)
            


    # Helper function to publish the current set of particles, 
    # so rviz can visualize them 
    def publish_particle_cloud(self):
        # self.get_logger().error("Publishing Particle Cloud...")

        self.particle_cloud.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='/map') 
        self.cloud_pub.publish(self.particle_cloud)

    # Helper function that publishes the current estimate of the 
    # robot pose, for rviz and debugging.  
    def publish_estimated_robot_pose(self):
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='/map') 
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)

def sample(b):
    return np.sum(np.random.uniform(-b, b, (1, 12)))/2

# the usual boilerplate to get python ROS running 
def main(args=None):
    rclpy.init(args=args)
    node = Locate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

