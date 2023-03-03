import rclpy, math, time 
from rclpy.node import Node 

from std_msgs.msg import Float32
from irobot_create_msgs.msg import WheelTicks 
from geometry_msgs.msg import Twist 
from rclpy.qos import qos_profile_sensor_data

class PID(Node): 
    def __init__(self): 
        super().__init__("PID") 
        # connect to topic that controls the wheels 
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) 

        # subscriber to read the wheel ticks 
        self.subscriber = self.create_subscription(WheelTicks, '/wheel_ticks', self.tick_callback, qos_profile_sensor_data) 

        # subscriber to drive a distance.  You should implement two more subscribers: one to turn an angle, and 
        # another to drive in a square three times.  
        # To publish a message to the drive topic from the command line, use 
        # ros2 topic pub --once /distance std_msgs/Float32 "data: 1.0"
        # Change the 1.0 to whatever distance you want the robot to drive.  Keep in mind 
        # that you will need the quotes. 

        # ros2 topic pub --once /distance std_msgs/Float32 "data: 1000.0"
        self.drive_sub = self.create_subscription(Float32, '/distance', self.drive, qos_profile_sensor_data)
        
        # ros2 topic pub --once /angle std_msgs/Float32 "data: 0.785"
        self.turn_sub = self.create_subscription(Float32, '/angle', self.turn, qos_profile_sensor_data)

        # ros2 topic pub --once /pid_values std_msgs/Float32 "kp: 1.0, ki: 1.0, kd: 1.0"
        self.pid_sub = self.create_subscription(Float32, '/pid_values', self.update_pid, qos_profile_sensor_data)

        self.sample_time = 0.05

        self.my_timer = self.create_timer(self.sample_time, self.pid)

        # current tick values received from /wheel_ticks topic
        self.left_ticks = 0 
        self.right_ticks = 0

        # first tick values received on start
        self.left_origin = 0
        self.right_origin = 0
        self.reset_origin = True
        
        # tick values converted into mm based on wheel diameter
        self.l_dist = 0
        self.r_dist = 0
        self.dist = 0 # mean of left and right distance

        # tick length = pi*d / # ticks in one revolution
        self.tick_length = (72*math.pi)/508.8
        # distance between the wheels
        self.wheel_baseline = 235

        self.goal_dist = 0
        self.goal_angle = 0

        self.error = []
        self.drive_forward = False # Rename
        self.drive_angle = False

        # self.move_queue = 

        # PID Values
        self.kp = 0.3
        self.ki = 0.01
        self.kd = 0.1

    # set class variables that hold the number of ticks from left and right.  You will also need to figure 
    # out what zero is.  So, at time t=0, store the current values as the starting point, and then 
    # subtract the starting point from the values.  
    def tick_callback(self, msg):
        if self.reset_origin:
            self.left_origin = msg.ticks_left
            self.right_origin = msg.ticks_right
            self.reset_origin = False

        self.left_ticks = msg.ticks_left
        self.right_ticks = msg.ticks_right

        self.l_dist = (self.left_ticks - self.left_origin) * self.tick_length
        self.r_dist = (self.right_ticks - self.right_origin) * self.tick_length
        self.dist = (self.l_dist + self.r_dist) * 0.5
        self.angle = (self.r_dist - self.l_dist)/(2*self.wheel_baseline)

        # print("tick_callback")
        # print(self.left_ticks, self.l_dist, self.right_ticks, self.r_dist)
 
    def update_pid(self, values):
        self.kp = values.kp
        self.ki = values.ki
        self.kd = values.kd

        print("New PID values: ", self.kp, self.ki, self.kd)

    # Implement the pid controller here.  use the tick values from tick_callback to compute the 
    # new velocity, and then publish the velocity on self.publisher 
    def pid(self): 

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        
        # if self.drive_forward and sum([abs(e) for e in self.error[-5:]]) > 10:
        if self.drive_forward and self.error[-1] > 0:
            print(sum([abs(e) for e in self.error[-5:]]))
            pid_out = self.kp*(self.error[-1] + self.ki*(sum(self.error[-10:])) + self.kd*(self.error[-1] - self.error[-2])) # figure out discrete pid equation
            pid_out *= 0.001 # error is around range of 1000, but max speed is 0.46
            cmd.linear.x = pid_out
            self.error.append(self.goal_dist - self.dist)
            print("Error:", self.error[-1], "Dist:", self.dist, "PID:", pid_out)

        # elif self.drive_angle and sum([abs(e) for e in self.error[-5:]]) > 0.05:
        elif self.drive_angle and self.error[-1] > 0:
            print(sum([abs(e) for e in self.error[-5:]]))
            pid_out = self.kp*(self.error[-1] + self.ki*(sum(self.error[-10:])) + self.kd*(self.error[-1] - self.error[-2])) # figure out discrete pid equation
            pid_out *= 1
            cmd.angular.z = pid_out
            self.error.append(self.goal_angle - self.angle)
            print("Error:", self.error[-1], "Angle:", self.angle, "PID:", pid_out)
        else:
            self.drive_forward = False
            self.drive_angle = False
            self.reset_origin = True
            self.error = []
            
        self.publisher.publish(cmd)

    # drive a given distance in meters.  Once the driven distance is close enough to the 
    # desired distance, stop the robot by publishing a Twist message of all zeros.  
    def drive(self, distance): 
        self.goal_dist = distance.data
        print("Driving", distance.data, "meters")
        self.error.append(self.goal_dist - self.dist)
        self.error.append(self.goal_dist - self.dist)

        print(self.error[0])
        self.drive_forward = True

    def turn(self, angle):
        self.goal_angle = angle.data
        print("Driving", angle.data, "meters")
        self.error.append(self.goal_angle - self.angle)
        self.error.append(self.goal_angle - self.angle)
        print(self.error[0])
        self.drive_angle = True


    def move_forward(self):
        # # print("moving forward")
        # drive = 0.1
        # if self.l_dist >= 1000.0 or self.r_dist >= 1000.0:
        #     drive = 0.0

        # cmd = Twist()
        # cmd.linear.x = drive
        # cmd.linear.y = 0.0
        # cmd.linear.z = 0.0
        # cmd.angular.x = 0.0
        # cmd.angular.y = 0.0
        # cmd.angular.z = 0.0
        # self.publisher.publish(cmd)
        pass


# main is basically bolerplate code.  You only need to change 
# the class you instantiate 
def main(args=None):
    # initialize all ROS Python machinery 
    rclpy.init(args=args)

    # instantiate your class 
    print("Instantiating PID Node...")
    node = PID()

    print("Entering infinite loop...")
    # this runs your class in an infinite loop until 
    # either your code crashes, or you press Ctlr-C 
    rclpy.spin(node)

    # gracefully clean up 
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
