#!/usr/bin/env python3

from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from gazebo_msgs.msg import ModelStates
import rospy
import numpy as np
import time
import tf
import csv

TIME_BEFORE_INIT = 10
TIME_BEFORE_START_TASK = 2

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

class Edrone():
    def __init__(self):
        rospy.init_node('position_controller')

        ###### ----------- Drone informations ----------- ######
        self.drone_location = [0.0, 0.0, 0.0]
        self.current_attitude = [0.0, 0.0, 0.0]

        self.car_location = []

        self.reached_desired_height = False
        self.reached_car = False
        h = 2.0
        l = 3.0
        #self.desired_location = [[l, 0.0, h ], [l, l, h], [-l, l, h+0.5], [-l, -l, h], [0.0, 0.0, h+3], [0.0, 0.0, 0.0] ]
        self.desired_location = [0.0, 0.0, h]
        self.car_location = []
     
        ###### ----------- Drone message command ----------- ######
        self.rpyt_cmd = edrone_cmd()
        self.rpyt_cmd.rcRoll = 1500.0
        self.rpyt_cmd.rcPitch = 1500.0
        self.rpyt_cmd.rcYaw = 1500.0
        self.rpyt_cmd.rcThrottle = 1500.0

        ###### ----------- PD parameters ----------- ######
        self.setpoint_location = [0.0, 0.0, 3.0]
        self.Kp = [10, 10, 50]
        self.Ki = [0, 0, 0]
        self.Kd = [400, 400, 2000]
       
        self.derivate_error = [0.0, 0.0, 0.0]
        self.proportional_error = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.integral_error = [0.0, 0.0, 0.0]

        ###### ----------- ROS topics ----------- ######
        self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.stop_car_pub= rospy.Publisher('/simulation_state', Bool, queue_size=1)

        self.x_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
        self.y_error_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)

        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.ground_truth_callback)
        rospy.Subscriber("/my_robot/odom", Odometry, self.car_odom_callback )
        rospy.Subscriber('/simulation_state', Bool, self.stop_drone)

    def car_odom_callback(self, msg):

        if self.reached_desired_height == False:
            return

        car_pos = msg.pose.pose.position
        self.car_location = [car_pos.x, car_pos.y, car_pos.z]
        self.desired_location = self.car_location

        #print(self.desired_location)
            
    def imu_callback(self, msg):
        '''
        IMU data are given in quaternion format. 
        Applying a conversion to obtain euler angles 
        '''
        imu_quaternion = [0, 0, 0, 0]
        imu_quaternion[0] = msg.orientation.x
        imu_quaternion[1] = msg.orientation.y
        imu_quaternion[2] = msg.orientation.z
        imu_quaternion[3] = msg.orientation.w
        
        euler_angles = tf.transformations.euler_from_quaternion(imu_quaternion)
        
        self.current_attitude[0] = euler_angles[0]
        self.current_attitude[1] = euler_angles[1]
        self.current_attitude[2] = euler_angles[2]

    def ground_truth_callback(self, msg):
        idx = msg.name.index("edrone")
        pos = msg.pose[idx].position

        self.drone_location[0] = pos.x
        self.drone_location[1] = pos.y
        self.drone_location[2] = pos.z

    def check_reached_car(self):

        error_on_x = abs(self.drone_location[0] - self.car_location[0]) 
        error_on_y = abs(self.drone_location[1] - self.car_location[1]) 
        error_on_z = abs(self.drone_location[2] - self.car_location[2])
        th_error = 0.1 

        if( (error_on_x < th_error) and (error_on_y < th_error) and (error_on_z < th_error)):
            #print("REACHED CAR STOPPING DRONE")
            self.reached_car = True
            self.check_reached_car()
            self.stop_car_pub.publish(Bool(data=True))
        
    def stop_drone(self, msg):
        if msg.data == True:

            e_drone.rpyt_cmd.rcRoll = 1500
            e_drone.rpyt_cmd.rcPitch = 1500
            e_drone.rpyt_cmd.rcYaw = 1500
            e_drone.rpyt_cmd.rcThrottle = 1000
            e_drone.rpyt_pub.publish(e_drone.rpyt_cmd)

    def pid(self):
        ###### ----------- Computing error: pid ---------- ######
        for i in range(3):
            self.proportional_error[i]  = self.setpoint_location[i] - self.drone_location[i]
            self.integral_error[i]      = self.integral_error[i] + self.proportional_error[i]
            self.derivate_error[i]      = self.proportional_error[i] - self.prev_error_value[i]
            self.prev_error_value[i]    = self.proportional_error[i]

        ###### ----------- PID equation ---------- ######
        cartesian_x_control = self.Kp[0]*self.proportional_error[0] + self.Ki[0]*self.integral_error[0] + self.Kd[0]*self.derivate_error[0]
        cartesian_y_control = self.Kp[1]*self.proportional_error[1] + self.Ki[1]*self.integral_error[1] + self.Kd[1]*self.derivate_error[1]
        cartesian_z_control = self.Kp[2]*self.proportional_error[2] + self.Ki[2]*self.integral_error[2] + self.Kd[2]*self.derivate_error[2]
        
        self.rpyt_cmd.rcRoll = 1500 + cartesian_x_control*np.cos(self.current_attitude[2]) + cartesian_y_control*np.sin(self.current_attitude[2])
        self.rpyt_cmd.rcPitch = 1500 + cartesian_x_control*np.sin(self.current_attitude[2]) - cartesian_y_control*np.cos(self.current_attitude[2])
        self.rpyt_cmd.rcThrottle = 1500 + cartesian_z_control

        ##### ------------ Clamping -------------- ######
        self.rpyt_cmd.rcRoll = clamp(self.rpyt_cmd.rcRoll, 1200, 1800)
        self.rpyt_cmd.rcPitch = clamp(self.rpyt_cmd.rcPitch, 1200, 1800)
        self.rpyt_cmd.rcThrottle = clamp(self.rpyt_cmd.rcThrottle, 1000, 2000)

        writer = csv.writer(f)
        writer.writerow([str(self.rpyt_cmd.rcRoll), str(self.rpyt_cmd.rcPitch), str(self.rpyt_cmd.rcThrottle), str(cartesian_x_control), str(cartesian_y_control), str(cartesian_z_control)])

        ###### ----------- Publishing messages --------- ######
        self.rpyt_pub.publish(self.rpyt_cmd)

def location_not_reached(actual, desired):
    error_on_x = abs(desired[0] - actual[0]) 
    error_on_y = abs(desired[1] - actual[1]) 
    error_on_z = abs(desired[2] - actual[2])
    th_error = 0.1 

    x_bool = error_on_x > th_error 
    y_bool = error_on_y > th_error
    z_bool = error_on_z > th_error

    if ( (x_bool) or ( y_bool ) or ( z_bool ) ):
        retval = True
    else:
        retval = False

    return retval

def main():
    e_drone.pid()
    rospy.loginfo("drone started from : " + str(e_drone.drone_location))

    #for i in range(len(e_drone.desired_location)):
    while not rospy.is_shutdown():
        
        e_drone.setpoint_location = e_drone.desired_location #[i]
        print(f"drone target: {e_drone.setpoint_location}")
        
        while (location_not_reached(e_drone.drone_location, e_drone.setpoint_location)):
            e_drone.pid()
            time.sleep(0.05)
        
        e_drone.reached_desired_height = True
        rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))

        # pause of 10 sec to stablize the drone at that position
        t = time.time()
        while time.time() -t < TIME_BEFORE_START_TASK:
            e_drone.pid()
            time.sleep(0.05)

    rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))
    rospy.loginfo("destination reached!!!")

if __name__ == '__main__':

    
    print(F"[CONTROLL] Waiting {TIME_BEFORE_START_TASK} seconds initialize and start task")
    t = time.time()
    while time.time() -t < TIME_BEFORE_INIT:
        pass

    e_drone = Edrone()

    while not rospy.is_shutdown():
        with open("/home/vboxuser/Desktop/catkin_ws/src/vitarana_drone/scripts/drone_positions.csv", "w", newline="") as f:
            main()
            break
