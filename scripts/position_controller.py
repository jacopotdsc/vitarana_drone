#!/usr/bin/env python3

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates
import rospy
import numpy as np
import time
import tf
import csv

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

use_cartesian = True

class Edrone():
    def __init__(self):
        rospy.init_node('position_controller')

        ###### ----------- Drone informations ----------- ######
        self.drone_location = [0.0, 0.0, 0.0]
        self.current_attitude = [0.0, 0.0, 0.0]
        if use_cartesian == True:
            self.desired_location = [[5.0, 5.0, 3.0 ], [5.0, 5.0, 3.0], [5.0, 5.0, 0.0]]
        else:
            self.desired_location = [[19.0, 72.0, 3.0], [19.0000451704, 72.0, 3.0] ,[19.0000451704, 72.0, 0.31]]

        ###### ----------- Drone message command ----------- ######
        self.rpyt_cmd = edrone_cmd()
        self.rpyt_cmd.rcRoll = 1500.0
        self.rpyt_cmd.rcPitch = 1500.0
        self.rpyt_cmd.rcYaw = 1500.0
        self.rpyt_cmd.rcThrottle = 1500.0

        ###### ----------- PD parameters ----------- ######
        if use_cartesian == True:
            self.setpoint_location = [0.0, 0.0, 3.0]
            self.Kp = [10, 10, 50]
            self.Ki = [0, 0, 0]
            self.Kd = [400, 400, 2000]
        else:
            self.setpoint_location = [19.0, 72.0, 3.0]
            self.Kp = [1080000, 1140000, 48]
            self.Ki = [0, 0, 0]
            self.Kd = [57600000, 57900000, 3000]
       
        self.derivate_error = [0.0, 0.0, 0.0]
        self.proportional_error = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.integral_error = [0.0, 0.0, 0.0]

        ###### ----------- ROS topics ----------- ######
        self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.x_error = rospy.Publisher('/x_error', Float32, queue_size=1)
        self.y_error = rospy.Publisher('/y_error', Float32, queue_size=1)
        self.z_error = rospy.Publisher('/z_error', Float32, queue_size=1)

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.ground_truth_callback)

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
        if use_cartesian == True:
            idx = msg.name.index("edrone")
            pos = msg.pose[idx].position

            self.drone_location[0] = pos.x
            self.drone_location[1] = pos.y
            self.drone_location[2] = pos.z

    def gps_callback(self, msg):

        if use_cartesian == False:
            self.drone_location[0] = msg.latitude
            self.drone_location[1] = msg.longitude
            self.drone_location[2] = msg.altitude


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
        
        #self.rpyt_cmd.rcRoll = 1500 + cartesian_y_control
        #self.rpyt_cmd.rcPitch = 1500 + cartesian_x_control

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

    #print(f"{error_on_x}: {x_bool}, {error_on_y}: {y_bool}, {error_on_z}: {z_bool}")
    if ( (x_bool) or ( y_bool ) or ( z_bool ) ):
        retval = True
    else:
        retval = False

    #print(f"retval: {retval} -> {x_bool}, {y_bool}, {z_bool}")
    return retval

def main():
    e_drone.pid()
    rospy.loginfo("drone started from : " + str(e_drone.drone_location))

    for i in range(len(e_drone.desired_location)):

        e_drone.setpoint_location = e_drone.desired_location[i]
        print(f"drone target: {e_drone.setpoint_location}")
        if use_cartesian == True:
            while (location_not_reached(e_drone.drone_location, e_drone.setpoint_location)):
                e_drone.pid()
                time.sleep(0.05)
        else: 
            while ((e_drone.drone_location[0] > 19.0+0.000004517 or e_drone.drone_location[0] < 19.0-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 3.0+0.2 or e_drone.drone_location[2] < 3.0-0.2)):
                e_drone.pid()
                time.sleep(0.05)

        rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))

        # pause of 10 sec to stablize the drone at that position
        t = time.time()
        while time.time() -t < 10:
            e_drone.pid()
            time.sleep(0.05)

    # turning off
    e_drone.rpyt_cmd.rcRoll = 1500
    e_drone.rpyt_cmd.rcPitch = 1500
    e_drone.rpyt_cmd.rcYaw = 1500
    e_drone.rpyt_cmd.rcThrottle = 1000
    e_drone.rpyt_pub.publish(e_drone.rpyt_cmd)

    rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))
    rospy.loginfo("destination reached!!!")

if __name__ == '__main__':

    print("[CONTROLL] Waiting 4 seconds before initialization")
    t = time.time()
    while time.time() -t < 4:
        pass

    e_drone = Edrone()

    print("[CONTROLL] Waiting 5 seconds before start")
    t = time.time()
    while time.time() -t < 5:
        pass

    while not rospy.is_shutdown():
        with open("/home/vboxuser/Desktop/catkin_ws/src/vitarana_drone/scripts/drone_positions.csv", "w", newline="") as f:
            main()
            break
