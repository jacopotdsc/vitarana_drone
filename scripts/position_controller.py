#!/usr/bin/env python3

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import numpy as np
import time
import tf

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

class Edrone():
    def __init__(self):
        rospy.init_node('position_controller')

        ###### ----------- Drone informations ----------- ######
        self.drone_location = [0.0, 0.0, 0.0]
        self.current_attitude = [0.0, 0.0, 0.0]
        self.setpoint_location = [19.0, 72.0, 3.0]

        ###### ----------- Drone message command ----------- ######
        self.rpyt_cmd = edrone_cmd()
        self.rpyt_cmd.rcRoll = 1500.0
        self.rpyt_cmd.rcPitch = 1500.0
        self.rpyt_cmd.rcYaw = 1500.0
        self.rpyt_cmd.rcThrottle = 1500.0

        ###### ----------- PD parameters ----------- ######
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

    def gps_callback(self, msg):
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
        
        self.rpyt_cmd.rcRoll = 1500 + cartesian_x_control*np.cos(self.current_attitude[2]) - cartesian_y_control*np.sin(self.current_attitude[2])
        self.rpyt_cmd.rcPitch = 1500 + cartesian_x_control*np.sin(self.current_attitude[2]) + cartesian_y_control*np.cos(self.current_attitude[2])
        self.rpyt_cmd.rcThrottle = 1500 + cartesian_z_control

        ##### ------------ Clamping -------------- ######
        self.rpyt_cmd.rcRoll = clamp(self.rpyt_cmd.rcRoll, 1200, 1800)
        self.rpyt_cmd.rcPitch = clamp(self.rpyt_cmd.rcPitch, 1200, 1800)
        self.rpyt_cmd.rcThrottle = clamp(self.rpyt_cmd.rcThrottle, 1000, 2000)

        ###### ----------- Publishing messages --------- ######
        self.rpyt_pub.publish(self.rpyt_cmd)


def main():
    e_drone.pid()
    rospy.loginfo("drone started from : " + str(e_drone.drone_location))

    # setting setpoint to first point
    e_drone.setpoint_location = [19.0, 72.0, 3]
    # loop until tolerences 
    while ((e_drone.drone_location[0] > 19.0+0.000004517 or e_drone.drone_location[0] < 19.0-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 3.0+0.2 or e_drone.drone_location[2] < 3.0-0.2)):
        e_drone.pid()
        time.sleep(0.05)
    # pause of 10 sec to stablize the drone at that position
    t = time.time()
    while time.time() -t < 10:
        e_drone.pid()
        time.sleep(0.05)

    rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))

    # setting setpoint to the second point
    e_drone.setpoint_location = [19.0000451704, 72.0, 3]
    #loop until tolerences 
    while ((e_drone.drone_location[0] > 19.00004517040+0.000004517 or e_drone.drone_location[0] < 19.00004517040-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 3.0+0.2 or e_drone.drone_location[2] < 3.0-0.2)):
        e_drone.pid()
        time.sleep(0.05)
    # pause of 10 sec to stablize the drone at that position
    t = time.time()
    while time.time() -t < 10:
        e_drone.pid()
        time.sleep(0.05)

    rospy.loginfo("drone reached point : "+ str(e_drone.drone_location))

    # setting setpoint to final point
    e_drone.setpoint_location = [19.0000451704, 72.0, 0.31]
    # loop until tolerences 
    while ((e_drone.drone_location[0] > 19.00004517040+0.000004517 or e_drone.drone_location[0] < 19.00004517040-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 0.31+0.2 or e_drone.drone_location[2] < 0.31-0.2)):
        e_drone.pid()
        time.sleep(0.05)
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

    t = time.time()
    while time.time() -t < 4:
        pass

    e_drone = Edrone()

    t = time.time()
    while time.time() -t < 1:
        pass

    while not rospy.is_shutdown():
        main()
        break
