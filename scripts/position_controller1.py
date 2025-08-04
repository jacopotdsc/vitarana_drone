#!/usr/bin/env python3


from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import numpy as np
import time
import tf


class Edrone():
    def __init__(self):
        rospy.init_node('position_controller1')
        self.drone_location = [0.0, 0.0, 0.0]
        self.setpoint_location = [19.0, 72.0, 3.0]

        # Drone orientation in quaternion format [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # Drone orientation in rpy format [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # rpyt_cmd => roll, pitch, yaw, throttle command
        self.rpyt_cmd = edrone_cmd()
        self.rpyt_cmd.rcRoll = 1500.0
        self.rpyt_cmd.rcPitch = 1500.0
        self.rpyt_cmd.rcYaw = 1500.0
        self.rpyt_cmd.rcThrottle = 1500.0

        # Declaring error values to publish for visualization in plotjuggler
        self.latitude_Error = Float32()
        self.latitude_Error.data = 0.0
        self.latitude_Up = Float32()
        self.latitude_Up.data = 0.000004517
        self.latitude_Low = Float32()
        self.latitude_Low.data = -0.000004517

        self.longitude_Error = Float32()
        self.longitude_Error.data = 0.0
        self.longitude_Up = Float32()
        self.longitude_Up.data = 0.0000047487
        self.longitude_Low = Float32()
        self.longitude_Low.data = -0.0000047487

        self.altitude_Error = Float32()
        self.altitude_Error.data = 0.0
        self.altitude_Up = Float32()
        self.altitude_Up.data = 0.2
        self.altitude_Low = Float32()
        self.altitude_Low.data = -0.2

        ###### ----------- PD parameters ----------- ######
        self.Kp = [1080000, 1140000, 48]
        self.Ki = [0, 0, 0]
        self.Kd = [57600000, 57900000, 3000]
       
        self.change_in_error_value = [0.0, 0.0, 0.0]
        self.error_value = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.sum_error_value = [0.0, 0.0, 0.0]

        self.max_values = [2000.0, 2000.0, 2000.0, 2000.0]
        self.min_values = [1000.0, 1000.0, 1000.0, 1000.0]

        ###### ----------- ROS topics ----------- ######
        self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.latitude_error = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.longitude_error = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        self.altitude_error = rospy.Publisher('/altitude_error', Float32, queue_size=1)

        self.latitude_up = rospy.Publisher('/latitude_up', Float32, queue_size=1)
        self.longitude_up = rospy.Publisher('/longitude_up', Float32, queue_size=1)
        self.altitude_up = rospy.Publisher('/altitude_up', Float32, queue_size=1)
        self.latitude_low = rospy.Publisher('/latitude_low', Float32, queue_size=1)
        self.longitude_low = rospy.Publisher('/longitude_low', Float32, queue_size=1)
        self.altitude_low = rospy.Publisher('/altitude_low', Float32, queue_size=1)

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
    


    def imu_callback(self, msg):
        '''
        IMU data are given in quaternion format. 
        Applying a conversion to obtain euler angles 
        '''
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])


    def gps_callback(self, msg):
        self.drone_location[0] = msg.latitude
        self.drone_location[1] = msg.longitude
        self.drone_location[2] = msg.altitude


    def pid(self):
        ###### ----------- Computing error ---------- ######
        for i in range(3):
            self.error_value[i] = self.setpoint_location[i] - self.drone_location[i]
            self.sum_error_value[i] = self.sum_error_value[i] + self.error_value[i]
            self.change_in_error_value[i] = self.error_value[i] - self.prev_error_value[i]
            self.prev_error_value[i] = self.error_value[i]
        self.latitude_Error.data = self.error_value[0]
        self.longitude_Error.data = self.error_value[1]
        self.altitude_Error.data = self.error_value[2]

        ###### ----------- PID equation ---------- ######
        output0 = self.Kp[0]*self.error_value[0] + self.Ki[0]*self.sum_error_value[0] + self.Kd[0]*self.change_in_error_value[0]
        output1 = self.Kp[1]*self.error_value[1] + self.Ki[1]*self.sum_error_value[1] + self.Kd[1]*self.change_in_error_value[1]
        output2 = self.Kp[2]*self.error_value[2] + self.Ki[2]*self.sum_error_value[2] + self.Kd[2]*self.change_in_error_value[2]
        
        self.rpyt_cmd.rcRoll = 1500 + output0*np.cos(self.drone_orientation_euler[2]) - output1*np.sin(self.drone_orientation_euler[2])
        self.rpyt_cmd.rcPitch = 1500 + output0*np.sin(self.drone_orientation_euler[2]) + output1*np.cos(self.drone_orientation_euler[2])
        self.rpyt_cmd.rcThrottle = 1500 + output2
        
        if(self.rpyt_cmd.rcRoll > 1800):
            self.rpyt_cmd.rcRoll = 1800
        elif(self.rpyt_cmd.rcRoll<1200):
            self.rpyt_cmd.rcRoll = 1200

        if(self.rpyt_cmd.rcPitch > 1800):
            self.rpyt_cmd.rcPitch = 1800
        elif(self.rpyt_cmd.rcPitch<1200):
            self.rpyt_cmd.rcPitch = 1200

        if(self.rpyt_cmd.rcThrottle > 2000):
            self.rpyt_cmd.rcThrottle = 2000
        elif(self.rpyt_cmd.rcThrottle<1000):
            self.rpyt_cmd.rcThrottle = 1000

        ###### ----------- Publishing messages --------- ######
        self.rpyt_pub.publish(self.rpyt_cmd)

        self.latitude_error.publish(self.latitude_Error)
        self.longitude_error.publish(self.longitude_Error)
        self.altitude_error.publish(self.altitude_Error)
        self.latitude_up.publish(self.latitude_Up)
        self.longitude_up.publish(self.longitude_Up)
        self.altitude_up.publish(self.altitude_Up)
        self.latitude_low.publish(self.latitude_Low)
        self.longitude_low.publish(self.longitude_Low)
        self.altitude_low.publish(self.altitude_Low)


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
