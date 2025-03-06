#!/usr/bin/env python3


from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class AttitudeController():
    def __init__(self):
        rospy.init_node('attitude_controller') 

        ###### ----------- PD parameters: rpy----------- ######
        self.Kp = [60   , 60   , 2100]
        self.Ki = [0    , 0    , 0]
        self.Kd = [450, 450, 435]

        self.error_value = [0.0, 0.0, 0.0]
        self.change_in_error_value = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.sum_error_value = [0.0, 0.0, 0.0]

        ###### ----------- Drone attitude---------- ######
        self.edrone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]   # Read from IMU
        self.edrone_orientation_euler = [0.0, 0.0, 0.0]   # Conversion to orientation

        self.setpoint_cmd = [0.0, 0.0, 0.0] 
        self.set_throttle = 0.0

        self.setpoint_euler = [0.0, 0.0, 0.0]

        ###### ----------- Message declaration---------- ######
        self.pwm_cmd = prop_speed()

        self.roll_Error = Float32()
        self.roll_Error.data = 0.0
        self.pitch_Error = Float32()
        self.pitch_Error.data = 0.0
        self.yaw_Error = Float32()
        self.yaw_Error.data = 0.0

        ###### ----------- ROS topics ----------- ######
        self.cartesian_thrust_pub = rospy.Publisher('/edrone/cartesian_thrust', cartesian_thrust, queue_size=1) # Used to communication with firmware
        self.roll_error = rospy.Publisher('/roll_error', Float32, queue_size=1)     #   For plotting purporses
        self.pitch_error = rospy.Publisher('/pitch_error', Float32, queue_size=1)   #   For plotting purporses
        self.yaw_error = rospy.Publisher('/yaw_error', Float32, queue_size=1)       #   For plotting purporses

        rospy.Subscriber('/drone_command', edrone_cmd, self.edrone_command_callback)    # Take data from position controller
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)    # Take data about attitude state of the drone


    def imu_callback(self, msg):
        '''
        IMU data are given in quaternion format. 
        Applying a conversion to obtain euler angles 
        '''
        edrone_orientation_quaternion = [0, 0, 0, 0]
        edrone_orientation_quaternion[0] = msg.orientation.x
        edrone_orientation_quaternion[1] = msg.orientation.y
        edrone_orientation_quaternion[2] = msg.orientation.z
        edrone_orientation_quaternion[3] = msg.orientation.w

        euler_angles = tf.transformations.euler_from_quaternion(edrone_orientation_quaternion)
        
        self.edrone_orientation_euler[0] = euler_angles[1]
        self.edrone_orientation_euler[1] = euler_angles[0]  
        self.edrone_orientation_euler[2] = euler_angles[2]

    def edrone_command_callback(self, msg):
        '''
        Applying a conversion from RC ( Radio Control ) comand domain to PWN domain
            - RC values are in 1000-2000 where 1500 mean no input
            - PWM are value of power given to the motors
        '''
        self.setpoint_cmd[0] = msg.rcRoll  * 0.00315 - 4.725
        self.setpoint_cmd[1] = msg.rcPitch * 0.00315 - 4.725
        self.setpoint_cmd[2] = msg.rcYaw   * 0.00630 - 9.45
        self.set_throttle = msg.rcThrottle * 1.024  - 1024.0  
    

    def pid(self):

        # converting the current orientations from quaternion to euler angles 
        (self.edrone_orientation_euler[1], self.edrone_orientation_euler[0], self.edrone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.edrone_orientation_quaternion[0], self.edrone_orientation_quaternion[1], self.edrone_orientation_quaternion[2], self.edrone_orientation_quaternion[3]])
        
        # Convertng the range from 1000 to 2000 in the range of -1.575rad to 1.575rad for roll, pitch and in the range of -3.15rad to 3.15rad for yaw axis
        self.setpoint_euler[0] = self.setpoint_cmd[0]   # * 0.00315 - 4.725
        self.setpoint_euler[1] = self.setpoint_cmd[1]   # * 0.00315 - 4.725
        self.setpoint_euler[2] = self.setpoint_cmd[2]   # * 0.00630 - 9.45

        # converting the range from 1000 to 2000 in the range of 0 to 1024 for set_throttle value
        set_throttle_value = self.set_throttle*1.024  - 1024.0
        
        # updating all the error values to be used in PID equation
        for i in range(3):
            self.error_value[i] = self.setpoint_euler[i] - self.edrone_orientation_euler[i]
            self.sum_error_value[i] = self.sum_error_value[i] + self.error_value[i]
            self.change_in_error_value[i] = self.error_value[i] - self.prev_error_value[i]
            self.prev_error_value[i] = self.error_value[i]

        # assigning error values to its container to publish
        self.roll_Error.data = self.error_value[0]
        self.pitch_Error.data = self.error_value[1]
        self.yaw_Error.data = self.error_value[2]

        # PID eqation for roll
        output0 = self.Kp[0]*self.error_value[0] + self.Ki[0]*self.sum_error_value[0] + self.Kd[0]*self.change_in_error_value[0]
        
        # PID eqation for pitch
        output1 = self.Kp[1]*self.error_value[1] + self.Ki[1]*self.sum_error_value[1] + self.Kd[1]*self.change_in_error_value[1]
        
        # PID eqation for yaw
        output2 = self.Kp[2]*self.error_value[2] + self.Ki[2]*self.sum_error_value[2] + self.Kd[2]*self.change_in_error_value[2]

        cartesian_thrust_msg = cartesian_thrust()
        cartesian_thrust_msg.thrustRoll = output0
        cartesian_thrust_msg.thrustPitch = output1
        cartesian_thrust_msg.thrustYaw = output2
        cartesian_thrust_msg.Throttle = set_throttle_value
        self.cartesian_thrust_pub.publish(cartesian_thrust_msg)

        # publishing different error values
        self.roll_error.publish(self.roll_Error)
        self.pitch_error.publish(self.pitch_Error)
        self.yaw_error.publish(self.yaw_Error)


if __name__ == '__main__':

    # pause of 5 sec to open and load the gazibo
    t = time.time()
    while time.time() -t < 5:
        pass

    attitude_controller = AttitudeController()
    while not rospy.is_shutdown():
        attitude_controller.pid()
        time.sleep(0.05)
