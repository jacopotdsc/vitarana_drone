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

        self.attitude_error = [0.0, 0.0, 0.0]   # Error in attitude
        self.omega_error    = [0.0, 0.0, 0.0]   # Error in rpy angular velocity 
        self.constant_error = [0.0, 0.0, 0.0]   # For constant errors
        self.prev_error_value = [0.0, 0.0, 0.0]

        ###### ----------- Drone attitude---------- ######
        #self.edrone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]   # Read from IMU
        #self.edrone_orientation_euler = [0.0, 0.0, 0.0]   # Conversion to orientation
        self.current_attitude = [0.0, 0.0, 0.0] # Taken from IMU

        self.desired_attitude = [0.0, 0.0, 0.0] 
        self.desired_throttle = 0.0

        #self.setpoint_euler = [0.0, 0.0, 0.0]

        ###### ----------- Message declaration---------- ######
        self.pwm_cmd = prop_speed()

        self.roll_Error = Float32()
        self.pitch_Error = Float32()
        self.yaw_Error = Float32()
        self.roll_Error.data = 0.0
        self.pitch_Error.data = 0.0
        self.yaw_Error.data = 0.0

        ###### ----------- ROS topics ----------- ######
        self.cartesian_thrust_pub = rospy.Publisher('/edrone/cartesian_thrust', cartesian_thrust, queue_size=1) # Used to communication with firmware
        self.roll_error = rospy.Publisher('/roll_error', Float32, queue_size=1)     #   For plotting purporses
        self.pitch_error = rospy.Publisher('/pitch_error', Float32, queue_size=1)   #   For plotting purporses
        self.yaw_error = rospy.Publisher('/yaw_error', Float32, queue_size=1)       #   For plotting purporses

        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)    # Take data from position controller
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)    # Take data about attitude state of the drone


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
        
        self.current_attitude[0] = euler_angles[1]
        self.current_attitude[1] = euler_angles[0]  
        self.current_attitude[2] = euler_angles[2]

    def drone_command_callback(self, msg):
        '''
        Applying a conversion from RC ( Radio Control ) comand domain to PWN domain
            - RC values are in 1000-2000 where 1500 mean no input
            - PWM are value of power given to the motors
        '''
        self.desired_attitude[0] = msg.rcRoll  * 0.00315 - 4.725
        self.desired_attitude[1] = msg.rcPitch * 0.00315 - 4.725
        self.desired_attitude[2] = msg.rcYaw   * 0.00630 - 9.45
        self.desired_throttle  = msg.rcThrottle * 1.024  - 1024.0  
    

    def pid(self):

        # converting the current orientations from quaternion to euler angles 
        #(self.edrone_orientation_euler[1], self.edrone_orientation_euler[0], self.edrone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.edrone_orientation_quaternion[0], self.edrone_orientation_quaternion[1], self.edrone_orientation_quaternion[2], self.edrone_orientation_quaternion[3]])
        
        # Converting the range from 1000 to 2000 in the range of -1.575rad to 1.575rad for roll, pitch and in the range of -3.15rad to 3.15rad for yaw axis
        #self.setpoint_euler[0] = self.desired_attitude[0]   # * 0.00315 - 4.725
        #self.setpoint_euler[1] = self.desired_attitude[1]   # * 0.00315 - 4.725
        #self.setpoint_euler[2] = self.desired_attitude[2]   # * 0.00630 - 9.45

        # converting the range from 1000 to 2000 in the range of 0 to 1024 for set_throttle value
        #set_throttle_value = self.desired_throttle*1.024  - 1024.0
        
        ###### ----------- Computing error: rpy---------- ######
        for i in range(3):
            #self.attitude_error[i] = self.setpoint_euler[i] - self.edrone_orientation_euler[i]
            self.attitude_error[i] = self.desired_attitude[i] - self.current_attitude[i]
            self.constant_error[i] = self.constant_error[i] + self.attitude_error[i]
            self.omega_error[i] = self.attitude_error[i] - self.prev_error_value[i]
            self.prev_error_value[i] = self.attitude_error[i]


        ###### ----------- PID equation: rpy---------- ######
        thrust_roll_controll    = self.Kp[0]*self.attitude_error[0] + self.Ki[0]*self.intergral_error[0] + self.Kd[0]*self.velocity_error[0]
        thrust_pitch_controll   = self.Kp[1]*self.attitude_error[1] + self.Ki[1]*self.intergral_error[1] + self.Kd[1]*self.velocity_error[1]
        thrust_yaw_controll     = self.Kp[2]*self.attitude_error[2] + self.Ki[2]*self.intergral_error[2] + self.Kd[2]*self.velocity_error[2]

        ###### ----------- Publishing messages --------- ######
        cartesian_thrust_msg = cartesian_thrust()
        cartesian_thrust_msg.thrustRoll     = thrust_roll_controll
        cartesian_thrust_msg.thrustPitch    = thrust_pitch_controll
        cartesian_thrust_msg.thrustYaw      = thrust_yaw_controll
        cartesian_thrust_msg.Throttle       = self.desired_throttle #set_throttle_value
        self.cartesian_thrust_pub.publish(cartesian_thrust_msg)

        ###### ----------- Errorr messages: plotting purpos --------- ######
        self.roll_Error.data    = self.attitude_error[0]
        self.pitch_Error.data   = self.attitude_error[1]
        self.yaw_Error.data     = self.attitude_error[2]
        
        self.roll_error.publish(self.roll_Error)
        self.pitch_error.publish(self.pitch_Error)
        self.yaw_error.publish(self.yaw_Error)


if __name__ == '__main__':

    # Time to open gazebo: waiting a little
    t = time.time()
    while time.time() -t < 5:
        pass

    attitude_controller = AttitudeController()
    while not rospy.is_shutdown():
        attitude_controller.pid()
        time.sleep(0.05)
