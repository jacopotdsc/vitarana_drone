#!/usr/bin/env python3


from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class AttitudeController():
    def __init__(self, attitude_controller_params):
        rospy.init_node('attitude_controller') 

        # Drone orientation in quaternion format [x,y,z,w]
        self.edrone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # Drone orientation in rpy format [r,p,y]
        self.edrone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the edrone_command in the range from 1000 to 2000
        # [roll_setpoint, pitch_setpoint, yaw_setpoint], and the set_throttle value
        self.setpoint_cmd = [0.0, 0.0, 0.0]
        self.set_throttle = 0.0

        # The setpoint of orientation in euler angles at which we want to stabilize the edrone
        # [roll_setpoint, pitch_psetpoint, yaw_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values.
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Declaring error values to publish for visualization in plotjuggler
        self.roll_Error = Float32()
        self.roll_Error.data = 0.0
        self.pitch_Error = Float32()
        self.pitch_Error.data = 0.0
        self.yaw_Error = Float32()
        self.yaw_Error.data = 0.0

         ###### ----------- PD parameters: rpy----------- ######
        self.Kp = [60   , 60   , 2100]
        self.Ki = [0    , 0    , 0]
        self.Kd = [450, 450, 435]

        # Declaring variable to store different error values, to be used in PID equations.
        self.error_value = [0.0, 0.0, 0.0]
        self.change_in_error_value = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.sum_error_value = [0.0, 0.0, 0.0]
    
        # initializing Publisher for /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.cartesian_thrust_pub = rospy.Publisher('/edrone/cartesian_thrust', cartesian_thrust, queue_size=1)
        
        self.roll_error = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error = rospy.Publisher('/yaw_error', Float32, queue_size=1)

        # Subscribing to /edrone_command, imu/data
        rospy.Subscriber('/drone_command', edrone_cmd, self.edrone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)


    # Imu callback function. The function gets executed each time when imu publishes /edrone/imu/data
    def imu_callback(self, msg):
        self.edrone_orientation_quaternion[0] = msg.orientation.x
        self.edrone_orientation_quaternion[1] = msg.orientation.y
        self.edrone_orientation_quaternion[2] = msg.orientation.z
        self.edrone_orientation_quaternion[3] = msg.orientation.w

     
    # edrone cmd callback function. The function gets executed each time when edrone_cmd publishes /edrone_command
    def edrone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.set_throttle = msg.rcThrottle
    

    # this function is containing all the pid equation to control the attitude of the edrone
    def pid(self):
        # converting the current orientations from quaternion to euler angles 
        (self.edrone_orientation_euler[1], self.edrone_orientation_euler[0], self.edrone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.edrone_orientation_quaternion[0], self.edrone_orientation_quaternion[1], self.edrone_orientation_quaternion[2], self.edrone_orientation_quaternion[3]])
        
        # Convertng the range from 1000 to 2000 in the range of -1.575rad to 1.575rad for roll, pitch and in the range of -3.15rad to 3.15rad for yaw axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.00315 - 4.725
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.00315 - 4.725
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.00630 - 9.45

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

    attitude_controller_params = {
        'k_phi_p': 0.5,
        'k_phi_d': 0.7,
        'k_theta_p': 0.3,
        'k_theta_d': 0.5,
        'k_psi_p': 0.5,
        'k_psi_d': 0.3
    }

    attitude_controller = AttitudeController(attitude_controller_params)
    while not rospy.is_shutdown():
        attitude_controller.pid()
        time.sleep(0.05)
