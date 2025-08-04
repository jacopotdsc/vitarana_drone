#!/usr/bin/env python3

from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf
import numpy as np
from sympy import symbols
from sympy import Eq
from sympy import nonlinsolve

class Firmware():
    def __init__(self):
        rospy.init_node('motor_api')  

        ###### ----------- PWMù: parameters ---------- ######
        self.max_pwm = 1024
        self.min_pwm = 0

        ###### ----------- PWM: initialization ---------- ######
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        ###### ----------- ROS topics ----------- ######
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
    
        rospy.Subscriber('/edrone/cartesian_thrust', cartesian_thrust, self.set_motor_pwm)
        
    def set_motor_pwm(self, msg):

        throttle_value = msg.Throttle
        roll_cmd = msg.thrustRoll
        pitch_cmd = msg.thrustPitch
        yam_cmd = msg.thrustYaw

        motor1 = throttle_value - roll_cmd + pitch_cmd - yam_cmd
        motor2 = throttle_value - roll_cmd - pitch_cmd + yam_cmd
        motor3 = throttle_value + roll_cmd - pitch_cmd - yam_cmd
        motor4 = throttle_value + roll_cmd + pitch_cmd + yam_cmd

        '''
        #print(f"--- msg: {throttle_value}, {roll_cmd}, {pitch_cmd}, {yam_cmd}")
        #print(f"--- before_pwm: {motor1}, {motor2}, {motor3}, {motor4}")

        # Arm length (check if this is correct from your SDF file)
        l = 0.65

        # Precompute helper terms
        A = (roll_cmd / l) + (pitch_cmd / l)
        B = (pitch_cmd / l) - (roll_cmd / l)

        # Compute propeller forces using the closed-form solution
        motor1 = (throttle_value - A - yam_cmd + B) / 4 + (roll_cmd / l)
        motor2 = (throttle_value - A + yam_cmd - B) / 4 + (pitch_cmd / l)
        motor3 = (throttle_value - A - yam_cmd + B) / 4
        motor4 = (throttle_value - A + yam_cmd - B) / 4


        print(f"--- after: {motor1}, {motor2}, {motor3}, {motor4} -> throttle: {throttle_value}\n")
        
        
        f1, f2, f3, f4 = symbols('f1 f2 f3 f4')

        eq1 = Eq(throttle_value*4,    f1 + f2 + f3 + f4)
        eq2 = Eq(roll_cmd,           0.65*(f2 - f4))
        eq3 = Eq(pitch_cmd,           0.65*(f1 - f3))
        eq4 = Eq(yam_cmd,           (-f1 + f2 - f3 + f4) )


        system = [eq1, eq2, eq3, eq4] #+ constraints
        solution = nonlinsolve(system, [f1, f2, f3, f4])
        solution = {var: val for var, val in zip( (f1, f2, f3, f4), list(solution)[0])} 

        motor1 = float(solution.get(f1, 0)) 
        motor2 = float(solution.get(f2, 0))
        motor3 = float(solution.get(f3, 0))
        motor4 = float(solution.get(f4, 0))
        print(f"--- after: {motor1}, {motor2}, {motor3}, {motor4} -> throttle: {msg.Throttle}\n")
        '''

        ###### ----------- PWM: clip to values and actuating motors ----------- ######
        self.pwm_cmd.prop1 = np.clip(motor1, a_min=self.min_pwm, a_max=self.max_pwm)
        self.pwm_cmd.prop2 = np.clip(motor2, a_min=self.min_pwm, a_max=self.max_pwm)
        self.pwm_cmd.prop3 = np.clip(motor3, a_min=self.min_pwm, a_max=self.max_pwm)
        self.pwm_cmd.prop4 = np.clip(motor4, a_min=self.min_pwm, a_max=self.max_pwm)

        self.pwm_pub.publish(self.pwm_cmd)

        
if __name__ == '__main__':

    # pause of 5 sec to open and load the gazibo
    t = time.time()
    while time.time() -t < 5:
        pass

    firmware = Firmware()
    rospy.spin()
