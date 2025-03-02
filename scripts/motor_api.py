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

        # iperparameters
        self.max_pwm = 1024
        self.min_pwm = 0

        # Initializing pwm
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Publisher to give input to edrone in simulation 
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        
        # Subscriber topic
        rospy.Subscriber('/edrone/cartesian_thrust', cartesian_thrust, self.set_motor_pwm)
        
    def set_motor_pwm(self, msg):

        throttle_value = msg.Throttle
        output0 = msg.thrustRoll
        output1 = msg.thrustPitch
        output2 = msg.thrustYaw

        prop1 = throttle_value - output0 +output1 - output2
        prop2 = throttle_value - output0 -output1 + output2
        prop3 = throttle_value + output0 -output1 - output2
        prop4 = throttle_value + output0 +output1 + output2

        '''
        #print(f"--- msg: {throttle_value}, {output0}, {output1}, {output2}")
        #print(f"--- before_pwm: {prop1}, {prop2}, {prop3}, {prop4}")

        # Arm length (check if this is correct from your SDF file)
        l = 0.65

        # Precompute helper terms
        A = (output0 / l) + (output1 / l)
        B = (output1 / l) - (output0 / l)

        # Compute propeller forces using the closed-form solution
        prop1 = (throttle_value - A - output2 + B) / 4 + (output0 / l)
        prop2 = (throttle_value - A + output2 - B) / 4 + (output1 / l)
        prop3 = (throttle_value - A - output2 + B) / 4
        prop4 = (throttle_value - A + output2 - B) / 4


        print(f"--- after: {prop1}, {prop2}, {prop3}, {prop4} -> throttle: {throttle_value}\n")
        
        
        f1, f2, f3, f4 = symbols('f1 f2 f3 f4')

        eq1 = Eq(throttle_value*4,    f1 + f2 + f3 + f4)
        eq2 = Eq(output0,           0.65*(f2 - f4))
        eq3 = Eq(output1,           0.65*(f1 - f3))
        eq4 = Eq(output2,           (-f1 + f2 - f3 + f4) )


        system = [eq1, eq2, eq3, eq4] #+ constraints
        solution = nonlinsolve(system, [f1, f2, f3, f4])
        solution = {var: val for var, val in zip( (f1, f2, f3, f4), list(solution)[0])} 

        prop1 = float(solution.get(f1, 0)) 
        prop2 = float(solution.get(f2, 0))
        prop3 = float(solution.get(f3, 0))
        prop4 = float(solution.get(f4, 0))
        print(f"--- after: {prop1}, {prop2}, {prop3}, {prop4} -> throttle: {msg.Throttle}\n")
        '''

        # checking the boundary conditions for prop pwm values and updating the pwm_cmd container to publish
        self.pwm_cmd.prop1 = np.clip(prop1, 0, 1024)
        self.pwm_cmd.prop2 = np.clip(prop2, 0, 1024)
        self.pwm_cmd.prop3 = np.clip(prop3, 0, 1024)
        self.pwm_cmd.prop4 = np.clip(prop4, 0, 1024)


        # publishing pwm_cmd to /edrone/pwm
        self.pwm_pub.publish(self.pwm_cmd)

        
if __name__ == '__main__':

    # pause of 5 sec to open and load the gazibo
    t = time.time()
    while time.time() -t < 5:
        pass

    firmware = Firmware()
    rospy.spin()
