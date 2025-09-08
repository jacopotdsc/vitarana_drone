#!/usr/bin/env python3

from vitarana_drone.msg import edrone_cmd
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates
import rospy
import numpy as np
import time
import tf
import csv
import math

TIME_BEFORE_INIT = 10.0
TIME_BEFORE_START_TASK = 2.0

# ----------------- helpers -----------------
def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

use_cartesian = True

# Yaw behaviour flags
ALIGN_TO_PATH = False  # set False to keep fixed yaw (FIXED_YAW)
FIXED_YAW = 0.0       # rad

class Edrone:
    def __init__(self):
        rospy.init_node('position_controller')

        ###### ----------- Drone informations ----------- ######
        self.drone_location = [0.0, 0.0, 0.0]
        self.current_attitude = [0.0, 0.0, 0.0]

        if use_cartesian:
            h = 3.0
            l = 3.0
            self.desired_location = [[0.0, 0.0, h], [l, 0.0, h], [l, l, h], [0.0, 0.0, h+1.0], [0.0, 0.0, 0.0]]
        else:
            self.desired_location = [[19.0, 72.0, 3.0], [19.0000451704, 72.0, 3.0] ,[19.0000451704, 72.0, 0.31]]

        ###### ----------- Drone message command ----------- ######
        self.rpyt_cmd = edrone_cmd()
        self.rpyt_cmd.rcRoll = 1500.0
        self.rpyt_cmd.rcPitch = 1500.0
        self.rpyt_cmd.rcYaw = 1500.0
        self.rpyt_cmd.rcThrottle = 1500.0

        ###### ----------- PD parameters ----------- ######
        if use_cartesian:
            self.setpoint_location = [0.0, 0.0, 3.0]
            self.Kp = [10, 10, 50]       # slightly softened
            self.Ki = [0, 0, 0]
            self.Kd = [400, 400, 2000]  # works with derivative as (e - e_prev)
        else:
            self.setpoint_location = [19.0, 72.0, 3.0]
            self.Kp = [1080000, 1140000, 48]
            self.Ki = [0, 0, 0]
            self.Kd = [57600000, 57900000, 3000]
       
        self.derivate_error = [0.0, 0.0, 0.0]
        self.proportional_error = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.integral_error = [0.0, 0.0, 0.0]

        # ---- Yaw PD (conservative) ----
        self.Kp_yaw = 220.0   # start small; rcYaw expects rate-like input
        self.Ki_yaw = 0.0
        self.Kd_yaw = 0.0     # keep 0 initially to avoid kick
        self.integral_error_yaw = 0.0
        self.prev_error_yaw = 0.0
        self.YAW_DEADBAND = 0.05  # rad (~3°)
        self.RC_YAW_SLEW = 40.0   # max µs change per control step

        # Output clamps (tight to avoid runaways)
        self.RP_LIMIT = 180.0   # µs delta
        self.THR_LIMIT = 250.0  # µs delta
        self.YAW_LIMIT = 200.0  # µs delta

        # ---- Z setpoint slew for gentle descent ----
        self.VZ_MAX_UP = 0.8
        self.VZ_MAX_DN = 0.35
        self.z_sp_filtered = self.setpoint_location[2]

        ###### ----------- ROS topics ----------- ######
        self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.x_error = rospy.Publisher('/x_error', Float32, queue_size=1)
        self.y_error = rospy.Publisher('/y_error', Float32, queue_size=1)
        self.z_error = rospy.Publisher('/z_error', Float32, queue_size=1)

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.ground_truth_callback)

        self._last_rc_yaw = 1500.0

    def imu_callback(self, msg):
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
        if use_cartesian:
            if "edrone" in msg.name:
                idx = msg.name.index("edrone")
                pos = msg.pose[idx].position
                self.drone_location[0] = pos.x
                self.drone_location[1] = pos.y
                self.drone_location[2] = pos.z

    def gps_callback(self, msg):
        if not use_cartesian:
            self.drone_location[0] = msg.latitude
            self.drone_location[1] = msg.longitude
            self.drone_location[2] = msg.altitude

    def pid(self):
        # ---- Smooth Z setpoint ----
        z_des = self.setpoint_location[2]
        dz = z_des - self.z_sp_filtered
        step_up = self.VZ_MAX_UP * 0.02
        step_dn = self.VZ_MAX_DN * 0.02
        if dz > 0:
            dz = clamp(dz, -step_dn, step_up)
        else:
            dz = clamp(dz, -step_dn, step_up)
        self.z_sp_filtered += dz

        ###### ----------- Computing error: pid ---------- ######
        for i in range(3):
            if i == 2:
                self.proportional_error[i] = self.z_sp_filtered - self.drone_location[i]
            else:
                self.proportional_error[i] = self.setpoint_location[i] - self.drone_location[i]
            self.integral_error[i]      = self.integral_error[i] + self.proportional_error[i]
            self.derivate_error[i]      = self.proportional_error[i] - self.prev_error_value[i]
            self.prev_error_value[i]    = self.proportional_error[i]

        ###### ----------- PID equation ---------- ######
        cartesian_x_control = self.Kp[0]*self.proportional_error[0] + self.Ki[0]*self.integral_error[0] + self.Kd[0]*self.derivate_error[0]
        cartesian_y_control = self.Kp[1]*self.proportional_error[1] + self.Ki[1]*self.integral_error[1] + self.Kd[1]*self.derivate_error[1]
        cartesian_z_control = self.Kp[2]*self.proportional_error[2] + self.Ki[2]*self.integral_error[2] + self.Kd[2]*self.derivate_error[2]
        
        # ------ Restore your original mapping (world → rc) ------
        yaw = self.current_attitude[2]
        self.rpyt_cmd.rcRoll = clamp(1500 + cartesian_x_control*math.cos(yaw) + cartesian_y_control*math.sin(yaw), 1500-self.RP_LIMIT, 1500+self.RP_LIMIT)
        self.rpyt_cmd.rcPitch = clamp(1500 + cartesian_x_control*math.sin(yaw) - cartesian_y_control*math.cos(yaw), 1500-self.RP_LIMIT, 1500+self.RP_LIMIT)
        self.rpyt_cmd.rcThrottle = clamp(1500 + cartesian_z_control, 1500-self.THR_LIMIT, 1500+self.THR_LIMIT)

        # ------ Yaw control (rate-like RC with deadband & slew) ------
        if ALIGN_TO_PATH:
            des_yaw = math.atan2(self.proportional_error[1], self.proportional_error[0]) if abs(self.proportional_error[0])+abs(self.proportional_error[1]) > 1e-4 else yaw
        else:
            des_yaw = FIXED_YAW
        yaw_error = wrap_to_pi(des_yaw - yaw)
        if abs(yaw_error) < self.YAW_DEADBAND:
            yaw_cmd = 0.0
        else:
            # Simple PD without /dt to match position PID style
            d_yaw = yaw_error - self.prev_error_yaw
            self.prev_error_yaw = yaw_error
            yaw_cmd = self.Kp_yaw*yaw_error + self.Kd_yaw*d_yaw
        # Convert to rc and apply slew + clamp
        target_rc_yaw = clamp(1500 + yaw_cmd, 1500-self.YAW_LIMIT, 1500+self.YAW_LIMIT)
        delta = clamp(target_rc_yaw - self._last_rc_yaw, -self.RC_YAW_SLEW, self.RC_YAW_SLEW)
        self.rpyt_cmd.rcYaw = self._last_rc_yaw + delta
        self._last_rc_yaw = self.rpyt_cmd.rcYaw

        ###### ----------- Publishing messages --------- ######
        self.rpyt_pub.publish(self.rpyt_cmd)
        self.x_error.publish(Float32(self.proportional_error[0]))
        self.y_error.publish(Float32(self.proportional_error[1]))
        self.z_error.publish(Float32(self.proportional_error[2]))

def location_not_reached(actual, desired):
    error_on_x = abs(desired[0] - actual[0]) 
    error_on_y = abs(desired[1] - actual[1]) 
    error_on_z = abs(desired[2] - actual[2])
    th_error = 0.1 
    return (error_on_x > th_error) or (error_on_y > th_error) or (error_on_z > th_error)


def main():
    e_drone = Edrone()
    rospy.loginfo("[CONTROLL] Waiting %s seconds initialize and start task", TIME_BEFORE_START_TASK)
    t = time.time()
    while time.time() -t < TIME_BEFORE_INIT:
        pass

    rospy.loginfo("drone started from : %s", str(e_drone.drone_location))

    rate = rospy.Rate(50)  # 50 Hz

    for i in range(len(e_drone.desired_location)):
        e_drone.setpoint_location = e_drone.desired_location[i]
        print(f"drone target: {e_drone.setpoint_location}")
        if use_cartesian == True:
            while (location_not_reached(e_drone.drone_location, e_drone.setpoint_location)):
                e_drone.pid()
                rate.sleep()
        else: 
            while ((e_drone.drone_location[0] > 19.0+0.000004517 or e_drone.drone_location[0] < 19.0-0.000004517) or (e_drone.drone_location[1] >  72.0+0.0000047487 or e_drone.drone_location[1] < 72.0-0.0000047487) or (e_drone.drone_location[2] > 3.0+0.2 or e_drone.drone_location[2] < 3.0-0.2)):
                e_drone.pid()
                rate.sleep()

        rospy.loginfo("drone reached point : %s", str(e_drone.drone_location))

        t = time.time()
        while time.time() -t < TIME_BEFORE_START_TASK:
            e_drone.pid()
            rate.sleep()

    # turning off
    e_drone.rpyt_cmd.rcRoll = 1500
    e_drone.rpyt_cmd.rcPitch = 1500
    e_drone.rpyt_cmd.rcYaw = 1500
    e_drone.rpyt_cmd.rcThrottle = 1000
    e_drone.rpyt_pub.publish(e_drone.rpyt_cmd)

    rospy.loginfo("drone reached point : %s", str(e_drone.drone_location))
    rospy.loginfo("destination reached!!!")

if __name__ == '__main__':
    main()