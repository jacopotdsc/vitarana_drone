#!/usr/bin/env python3

from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Twist
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
        self.drone_velocity = [0.0, 0.0, 0.0]

        self.car_location = [0.0, 0.0, 0.0]
        self.car_yaw = 0.0
        self.car_velocity = [ 0.0, 0.0 ]

        self.obstacles_location = []

        self.reached_desired_height = False
        self.reached_car = False

        h = 4.0
        self.hovering_location = [0.0, 0.0, h]
     
        ###### ----------- Drone message command ----------- ######
        self.rpyt_cmd = edrone_cmd()
        self.rpyt_cmd.rcRoll = 1500.0
        self.rpyt_cmd.rcPitch = 1500.0
        self.rpyt_cmd.rcYaw = 1500.0
        self.rpyt_cmd.rcThrottle = 1500.0

        ###### ----------- controller parameters ----------- ######
        self.Kp = [10, 10, 20]
        self.Ki = [0, 0, 0]
        self.Kd = [400, 400, 2000]

        self.Kar = 15 
        self.Kap = 15
        self.Kaz = 10

        self.Kox = 30
        self.Koy = 30
        self.Koz = 0
        self.obs_th = 4
        self.d_safe = 0.5 # radius of the obstacle
       
        self.derivate_error = [0.0, 0.0, 0.0]
        self.proportional_error = [0.0, 0.0, 0.0]
        self.prev_error_value = [0.0, 0.0, 0.0]
        self.integral_error = [0.0, 0.0, 0.0]

        self.prev_time = rospy.Time.now().to_sec()
        self.use_adpt = 1
        self.adpt_th = 3

        self.th_check = 0.5

        ###### ----------- ROS topics ----------- ######
        self.rpyt_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.stop_car_pub= rospy.Publisher('/simulation_state', Bool, queue_size=1)

        self.x_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
        self.y_error_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)

        self.ca_pub = rospy.Publisher('/drone_ca_control', PoseStamped, queue_size=1)
        self.base_pub = rospy.Publisher('/drone_base_control', PoseStamped, queue_size=1)
        self.drone_pose_pub = rospy.Publisher('/drone_pose_pub', PoseStamped, queue_size=1)

        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.ground_truth_callback)
        rospy.Subscriber("/my_robot/odom", Odometry, self.car_odom_callback )
        rospy.Subscriber("/my_robot/cmd_vel", Twist, self.car_vel_callback )
        rospy.Subscriber('/simulation_state', Bool, self.stop_drone)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.obstacles_callback)


    def car_odom_callback(self, msg):
        if self.reached_desired_height == False:
            return
        else:
            car_pos = msg.pose.pose.position
            self.car_location = [car_pos.x, car_pos.y, self.hovering_location[2]]

            car_quaternion = [0, 0, 0, 0]
            car_quaternion[0] = msg.pose.pose.orientation.x
            car_quaternion[1] = msg.pose.pose.orientation.y
            car_quaternion[2] = msg.pose.pose.orientation.z
            car_quaternion[3] = msg.pose.pose.orientation.w

            car_roll, car_pitch, car_yaw = tf.transformations.euler_from_quaternion(car_quaternion)
            self.car_yaw = car_yaw

    def car_vel_callback(self, msg):

        if self.reached_desired_height == False:
            self.car_velocity = [ 0.0, 0.0 ]
        else:
            self.car_velocity = [ msg.linear.x, msg.angular.z]
            
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

    def obstacles_callback(self, msg):
        cylinder_names = [name for name in msg.name if name.startswith("cylinder")]
        self.obstacles_location = []
        for name in cylinder_names:
            idx = msg.name.index(name)
            pos = msg.pose[idx].position
            self.obstacles_location.append([pos.x, pos.y])
    
    def ground_truth_callback(self, msg):
        idx = msg.name.index("edrone")
        pos = msg.pose[idx].position

        self.drone_location[0] = pos.x
        self.drone_location[1] = pos.y
        self.drone_location[2] = pos.z

        vel = msg.twist[idx].linear
        self.drone_velocity[0] = vel.x
        self.drone_velocity[1] = vel.y
        self.drone_velocity[2] = vel.z

        drone_pose_msg = PoseStamped()
        drone_pose_msg.header.stamp = rospy.Time.now()
        drone_pose_msg.pose.position.x = self.drone_location[0]
        drone_pose_msg.pose.position.y = self.drone_location[1]
        drone_pose_msg.pose.position.z = self.drone_location[2]

        self.drone_pose_pub.publish(drone_pose_msg)

    def check_reached_car(self):
        dx = self.drone_location[0] - self.car_location[0]
        dy = self.drone_location[1] - self.car_location[1]
        dz = self.drone_location[2] - self.car_location[2]

        vx = self.drone_velocity[0] - self.car_velocity[0]*np.cos(self.car_yaw)
        vy = self.drone_velocity[1] - self.car_velocity[0]*np.sin(self.car_yaw)
        vz = self.drone_velocity[2]


        #print(f"{dx:.2f}, {dy:.2f} ,{dz:.2f} / {vx:.2f}, {vy:.2f} ,{vz:.2f}")

        dist_x = dx <= 0.20
        dist_y = dy <= 0.20
        dist_z = dz <= 0.25

        vel_x = abs(vx) <= 0.15
        vel_y = abs(vy) <= 0.15
        vel_z  = abs(vz) <= 0.10

        if dist_x and dist_y and dist_z and vel_x and vel_y and vel_z:
            #self.reached_car = True
            #self.stop_car_pub.publish(Bool(data=True))
            return False
            return True
        else:
            return False
        
    def stop_drone(self, msg):
        if msg.data == True:
            self.rpyt_cmd.rcRoll = 1500
            self.rpyt_cmd.rcPitch = 1500
            self.rpyt_cmd.rcYaw = 1500
            self.rpyt_cmd.rcThrottle = 1000
            self.rpyt_pub.publish(self.rpyt_cmd)

    def compute_obstacle_avoidance(self):

        ca_x_control = 0
        ca_y_control = 0 
        ca_z_control = 0

        eps = 1e-6

        # computing distance car w.r.t drone
        px, py = self.drone_location[:2]
        cx, cy = self.car_location[:2]

        vx = cx - px       
        vy = cy - py   

        car_dist = (vx*vx + vy*vy) ** 0.5 + eps
        car_dir = [vx / car_dist, vy / car_dist]

        for o in self.obstacles_location:

            # computing distance drone w.r.t obstacle
            ox, oy = o[:2]
            dx = px - ox        
            dy = py - oy    
            d = (dx*dx + dy*dy) ** 0.5 + eps

            if d > self.obs_th:
                continue
            
            # smoothing for guarantee continuity 
            t = ( d - self.d_safe)/(self.obs_th - self.d_safe + eps)
            if t <= 0.0:
                w = 1.0
            elif t >= 1.0:
                w = 0.0
            else:
                w = 1.0 - (t*t*(3.0 - 2.0*t))

            # computing obstacle direction
            n_x = dx/d
            n_y = dy/d

            # repulsive force
            rep_mag = self.Kox * w * (1.0/max(d, self.d_safe)) # max used to avoid singularities
            rep_x = rep_mag * n_x
            rep_y = rep_mag * n_y

            # tangential force
            cross = n_x * car_dir[1] - n_y * car_dir[0]
            sign = 1.0 if cross >= 0.0 else -1.0    # choose tuning side

            t_x = -n_y * sign
            t_y =  n_x * sign
            tan_mag = self.Koy * w * (1.0/min(1.0, abs(cross)))
            tan_x = tan_mag * t_x
            tan_y = tan_mag * t_y

            ca_x_control += rep_x + tan_x
            ca_y_control += rep_y + tan_y

        ca_z_control = -ca_z_control*self.Koz

        ca_x_control = clamp(ca_x_control, -300, 300)
        ca_y_control = clamp(ca_y_control, -300, 300)
        ca_z_control = clamp(ca_z_control, -300, 300)

        ca_msg = PoseStamped()
        ca_msg.header.stamp = rospy.Time.now()
        ca_msg.pose.position.x = ca_x_control
        ca_msg.pose.position.y = ca_y_control
        self.ca_pub.publish(ca_msg)

        return ca_x_control, ca_y_control, ca_z_control
    

    def pid(self):
        ###### ----------- Computing error: pid ---------- ######
        for i in range(3):
            if self.reached_desired_height == True:
                self.proportional_error[i]  = self.car_location[i] - self.drone_location[i]
            else:
                self.proportional_error[i]  = self.hovering_location[i] - self.drone_location[i]
            self.integral_error[i]      = self.integral_error[i] + self.proportional_error[i]
            self.derivate_error[i]      = self.proportional_error[i] - self.prev_error_value[i]
            self.prev_error_value[i]    = self.proportional_error[i]

        ###### ----------- Computing adaptive pid ---------- ######
        car_linear_velocity = self.car_velocity[0] 
        car_angular_velocity = self.car_velocity[1]

        current_time = rospy.Time.now().to_sec() 
        dt = 0.25 #current_time - self.prev_time

        self.prev_time = current_time
        adpt_on = 1 if (self.proportional_error[0]**2 + self.proportional_error[1]**2 + self.proportional_error[2]**2) ** 0.5 <= self.adpt_th else 0

        velocity_x_control = np.cos(self.car_yaw + car_angular_velocity*dt )*car_linear_velocity*dt
        velocity_y_control = np.sin(self.car_yaw + car_angular_velocity*dt )*car_linear_velocity*dt

        adpt_x_control = self.use_adpt*adpt_on*( self.Kar*velocity_x_control + self.car_location[0]) 
        adpt_y_control = self.use_adpt*adpt_on*( self.Kap*velocity_y_control + self.car_location[1]) 
        adpt_z_control = self.use_adpt*adpt_on*( 0 )

        ###### ----------- Computing collision avoidance pid ---------- ######
        ca_x_control, ca_y_control , ca_z_control = self.compute_obstacle_avoidance()

        ###### ----------- Computing base pid ---------- ######
        base_x_control = self.Kp[0]*self.proportional_error[0] + self.Ki[0]*self.integral_error[0] + self.Kd[0]*self.derivate_error[0] 
        base_y_control = self.Kp[1]*self.proportional_error[1] + self.Ki[1]*self.integral_error[1] + self.Kd[1]*self.derivate_error[1]
        base_z_control = self.Kp[2]*self.proportional_error[2] + self.Ki[2]*self.integral_error[2] + self.Kd[2]*self.derivate_error[2] 

        ###### ----------- PID equation ---------- ######
        cartesian_x_control = clamp(base_x_control, -300, 300) #+ adpt_x_control + ca_x_control
        cartesian_y_control = clamp(base_y_control, -300, 300) #+ adpt_y_control + ca_y_control
        cartesian_z_control = clamp(base_z_control, -300, 300) #+ adpt_z_control + ca_z_control

        self.rpyt_cmd.rcRoll = 1500 + cartesian_x_control*np.cos(self.current_attitude[2]) + cartesian_y_control*np.sin(self.current_attitude[2])
        self.rpyt_cmd.rcPitch = 1500 + cartesian_x_control*np.sin(self.current_attitude[2]) - cartesian_y_control*np.cos(self.current_attitude[2])
        self.rpyt_cmd.rcThrottle = 1500 + cartesian_z_control

        ##### ------------ Clamping -------------- ######
        self.rpyt_cmd.rcRoll = clamp(self.rpyt_cmd.rcRoll, 1200, 1800) + ca_x_control
        self.rpyt_cmd.rcPitch = clamp(self.rpyt_cmd.rcPitch, 1200, 1800) + ca_y_control
        self.rpyt_cmd.rcThrottle = clamp(self.rpyt_cmd.rcThrottle, 1000, 2000)


        base_msg = PoseStamped()
        base_msg.header.stamp = rospy.Time.now()
        base_msg.pose.position.x = self.rpyt_cmd.rcRoll
        base_msg.pose.position.y = self.rpyt_cmd.rcPitch
        base_msg.pose.position.z = self.rpyt_cmd.rcThrottle
        self.base_pub.publish(base_msg)


        ###### ----------- Publishing messages --------- ######
        if self.reached_car == False:
            self.rpyt_pub.publish(self.rpyt_cmd)

        

def location_not_reached(actual, desired):
    error_on_x = abs(desired[0] - actual[0]) 
    error_on_y = abs(desired[1] - actual[1]) 
    error_on_z = abs(desired[2] - actual[2])
    th_error = 0.5

    x_bool = error_on_x > th_error 
    y_bool = error_on_y > th_error
    z_bool = error_on_z > th_error

    if ( (x_bool) or ( y_bool ) or ( z_bool ) ):
        retval = True
    else:
        retval = False

    return retval

def main():
    e_drone = Edrone()
    e_drone.use_adpt = 1 if rospy.get_param('~use_adaptive_controller', False) == True else 0

    e_drone.pid()
    rospy.loginfo("drone started from : " + str(e_drone.drone_location))    

    while not rospy.is_shutdown():
        
        while (location_not_reached(e_drone.drone_location, e_drone.hovering_location)):
            e_drone.pid()
            time.sleep(0.05)
        
        if e_drone.reached_desired_height == False:
            e_drone.reached_desired_height = True
            rospy.loginfo("Drone reached hovering state at: [%.2f, %.2f]" % (e_drone.drone_location[0], e_drone.drone_location[1]))

        while(e_drone.check_reached_car() == False):
            e_drone.pid()
            time.sleep(0.05)
        
        break


    rospy.loginfo("Drone reached car at: [%.2f, %.2f]" % (e_drone.drone_location[0], e_drone.drone_location[1]))
    rospy.loginfo("Finished")

if __name__ == '__main__':

    
    print(F"[CONTROLL] Waiting {TIME_BEFORE_START_TASK} seconds initialize and start task")
    t = time.time()
    while time.time() -t < TIME_BEFORE_INIT:
        pass

    main()
