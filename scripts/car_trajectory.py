#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf
import math


class Car:
    def __init__(self, v, omega):
        self.car_location = [0.0, 0.0, 0.0]
        self.car_yaw = 0.0

        self.v = v
        self.omega = omega

        self.pub = rospy.Publisher('/my_robot/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.stop_node = False

        rospy.Subscriber('/simulation_state', Bool, lambda msg: self.stop_car(msg, self.pub))
        rospy.Subscriber("/my_robot/odom", Odometry, self.car_odom_callback )

    def car_odom_callback(self, msg):
        car_pos = msg.pose.pose.position
        self.car_location = [car_pos.x, car_pos.y, 0.0]

        car_quaternion = [0, 0, 0, 0]
        car_quaternion[0] = msg.pose.pose.orientation.x
        car_quaternion[1] = msg.pose.pose.orientation.y
        car_quaternion[2] = msg.pose.pose.orientation.z
        car_quaternion[3] = msg.pose.pose.orientation.w

        car_roll, car_pitch, car_yaw = tf.transformations.euler_from_quaternion(car_quaternion)
        self.car_yaw = car_yaw

    def publish_circle(self):
        twist = Twist()
        while not rospy.is_shutdown() and self.stop_node:
            twist.linear.x = self.v
            twist.angular.z = self.omega
            self.pub.publish(twist)
            self.rate.sleep()

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.pub.publish(twist)

    def publish_figure8(self):
        twist = Twist()
        t_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown() and self.stop_node == False:
            t = rospy.Time.now().to_sec() - t_start
            twist.linear.x = self.v
            twist.angular.z = self.omega * math.cos(t)
            self.pub.publish(twist)
            self.rate.sleep()

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.pub.publish(twist)

    def publish_square(self):
        twist = Twist()
        side_time = 4.0  # s, fisso
        turn_time = 2.0  # s, fisso

        t_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown() and self.stop_node == False:
            t = rospy.Time.now().to_sec() - t_start
            cycle_time = side_time + turn_time
            phase = t % (cycle_time * 4)
            phase_in_cycle = phase % cycle_time

            if phase_in_cycle < side_time:
                twist.linear.x = self.v
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = math.pi / 2 / 2.0  

            self.pub.publish(twist)
            self.rate.sleep()
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.pub.publish(twist)

    def publish_straight(self):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = 0.0

        while not rospy.is_shutdown() and self.stop_node == False:
            self.pub.publish(twist)
            self.rate.sleep()

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.pub.publish(twist)

    def publish_custom(self):
        car_setpoint = [ [4.0, -1.0], [6.0, -3.0], [3.0, -5.0], [0.0, 0.0]  ]

        i = 0
        while not rospy.is_shutdown() and self.stop_node == False:
            
            point = car_setpoint[i % len(car_setpoint)] 
            
            dx = point[0] - self.car_location[0]
            dy = point[1] - self.car_location[1]
            dist = (dx**2 + dy**2)**0.5

            local_dx = dx*math.cos(self.car_yaw) + dy*math.sin(self.car_yaw)
            local_dy = dx*math.sin(self.car_yaw) - dy*math.cos(self.car_yaw)

            theta = math.atan2(local_dy, local_dx)
            
            if abs(theta) < 0.001:
                omega = 0
            else:
                omega = -1 if theta > 0 else 1

            v = min(max(0, local_dx ), self.v) * math.cos(theta)

            if dist < 0.1:
                i += 1
                continue

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega
            self.pub.publish(twist)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.pub.publish(twist)

    def car_odom_callback(self, msg):
        car_pos = msg.pose.pose.position
        self.car_location = [car_pos.x, car_pos.y, 0.0]

        car_quaternion = [0, 0, 0, 0]
        car_quaternion[0] = msg.pose.pose.orientation.x
        car_quaternion[1] = msg.pose.pose.orientation.y
        car_quaternion[2] = msg.pose.pose.orientation.z
        car_quaternion[3] = msg.pose.pose.orientation.w

        car_roll, car_pitch, car_yaw = tf.transformations.euler_from_quaternion(car_quaternion)
        self.car_yaw = car_yaw

    def stop_car(self, msg, pub):
        if msg.data == True:
            self.stop_node = True

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            pub.publish(twist)

            rospy.loginfo("Drone reached car. Stopping car")
            rospy.signal_shutdown("[CT] Drone reached car. Stopping node")


def main():
    rospy.init_node('car_trajectory', anonymous=True)

    traj = rospy.get_param('~trajectory', 'circle')
    v = rospy.get_param('~linear_velocity', 0.5)
    omega = rospy.get_param('~angular_velocity', 0.6)

    car = Car(v, omega)

    if traj == 'circle':
        car.publish_circle()
    elif traj == 'figure8':
        car.publish_figure8()
    elif traj == 'square':
        car.publish_square()
    elif traj == 'straight':
        car.publish_straight()
    elif traj == 'custom':
        car.publish_custom()
    else:
        rospy.logerr("Traiettoria non riconosciuta: %s", traj)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
