#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

def publish_circle(pub, rate, v, omega):
    twist = Twist()
    while not rospy.is_shutdown():
        twist.linear.x = v
        twist.angular.z = omega
        pub.publish(twist)
        rate.sleep()

def publish_figure8(pub, rate, v, omega_max):
    twist = Twist()
    t_start = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t_start
        twist.linear.x = v
        twist.angular.z = omega_max * math.cos(t)
        pub.publish(twist)
        rate.sleep()

def publish_square(pub, rate, v, omega_90):
    twist = Twist()
    side_time = 4.0  # s, fisso
    turn_time = 2.0  # s, fisso

    t_start = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t_start
        cycle_time = side_time + turn_time
        phase = t % (cycle_time * 4)
        phase_in_cycle = phase % cycle_time

        if phase_in_cycle < side_time:
            twist.linear.x = v
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = omega_90 

        pub.publish(twist)
        rate.sleep()

def publish_straight(pub, rate, v):
    twist = Twist()
    twist.linear.x = v
    twist.angular.z = 0.0

    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

def stop_car(msg, pub):
    if msg.data == True:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        rospy.loginfo("Drone reached car. Stopping car")
        rospy.signal_shutdown("[CT] Drone reached car. Stopping node")


def main():
    rospy.init_node('car_trajectory', anonymous=True)
    pub = rospy.Publisher('/my_robot/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    rospy.Subscriber('/simulation_state', Bool, lambda msg: stop_car(msg, pub))


    traj = rospy.get_param('~trajectory', 'circle')
    v = rospy.get_param('~linear_velocity', 0.5)
    omega = rospy.get_param('~angular_velocity', 0.6)
    omega_90 = math.pi / 2 / 2.0  # 90 gradi in 2 secondi (fisso per quadrato)

    if traj == 'circle':
        publish_circle(pub, rate, v, omega)
    elif traj == 'figure8':
        publish_figure8(pub, rate, v, omega)
    elif traj == 'square':
        publish_square(pub, rate, v, omega_90)
    elif traj == 'straight':
        publish_straight(pub, rate, v)
    else:
        rospy.logerr("Traiettoria non riconosciuta: %s", traj)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
