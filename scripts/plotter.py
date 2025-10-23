#!/usr/bin/env python3

import rospy
import rospkg
import os
import csv
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

class Plotter:
    def __init__(self):
        rospy.init_node('plotter')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vitarana_drone')

        self.plot_dir = os.path.join(pkg_path, 'plot')

        if not os.path.exists(self.plot_dir):
            os.makedirs(self.plot_dir)

        self.topics = {
            '/my_robot/odom': 'main_car_odom.csv',
            '/drone_ff_control': 'main_ff_value.csv',
            '/drone_base_control': 'main_base_pid_value.csv',
            '/drone_pose_pub': 'main_drone_pose.csv'
        }

        self.messages = {
            '/my_robot/odom': Odometry,
            '/drone_ff_control': PoseStamped,
            '/drone_base_control': PoseStamped,
            '/drone_pose_pub': Odometry
        }

        self.files = {}
        self.writers = {}
        self.subscribers = {}

        for topic, filename in self.topics.items():
            msg_type = self.messages[topic]
            if msg_type is None:
                rospy.logwarn(f"[PLOTTER] No message type defined for {topic}, skip.")
                continue

            path = os.path.join(self.plot_dir, filename)
            f = open(path, 'w')
            writer = csv.writer(f)
            
            self.files[topic] = f
            self.writers[topic] = writer
            rospy.Subscriber(topic, msg_type, lambda msg, t=topic: self.generic_callback(msg, t))
            #self.subscribers[topic] = rospy.Subscriber(topic, msg_type, lambda msg, t=topic: self.generic_callback(msg, t) )

        rospy.spin()

    def generic_callback(self, msg, topic):
        t = msg.header.stamp.to_sec()
        
        if isinstance(msg, PoseStamped):
            if self.files[topic].tell() == 0:
                self.writers[topic].writerow(["t", "x", "y", "z"])

            t = f"{msg.header.stamp.to_sec():.3f}"
            x = round(msg.pose.position.x, 2)
            y = round(msg.pose.position.y, 2)
            z = round(msg.pose.position.z, 2)
            self.writers[topic].writerow([t, x, y, z])
        elif isinstance(msg, Odometry):
            if self.files[topic].tell() == 0:
                self.writers[topic].writerow(["t", "x", "y", "z", "vx", "vy", "vz"])

            t = f"{msg.header.stamp.to_sec():.3f}"
            x = round(msg.pose.pose.position.x, 2)
            y = round(msg.pose.pose.position.y, 2)
            z = round(msg.pose.pose.position.z, 2)
            vx = round(msg.pose.twist.twist.linear.x, 2)
            vy = round(msg.pose.twist.twist.linear.y, 2)
            vz = round(msg.pose.twist.twist.linear.z, 2)
            self.writers[topic].writerow([t, x, y, z, vx, vy, vz])
        elif isinstance(msg, Twist):
            x = round(msg.linear.x, 2)
            y = round(msg.linear.y, 2)
            z = round(msg.linear.z, 2)
            self.writers[topic].writerow([t, x, y, z])
        else:
            rospy.logwarn(f"[PLOTTER] No message type define for {topic}: editat generic_callback in plotter.py")
            return

        self.files[topic].flush()

    def close(self):
        for f in self.files.values():
            f.close()

def main():
    plotter = Plotter()
    

if __name__ == '__main__':
    main()
