# -*- coding: utf-8 -*-
# @Author: Franklin Selva
# @Date:   2021-09-28 21:23:31
# @Last Modified by:   Franklin Selva
# @Last Modified time: 2021-09-29 12:27:00
#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

STATUS: bool = False
current_pose: Pose = Pose()


def status_callback(msg):
    global STATUS
    STATUS = msg.data


def odom_callback(msg):
    global current_pose
    current_pose = msg.pose.pose


class Plotter:
    x: list = []
    y: list = []

    def __init__(self):
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(1, 1, 1)

    def animate(self, i):
        try:
            self.ax.clear()
            self.ax.plot(self.x, self.y)

            plt.title("Pose tracker - Turtlebot3")
            plt.xlabel("X")
            plt.ylabel("Y")
        except Exception as e:
            rospy.logwarn(e)

    def plot(self, pose: Pose):
        self.pose = pose
        self.x.append(round(pose.position.x, 2))
        self.y.append(round(pose.position.y, 2))

        self.ani = animation.FuncAnimation(self.figure, self.animate, interval=1000)


def main():
    """Main Function"""
    rospy.init_node("graph_node", anonymous=False)
    status_subscriber = rospy.Subscriber("graph_mode_status", Bool, status_callback)
    odom_subscriber = rospy.Subscriber("/odom", Odometry, odom_callback)

    pose_publisher = rospy.Publisher("robot_pose", Pose, queue_size=10)
    rate = rospy.Rate(60)
    global STATUS
    PLOT = Plotter()

    while not rospy.is_shutdown():
        pose_publisher.publish(current_pose)
        if STATUS:
            PLOT.plot(current_pose)
            # plt.show()
            plt.draw()
            plt.pause(0.001)
        rate.sleep()


if __name__ == "__main__":
    main()
