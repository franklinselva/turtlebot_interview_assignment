# -*- coding: utf-8 -*-
# @Author: Franklin Selva
# @Date:   2021-09-28 21:24:07
# @Last Modified by:   Franklin Selva
# @Last Modified time: 2021-09-28 21:34:38
#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

STATUS: bool


def callback(msg):
    STATUS = msg.data


def main():
    """Main Function"""
    rospy.init_node("logger_node", anonymous=False)
    subscriber = rospy.Subscriber("log_mode_status", Bool, callback)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
