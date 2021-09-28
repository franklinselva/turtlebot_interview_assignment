# -*- coding: utf-8 -*-
# @Author: Franklin Selva
# @Date:   2021-09-28 20:31:57
# @Last Modified by:   Franklin Selva
# @Last Modified time: 2021-09-28 23:03:26
#!/usr/bin/env python
import tty
import sys
import termios
import select

import rospy
from std_msgs.msg import Bool


settings = termios.tcgetattr(sys.stdin)

publish_teleop_mode = rospy.Publisher("teleop_mode_status", Bool, queue_size=10)
publish_screenshot_mode = rospy.Publisher("screenshot_mode_status", Bool, queue_size=10)
publish_graph_mode = rospy.Publisher("graph_mode_status", Bool, queue_size=10)
publish_log_mode = rospy.Publisher("log_mode_status", Bool, queue_size=10)


def get_key():
    """Get Keyboard input realtime even without user entering Enter by changing terminal settings

    Returns:
        key: Pressed key
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def mode_select(control_mode: list):
    """Select mode based on keyboard input

    Args:
        control_mode (list): pressed key in order [teleop_control, screenshot, graph, log]
    """
    try:
        if control_mode[0]:
            publish_teleop_mode.publish(True)
        if control_mode[1]:
            publish_screenshot_mode.publish(True)
        if control_mode[2]:
            publish_graph_mode.publish(True)
        if control_mode[3]:
            publish_log_mode.publish(True)

    except Exception as e:
        rospy.logerr(e)


def main():
    """Main Function"""
    try:
        rospy.init_node("mode_selector", anonymous=False)
        rospy.loginfo("Starting Mode Selection Node")
        rate = rospy.Rate(60)

        control = [bool] * 4

        while not rospy.is_shutdown():
            key = get_key()
            if key == str(1):
                rospy.loginfo("Selected Teleop Key mode")
                control[0] = True
            if key == str(2):
                rospy.loginfo("Capturing current screenshot")
                control[1] = True
            if key == str(3):
                rospy.loginfo("Starting real-time graph")
                control[2] = True
            if key == str(3):
                rospy.loginfo("Saving log files")
                control[3] = True
            if key == "\x03":
                rospy.loginfo("Closing Mode Selection Node")
                break

            mode_select(control)
            rate.sleep()

    except Exception as e:
        rospy.logerr(e)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
