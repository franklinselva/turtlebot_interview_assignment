# -*- coding: utf-8 -*-
# @Author: Franklin Selva
# @Date:   2021-09-28 21:21:07
# @Last Modified by:   Franklin Selva
# @Last Modified time: 2021-09-29 20:20:00
#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import sys, select
import tty, termios

WAFFLE_MAX_LIN_VEL = 0.22
WAFFLE_MAX_ANG_VEL = 0.22

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

STATUS: bool = False
settings = termios.tcgetattr(sys.stdin)

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Waffle Pi max : ~ 0.22)
a/d : increase/decrease angular velocity (Waffle Pi max: ~ 0.22)

space key, s : force stop

"""


def callback(msg):
    global STATUS
    STATUS = msg.data


def get_key():
    """Get Keyboard input realtime even without user entering Enter by changing terminal settings

    Returns:
        str: pressed key
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def constrain(input: float, low: float, high: float):
    """To set velocities to minimum and maximum

    Args:
        input (float): Input velocity
        low (float): Minimum velocity
        high (float): Maximum velocity

    Returns:
        float: Returns mapped velocity
    """
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input


def limit_linear_velocity(vel: float):
    """Check Linear Velocity limit

    Args:
        vel (float): Input velocity

    Returns:
        float: Constrained velocity within limits
    """
    vel = constrain(vel, -WAFFLE_MAX_LIN_VEL / 2, WAFFLE_MAX_LIN_VEL)
    return vel


def limit_angular_velocity(vel: float):
    """Check angular velocity limit

    Args:
        vel (float): Input velocity

    Returns:
        float: Constrained velocity within limits
    """
    vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return vel


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel {%.2f}\t angular vel {%.2f} " % (
        target_linear_vel,
        target_angular_vel,
    )


def main():
    """Main Function"""
    rospy.init_node("teleop_control_node", anonymous=False)

    sub = rospy.Subscriber("teleop_mode_status", Bool, callback)
    publish_status = rospy.Publisher("teleop_mode_status", Bool, queue_size=10)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(60)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    global STATUS

    try:
        print(msg)
        while not rospy.is_shutdown():
            if STATUS:
                rospy.loginfo_once("Launching Teleop Control node")
                key = get_key()
                if key == "w":
                    target_linear_vel = limit_linear_velocity(
                        target_linear_vel + LIN_VEL_STEP_SIZE
                    )
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key == "x":
                    target_linear_vel = limit_linear_velocity(
                        target_linear_vel - LIN_VEL_STEP_SIZE
                    )
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key == "a":
                    target_angular_vel = limit_angular_velocity(
                        target_angular_vel + ANG_VEL_STEP_SIZE
                    )
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key == "d":
                    target_angular_vel = limit_angular_velocity(
                        target_angular_vel - ANG_VEL_STEP_SIZE
                    )
                    status = status + 1
                    print(vels(target_linear_vel, target_angular_vel))
                elif key == " " or key == "s":
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))
                # else:
                #     if key == "\x03":
                #         publish_status.publish(False)
                #         rospy.loginfo("Closing Teleop control")

                if status == 20:
                    status = 0

                twist = Twist()

                twist.linear.x = target_linear_vel
                twist.angular.z = target_angular_vel

                pub.publish(twist)
                rate.sleep()
    except Exception as e:
        rospy.logerr(e)
        publish_status.publish(False)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
