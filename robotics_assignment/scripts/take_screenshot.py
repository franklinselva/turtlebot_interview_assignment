# -*- coding: utf-8 -*-
# @Author: Franklin Selva
# @Date:   2021-09-28 21:22:45
# @Last Modified by:   Franklin Selva
# @Last Modified time: 2021-09-29 11:45:48
#!/usr/bin/env python
import os
import gi
from time import time

gi.require_version("Gdk", "3.0")
from gi.repository import Gdk

import rospy
import rospkg
from std_msgs.msg import Bool

STATUS: bool = False


def callback(msg):
    global STATUS
    STATUS = msg.data


def main():
    """Main Function"""
    # TODO: Capture only specific windows
    rospy.init_node("screenshot_control", anonymous=False)
    subscriber = rospy.Subscriber("screenshot_mode_status", Bool, callback)
    publisher = rospy.Publisher("screenshot_mode_status", Bool, queue_size=10)
    rate = rospy.Rate(60)

    save_dir = os.path.join(
        rospkg.RosPack().get_path("robotics_assignment"), "screenshots"
    )

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    else:
        for file in os.listdir(save_dir):
            os.remove(os.path.join(save_dir, file))

    while not rospy.is_shutdown():
        if STATUS:
            window = Gdk.get_default_root_window()
            screen = window.get_screen()
            typ = window.get_type_hint()

            for i, w in enumerate(screen.get_window_stack()):
                pb = Gdk.pixbuf_get_from_window(w, *w.get_geometry())
                pb.savev(
                    os.path.join(save_dir, "{}_{}.png".format(i, time())), "png", (), ()
                )

            rospy.loginfo(f"Screenshots have been at {save_dir}")
            publisher.publish(False)


if __name__ == "__main__":
    main()
