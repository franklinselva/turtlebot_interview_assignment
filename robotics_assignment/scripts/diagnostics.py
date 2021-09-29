# -*- coding: utf-8 -*-
# @Author: Franklin Selva
# @Date:   2021-09-29 17:56:33
# @Last Modified by:   Franklin Selva
# @Last Modified time: 2021-09-29 18:08:49
import rospy

from sensor_msgs.msg import CameraInfo
from diagnostic_msgs.msg import DiagnosticArray

RGB_INFO = CameraInfo()
DEPTH_INFO = CameraInfo()
DIAGNOSITCS = DiagnosticArray()


def rgb_callback(msg):
    RGB_INFO = msg


def depth_callback(msg):
    DEPTH_INFO = msg


def diag_callback(msg):
    DIAGNOSITCS = msg


def main():
    """Main function"""
    rospy.init_node("diagnosis_node", anonymous=False)
    rospy.loginfo("Starting Diagnosis node")

    depth_subscriber = rospy.Subscriber(
        "/camera/depth/camera_info", CameraInfo, depth_callback
    )
    rgb_subscriber = rospy.Subscriber(
        "/camera/rgb/camera_info", CameraInfo, rgb_callback
    )
    diag_subscriber = rospy.Subscriber("/diagnostics", DiagnosticArray, diag_callback)

    diagnostics_publisher = rospy.Publisher(
        "/Swarooph_diagnostics", DiagnosticArray, queue_size=10
    )

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        diagnostics_publisher.publish(DIAGNOSITCS)
        rate.sleep()


if __name__ == "__main__":
    main()
