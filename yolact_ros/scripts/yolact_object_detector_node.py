#! /usr/bin/env python

import rospy

from yolact_object_detector import YolactObjectDetector

if __name__ == "__main__":
    rospy.init_node("yolact_ros")
    YolactObjectDetector()
    rospy.spin()
