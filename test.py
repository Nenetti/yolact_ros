#! /usr/bin/env python
import os
import sys

import actionlib
import rospy

import cv_bridge
import cv2

from yolact_ros_msgs.msg import Segment, SegmentationGoal, SegmentationAction, SegmentationResult


class Test:

    def __init__(self):
        rospy.init_node("test")

        self.bridge = cv_bridge.CvBridge()

        self.client = actionlib.SimpleActionClient("/segmentation", SegmentationAction)

        cv_image = cv2.imread("./test1.png")
        image = self.bridge.cv2_to_imgmsg(cv_image)
        print(os.path.exists("/root/HSR/test1.png"))

        goal = SegmentationGoal()
        goal.image = image

        print("wait")
        self.client.wait_for_server()

        print("send_goal")
        self.client.send_goal(goal)

        print("wait_result")
        self.client.wait_for_result()
        result = self.client.get_result()  # type: SegmentationResult
        print(self.client.get_state())

        for segment in result.segments.segments:  # type: Segment
            indices_x = segment.mask_indices_x
            indices_y = segment.mask_indices_y
            print(len(indices_x), len(indices_y))
            for x, y in zip(indices_x, indices_y):
                cv_image[x, y] = 0, 0, 0

        print("save")
        cv2.imwrite('/root/HSR/test1_s.png', cv_image)

        sys.exit(1)


if __name__ == '__main__':
    Test()
