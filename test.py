#! /usr/bin/env python
import os
import sys

import actionlib
import rospy

import cv_bridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from yolact_ros.msg import Segment, SegmentationGoal, SegmentationAction, SegmentationResult


class Test:

    def __init__(self):
        rospy.init_node("test")

        self.bridge = cv_bridge.CvBridge()

        self.client = actionlib.SimpleActionClient("/segmentation", SegmentationAction)

        cv_image = cv2.imread("./test2.png")
        # plt.imshow(cv_image)
        # plt.show()
        #
        # rospy.spin()

        image = self.bridge.cv2_to_imgmsg(cv_image)

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

        for i, segment in enumerate(result.segments.segments):  # type: Segment
            print(i, segment.Class)
            img = cv_image.copy()
            indices_x = segment.mask_indices_x
            indices_y = segment.mask_indices_y
            for x, y in zip(indices_x, indices_y):
                img[x, y] = 255, 0, 0

            x1 = segment.xmin
            x2 = segment.xmax
            y1 = segment.ymin
            y2 = segment.ymax
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 1)

            text_str = "%s: %.2f" % (segment.Class, segment.probability)

            font_face = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 0.6
            font_thickness = 1

            text_w, text_h = cv2.getTextSize(text_str, font_face, font_scale, font_thickness)[0]

            text_pt = (x1, y1 - 3)
            text_color = [255, 255, 255]

            cv2.rectangle(img, (x1, y1), (x1 + text_w, y1 - text_h - 4), (0, 0, 255), -1)
            cv2.putText(img, text_str, text_pt, font_face, font_scale, text_color, font_thickness, cv2.LINE_AA)
            cv2.imwrite('/root/HSR/test/test1_{}.png'.format(i), img)

        for segment in result.segments.segments:  # type: Segment
            cv2.rectangle(cv_image, (segment.xmin, segment.ymin), (segment.xmax, segment.ymax), (255, 0, 0), 1)

        print("save")

        cv2.imwrite('/root/HSR/test/test1_s.png', cv_image)

        sys.exit(1)


if __name__ == '__main__':
    Test()
