#! /usr/bin/env python

import cv2
import os
import time
from collections import defaultdict

import actionlib
import cv_bridge
import numpy as np
import rospy
import torch
import torch.backends.cudnn as cudnn
from sensor_msgs.msg import CompressedImage
from yolact_ros.msg import Segment, Segments, SegmentationGoal, SegmentationResult, SegmentationAction

from modules.data import cfg, set_cfg, COLORS
from modules.layers.output_utils import postprocess
from modules.utils import FastBaseTransform
from modules.utils import timer
from modules.utils.functions import SavePath
from modules.yolact import Yolact


class YolactObjectDetector:

    def __init__(self):

        self.iou_thresholds = [x / 100 for x in range(50, 100, 5)]
        self.coco_cats = {}
        self.coco_cats_inv = {}
        self.color_cache = defaultdict(lambda: {})

        self.net = None

        self._bridge = cv_bridge.CvBridge()

        self.config = None
        self.trained_model = rospy.get_param("~trained_model")
        self.score_threshold = rospy.get_param("~score_threshold", default=0.3)
        self.top_k = rospy.get_param("~top_k", default=5)

        self.load_model()
        self.server = actionlib.SimpleActionServer("yolact_ros/check_for_objects", SegmentationAction,
                                                   self.action_call_back, auto_start=False)
        self.subscriber = rospy.Subscriber("yolact_ros/image/compressed", CompressedImage, self.call_back, queue_size=1)
        self.image_pub = rospy.Publisher("yolact_ros/detection/image/compressed", CompressedImage, queue_size=1)
        self.segments_info_pub = rospy.Publisher("yolact_ros/segments", Segments, queue_size=1)
        self.server.start()

    def action_call_back(self, goal):
        """
        :type goal: SegmentationGoal
        :return: SegmentationResult
        """
        print("subscribe")
        image = self._bridge.compressed_imgmsg_to_cv2(goal.image, desired_encoding="rgb8")
        msg = self.eval_image(image)
        if not self.server.is_preempt_requested():
            if msg is not None:
                result = SegmentationResult()
                result.segments = msg
                result.id = goal.id
                self.server.set_succeeded(result)

                detect_image = self.to_mask_image(image, msg)
                result_msg = self._bridge.cv2_to_compressed_imgmsg(detect_image, dst_format='jpg')
                result_msg.header.stamp = rospy.Time.now()
                self.image_pub.publish(result_msg)

    def call_back(self, msg):
        """
        :type msg: CompressedImage
        """
        print("subscribe")
        image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")
        result = self.eval_image(image)
        detect_image = self.to_mask_image(image, result)
        result_msg = self._bridge.cv2_to_compressed_imgmsg(detect_image, dst_format='jpg')
        result_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish(result_msg)
        self.segments_info_pub.publish(result)

    @staticmethod
    def to_mask_image(image, segments):
        """
        :type image:np.ndarray
        :type segments:Segments
        :return:
        """
        mask_img = image.copy()
        for i, segment in enumerate(segments.segments):
            color = COLORS[i * 5 % len(COLORS)]
            for x, y in zip(segment.x_masks, segment.y_masks):
                mask_img[y, x] = color

        result_img = cv2.addWeighted(image, 0.5, mask_img, 0.5, 20)
        for i, segment in enumerate(segments.segments):
            color = COLORS[i * 5 % len(COLORS)]
            x0 = segment.xmin
            y0 = segment.ymin
            x1 = segment.xmax
            y1 = segment.ymax
            cv2.rectangle(result_img, (x0, y0), (x1, y1), color, 2)
            label_str = '{0}: {1:.2f}'.format(segment.Class, segment.probability)
            text_config = {'text': label_str, 'fontFace': cv2.FONT_HERSHEY_DUPLEX, 'fontScale': 0.6, 'thickness': 1}
            size, baseline = cv2.getTextSize(**text_config)
            cv2.rectangle(result_img, (x0, y0), (x0 + size[0], y0 + size[1]), (255, 255, 255), cv2.FILLED)
            cv2.putText(result_img, org=(x0, y0 + size[1]), color=(255, 0, 0), **text_config)

        return cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)

    def load_model(self):

        model_path = SavePath.from_str(self.trained_model)
        # TODO: Bad practice? Probably want to do a name lookup instead.
        self.config = model_path.model_name + "_config"
        print("Config not specified. Parsed %s from the file name.\n" % self.config)
        set_cfg(self.config)

        with torch.no_grad():
            if not os.path.exists("results"):
                os.makedirs("results")

            cudnn.benchmark = True
            cudnn.fastest = True
            torch.set_default_tensor_type("torch.cuda.FloatTensor")

            print("Loading model...")
            self.net = Yolact()
            self.net.load_weights(self.trained_model)
            self.net.eval()
            self.net = self.net.cuda()
            print("Loading model done.")

            self.net.detect.use_fast_nms = True
            cfg.mask_proto_debug = False

        rospy.loginfo("Ready...")

    def eval_image(self, image):
        frame = torch.from_numpy(image).cuda().float()
        batch = FastBaseTransform().forward(frame.unsqueeze(0))
        preds = self.net(batch)

        start = time.time()
        try:
            msg = self.prep_display(preds, frame)
        except Exception as e:
            rospy.logerr(e)
            msg = None
        elapsed_time = time.time() - start
        rospy.loginfo("elapsed_time:{0}".format(elapsed_time) + "[sec]")
        return msg

    def prep_display(self, dets_out, img):
        """
        Note: If 'undo_transform=False' then 'im_h' and 'im_w' are allowed to be 'None'.
        """
        h, w, _ = img.shape

        with timer.env("Postprocess"):
            t = postprocess(dets_out, w, h,
                            crop_masks=True,
                            score_threshold=self.score_threshold)
            torch.cuda.synchronize(torch.cuda.current_device())

        with timer.env("Copy"):
            if cfg.eval_mask_branch:
                # Masks are drawn on the GPU, so don't copy.
                masks = t[3][:self.top_k]
            classes, scores, boxes = [x[:self.top_k].cpu().detach().numpy() for x in t[:3]]

        num_dets_to_consider = min(self.top_k, classes.shape[0])
        for j in range(num_dets_to_consider):
            if scores[j] < self.score_threshold:
                num_dets_to_consider = j
                break

        if num_dets_to_consider == 0:
            # No detections found so just output the original image.
            return Segments()

        mask_indices = [np.zeros(0)] * num_dets_to_consider
        for i in range(num_dets_to_consider):
            m = masks.byte().cpu().numpy()
            m = np.where(m[i] > 0)
            mask_indices[i] = m
        return self.create_msg(mask_indices, classes, scores, boxes)

    @staticmethod
    def create_msg(mask_indices, classes, scores, boxes):
        segments = Segments()
        for i in range(len(classes)):
            x1, y1, x2, y2 = boxes[i, :]
            segment = Segment()
            segment.Class = cfg.dataset.class_names[classes[i]]
            segment.probability = scores[i]
            segment.xmin = x1
            segment.ymin = y1
            segment.xmax = x2
            segment.ymax = y2
            segment.x_masks = mask_indices[i][1]
            segment.y_masks = mask_indices[i][0]
            segments.segments.append(segment)

        sorted(segments.segments, key=lambda x: len(x.x_masks), reverse=True)

        return segments
