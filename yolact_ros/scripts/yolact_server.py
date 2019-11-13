#! /usr/bin/env python

import os
import time
from collections import defaultdict

import actionlib
import cv_bridge
import numpy as np
import rospy
import torch
import torch.backends.cudnn as cudnn
from yolact_ros.msg import Segment, Segments, SegmentationGoal, SegmentationResult, SegmentationAction

from modules.data import cfg, set_cfg
from modules.layers.output_utils import postprocess, undo_image_transformation
from modules.utils import timer
from modules.utils import FastBaseTransform
from modules.utils.functions import SavePath
from yolact import Yolact


class YolactRos:

    def __init__(self):

        rospy.init_node("yolact_server")

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
        rospy.loginfo("Ready...")
        self.server = actionlib.SimpleActionServer("/yolact_ros/check_for_objects", SegmentationAction, self.call_back,
                                                   auto_start=False)
        self.server.start()

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
            print("Loading model Done.")

            self.net.detect.use_fast_nms = True
            cfg.mask_proto_debug = False

    def call_back(self, goal):
        """
        :type goal: SegmentationGoal
        :return: SegmentationResult
        """
        print("subscribe")
        image = self._bridge.imgmsg_to_cv2(goal.image)
        msg = self.eval_image(image)
        if not self.server.is_preempt_requested():
            if msg is not None:
                result = SegmentationResult()
                result.segments = msg
                result.id = goal.id
                self.server.set_succeeded(result)

    def eval_image(self, image):
        frame = torch.from_numpy(image).cuda().float()
        batch = FastBaseTransform()(frame.unsqueeze(0))
        preds = self.net(batch)

        start = time.time()
        try:
            msg = self.prep_display(preds, frame, None, None, undo_transform=False)
        except Exception as e:
            rospy.logerr(e)
            msg = None
        elapsed_time = time.time() - start
        rospy.loginfo("elapsed_time:{0}".format(elapsed_time) + "[sec]")
        return msg

    def prep_display(self, dets_out, img, h, w, undo_transform=True, class_color=False, mask_alpha=0.45):
        """
        Note: If undo_transform=False then im_h and im_w are allowed to be None.
        """
        if undo_transform:
            img_numpy = undo_image_transformation(img, w, h)
            img_gpu = torch.Tensor(img_numpy).cuda()
        else:
            img_gpu = img / 255.0
            h, w, _ = img.shape

        with timer.env("Postprocess"):
            t = postprocess(dets_out, w, h,
                            crop_masks=True,
                            score_threshold=self.score_threshold)
            torch.cuda.synchronize()

        with timer.env("Copy"):
            if cfg.eval_mask_branch:
                # Masks are drawn on the GPU, so don"t copy
                masks = t[3][:self.top_k]
            classes, scores, boxes = [x[:self.top_k].cpu().detach().numpy() for x in t[:3]]
            print(classes)
            print(scores)

        num_dets_to_consider = min(self.top_k, classes.shape[0])
        for j in range(num_dets_to_consider):
            if scores[j] < self.score_threshold:
                num_dets_to_consider = j
                break

        if num_dets_to_consider == 0:
            # No detections found so just output the original image
            # return (img_gpu * 255).byte().cpu().numpy()
            return Segments()

        mask_indices = [0] * num_dets_to_consider
        for i in range(num_dets_to_consider):
            m = masks.byte().cpu().numpy()
            m = np.where(m[i] > 0)
            mask_indices[i] = m

        return self.create_msg(mask_indices, classes, scores, boxes)
        #
        # # Quick and dirty lambda for selecting the color for a particular index
        # # Also keeps track of a per-gpu color cache for maximum speed
        # def get_color(j, on_gpu=None):
        #     color_idx = (classes[j] * 5 if class_color else j * 5) % len(COLORS)
        #
        #     if on_gpu is not None and color_idx in self.color_cache[on_gpu]:
        #         return self.color_cache[on_gpu][color_idx]
        #     else:
        #         color = COLORS[color_idx]
        #         if not undo_transform:
        #             # The image might come in as RGB or BRG, depending
        #             color = (color[2], color[1], color[0])
        #         if on_gpu is not None:
        #             color = torch.Tensor(color).to(on_gpu).float() / 255.
        #             self.color_cache[on_gpu][color_idx] = color
        #         return color
        #
        # # First, draw the masks on the GPU where we can do it really fast
        # # Beware: very fast but possibly unintelligible mask-drawing code ahead
        # # I wish I had access to OpenGL or Vulkan but alas, I guess Pytorch tensor operations will have to suffice
        # if cfg.eval_mask_branch:
        #     # After this, mask is of size [num_dets, h, w, 1]
        #
        #     # Prepare the RGB images for each mask given their color (size [num_dets, h, w, 1])
        #     colors = torch.cat([get_color(j, on_gpu=img_gpu.device.index).view(1, 1, 1, 3) for j in range(num_dets_to_consider)], dim=0)
        #
        #     masks = masks[:num_dets_to_consider, :, :, None]
        #     masks_color = masks.repeat(1, 1, 1, 3) * colors * mask_alpha
        #
        #     # This is 1 everywhere except for 1-mask_alpha where the mask is
        #     inv_alph_masks = masks * (-mask_alpha) + 1
        #
        #     # I did the math for this on pen and paper. This whole block should be equivalent to:
        #
        #     masks_color_summand = masks_color[0]
        #     if num_dets_to_consider > 1:
        #         inv_alph_cumul = inv_alph_masks[:(num_dets_to_consider - 1)].cumprod(dim=0)
        #         masks_color_cumul = masks_color[1:] * inv_alph_cumul
        #         masks_color_summand += masks_color_cumul.sum(dim=0)
        #
        #     img_gpu = img_gpu * inv_alph_masks.prod(dim=0) + masks_color_summand
        #
        # # Then draw the stuff that needs to be done on the cpu
        # # Note, make sure this is a uint8 tensor or opencv will not anti alias text for whatever reason
        # img_numpy = (img_gpu * 255).byte().cpu().numpy()
        # for j in reversed(range(num_dets_to_consider)):
        #     x1, y1, x2, y2 = boxes[j, :]
        #     color = get_color(j)
        #     score = scores[j]
        #     _class = cfg.dataset.class_names[classes[j]]
        #     print(_class)
        #     # if _class == "refrigerator":
        #     #     continue
        #     cv2.rectangle(img_numpy, (x1, y1), (x2, y2), color, 1)
        #
        #     text_str = "%s: %.2f" % (_class, score)
        #
        #     font_face = cv2.FONT_HERSHEY_DUPLEX
        #     font_scale = 0.6
        #     font_thickness = 1
        #
        #     text_w, text_h = cv2.getTextSize(text_str, font_face, font_scale, font_thickness)[0]
        #
        #     text_pt = (x1, y1 - 3)
        #     text_color = [255, 255, 255]
        #
        #     cv2.rectangle(img_numpy, (x1, y1), (x1 + text_w, y1 - text_h - 4), color, -1)
        #     cv2.putText(img_numpy, text_str, text_pt, font_face, font_scale, text_color, font_thickness, cv2.LINE_AA)
        #
        # img_numpy = img_numpy[:, :, (2, 1, 0)]
        #
        # plt.imshow(img_numpy)
        # plt.title("0")
        # plt.show()
        # return self.create_msg(mask_indices, classes, scores, boxes)

    @staticmethod
    def create_msg(mask_indices, classes, scores, boxes):
        segments = Segments()
        for i in range(len(classes)):
            x1, y1, x2, y2 = boxes[i, :]
            segment = Segment()
            segment.Class = cfg.dataset.class_names[classes[i]]
            if segment.Class == "refrigerator":
                continue
            segment.probability = scores[i]
            segment.xmin = x1
            segment.ymin = y1
            segment.xmax = x2
            segment.ymax = y2
            indices = mask_indices[i][1] + (640 * mask_indices[i][0])
            segment.x_masks = mask_indices[i][1]
            segment.y_masks = mask_indices[i][0]
            segment.pixel_size = len(indices)
            segments.segments.append(segment)

        return segments


if __name__ == "__main__":
    YolactRos()

    rospy.spin()
