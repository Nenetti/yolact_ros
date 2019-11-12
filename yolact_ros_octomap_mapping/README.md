# 'yolact_ros_octomap_mapping' Package

The `yolact_ros_octomap_mapping` package provides real-time segmentation octomap.
*   Maintainer: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).
*   Author: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).

**Content:**

*   [Launch](#Launch)
*   [Actions](#Actions)

## Launch

*   `octomap_dynamic.launch`: Start the octomap_server action.

## Action
* **`yolact_ros/check_for_objects`** ([yolact_ros::Segmentation])

    Sends an action with an image and the result is an array of bounding boxes and masks.