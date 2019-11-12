# 'yolact_ros_octomap_mapping' Package

The `yolact_ros_octomap_mapping` package provides real-time segmentation octomap.

Fork from [OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping)


*   Maintainer: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).
*   Author: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).



**Content:**

*   [Original Repository](#Fork Repository)
*   [Launch](#Launch)
*   [Actions](#Actions)

## Original Repository

* URL: `https://github.com/OctoMap/octomap_mapping`

* Branch: kinetic-devel

* Commit ID: `15953cf25db28ff351a4e21d5277371aeb4e12ca`


## Launch

*   `octomap_dynamic.launch`: Start the octomap_server action.

## Action
* **`yolact_ros/check_for_objects`** ([yolact_ros::Segmentation])

    Sends an action with an image and the result is an array of bounding boxes and masks.