# 'yolact_ros' Package

The `yolact_ros` meta-package contains two packages that together enable real-time instance segmentation: `yolact_ros` and `yolact_ros_octomap_mapping`.

*   Maintainer: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).
*   Author: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).

**Content:**

*   ['yolact_ros' Package](#yolact_ros_package)
    *   [History](#yolact_ros_history)
    *   [Launch](#yolact_ros_launch)
    *   [Nodes](#yolact_ros_nodes)
*   ['yolact_ros_octomap_mapping' Package](#yolact_ros_octomap_mapping_package)
    *   [History](#yolact_ros_octomap_mapping_history)
    *   [Launch](#yolact_ros_octomap_mapping_launch)

# 'yolact_ros' Package <a id="yolact_ros_package"></a>

The `yolact_ros` package provides real-time instance segmentation.

## History <a id="yolact_ros_history"></a>

Code forked from [dbolya/yolact](https://github.com/dbolya/yolact) at:
*   URL: `https://github.com/dbolya/yolact`
*   Branch: `master`
*   Commit: [`1763e8fc2f90c25293695f3e0a126978cc01bfd9`](https://github.com/dbolya/yolact/tree/1763e8fc2f90c25293695f3e0a126978cc01bfd9)

Original `README.md`: https://github.com/dbolya/yolact/blob/1763e8fc2f90c25293695f3e0a126978cc01bfd9/README.md

Original `LICENSE`: https://github.com/dbolya/yolact/blob/1763e8fc2f90c25293695f3e0a126978cc01bfd9/LICENSE

## Launch <a id="yolact_ros_launch"></a>

*   `yolact_ros.launch`: Start the server of `yolact_ros` action.
*   `mapping_demo.launch`: Start the demonstration of the segmentation action.

## Nodes <a id="yolact_ros_nodes"></a>

Subscribed topic:
* **`yolact_ros/image/compressed`** ([sensor_msgs/CompressedImage])

Published topics:
* **`yolact_ros/detection/image/compressed`** ([sensor_msgs/CompressedImage])
* **`yolact_ros/segmentation/image/compressed`** ([sensor_msgs/CompressedImage])
* **`yolact_ros/segments`** ([yolact_ros/Segments])

Action:
* **`yolact_ros/check_for_objects`** ([yolact_ros/Segmentation])  
Send an action with an image. The result is an array of bounding boxes and masks.

# 'yolact_ros_octomap_mapping' Package <a id="yolact_ros_octomap_mapping_package"></a>

The `yolact_ros_octomap_mapping` package provides real-time point cloud segmentation and OctoMap.

## History <a id="yolact_ros_octomap_mapping_history"></a>

Code forked from [OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping) at:
*   URL: `https://github.com/OctoMap/octomap_mapping`
*   Branch: `kinetic-devel`
*   Commit: [`15953cf25db28ff351a4e21d5277371aeb4e12ca`](https://github.com/OctoMap/octomap_mapping/tree/15953cf25db28ff351a4e21d5277371aeb4e12ca)

Original `README.md`: https://github.com/OctoMap/octomap_mapping/blob/15953cf25db28ff351a4e21d5277371aeb4e12ca/README.md

## Launch <a id="yolact_ros_octomap_mapping_launch"></a>

*   `octomap_dynamic.launch`: Start the `octomap_server` action.
