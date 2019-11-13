# 'yolact_ros' Package

The `yolact_ros` meta-package contains two packages that together enable real-time instance segmentation: `yolact_ros` and `yolact_ros_octomap_mapping`.

*   Maintainer: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).
*   Author: Akinori Kanechika ([kanechika.akinori@em.ci.ritsumei.ac.jp](mailto:kanechika.akinori@em.ci.ritsumei.ac.jp)).

**Content:**

*   ['yolact_ros' Package](#yolact_ros_package)
    *   [History](#history)
    *   [Launch](#launch)
    *   [Action](#action)
*   ['yolact_ros_octomap_mapping' Package](#yolact_ros_octomap_mapping_package)
    *   [History](#history)
    *   [Launch](#launch)

## 'yolact_ros' Package

The `yolact_ros` package provides real-time instance segmentation.

### History

Code forked from [dbolya/yolact](https://github.com/dbolya/yolact) at:
*   URL: `https://github.com/dbolya/yolact`
*   Branch: `master`
*   Commit: [`1763e8fc2f90c25293695f3e0a126978cc01bfd9`](https://github.com/dbolya/yolact/tree/1763e8fc2f90c25293695f3e0a126978cc01bfd9)

Original `README.md`: https://github.com/dbolya/yolact/blob/1763e8fc2f90c25293695f3e0a126978cc01bfd9/README.md

Original `LICENSE`: https://github.com/dbolya/yolact/blob/1763e8fc2f90c25293695f3e0a126978cc01bfd9/LICENSE

### Launch

*   `yolact_ros.launch`: Start the server of `yolact_ros` action.
*   `mapping_demo.launch`: Start the demonstration of the segmentation action.

## Nodes
### Subscribed Topics
* **`yolact_ros/image`** ([sensor_msgs/Image])

### Published Topics
* **`yolact_ros/detection_image`** ([sensor_msgs/Image])
* **`yolact_ros/segments`** ([yolact_ros/Segments])

### Action
* **`yolact_ros/check_for_objects`** ([yolact_ros/Segmentation])

    Sends an action with an image and the result is an array of bounding boxes and masks.

## 'yolact_ros_octomap_mapping' Package

The `yolact_ros_octomap_mapping` package provides real-time point cloud segmentation and OctoMap.

### History

Code forked from [OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping) at:
*   URL: `https://github.com/OctoMap/octomap_mapping`
*   Branch: `kinetic-devel`
*   Commit: [`15953cf25db28ff351a4e21d5277371aeb4e12ca`](https://github.com/OctoMap/octomap_mapping/tree/15953cf25db28ff351a4e21d5277371aeb4e12ca)

Original `README.md`: https://github.com/OctoMap/octomap_mapping/blob/15953cf25db28ff351a4e21d5277371aeb4e12ca/README.md

### Launch

*   `octomap_dynamic.launch`: Start the `octomap_server` action.
