# 'yolact_ros' Package

The `yolact_ros` package contains 2 packages.

**Content:**

*   [Packages](#Packages)
*   [Nodes](#Nodes)
*   [Launch](#Launch)

## Packages

*   `yolact_ros`: Run instance segmentation pkg.
*   `yolact_ros_octomap_mapping`: Generate pointcloud segmentation and octomap pkg.

## Launch

*   `yolact_ros.launch`: Start the server of yolact_ros action.
*   `mapping_demo.launch`: Start the demonstration of the segmentation action.

## Nodes
### Subscribed Topics
* **`yolact_ros/image`** ([sensor_msgs/Image])

### Published Topics
* **`yolact_ros/detection_image`** ([sensor_msgs/Image])
* **`yolact_ros/detection_image`** ([yolact_ros/Segments])

### Action
* **`yolact_ros/check_for_objects`** ([yolact_ros/Segmentation])

    Sends an action with an image and the result is an array of bounding boxes and masks.
