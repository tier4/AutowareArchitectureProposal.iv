# Autoware Global Parameter Loader

This package is to record ros2 bag with desired topic.

## Usage

Here is a usage below.

```sh
ros2 run autoware_rosbag_recorder record.sh [-o filename]
```

If you want some other topics to record, the you need to modify below in record.sh as an example.

```sh
  ros2 bag record -e "(.*)/velodyne_packets|/as/(.*)|/pacmod/(.*)|/vehicle/(.*)|/sensing/imu/(.*)|/sensing/gnss/(.*)|/sensing/camera/(.*)/camera_info|/sensing/camera/(.*)/compressed|/perception/object_recognition/detection/rois(.)|/perception/object_recognition/objects" -o "$OPTARG";
```

## Assumptions / Known limits

recording to all topics in autoware is very heavy so usually you need to avoid points cloud topics to record.
