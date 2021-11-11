# shape_estimation

## Purpose

This node calculates a refined object shape from a labeled pointcloud cluster by using L-shape fitting.

## Inner-workings / Algorithms

L-shape fitting implementation of the paper:

```bibtex
@conference{Zhang-2017-26536,
author = {Xiao Zhang and Wenda Xu and Chiyu Dong and John M. Dolan},
title = {Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners},
booktitle = {2017 IEEE Intelligent Vehicles Symposium},
year = {2017},
month = {June},
keywords = {autonomous driving, laser scanner, perception, segmentation},
}
```

## Inputs / Outputs

### Input

| Name    | Type                                                        | Description                           |
| ------- | ----------------------------------------------------------- | ------------------------------------- |
| `input` | `autoware_perception_msgs::msg::DetectedObjectsWithFeature` | detected objects with labeled cluster |

### Output

| Name             | Type                                                  | Description                         |
| ---------------- | ----------------------------------------------------- | ----------------------------------- |
| `output/objects` | `autoware_auto_perception_msgs::msg::DetectedObjects` | detected objects with refined shape |

## Parameters

| Name                        | Type | Default Value | Description                                        |
| --------------------------- | ---- | ------------- | -------------------------------------------------- |
| `use_corrector`             | bool | true          | The flag to apply rule based filter.               |
| `use_filter`                | bool | true          | The flag to apply rule based corrector             |
| `use_vehicle_reference_yaw` | bool | true          | The flag to use vehicle reference yaw for corrector |

## Assumptions / Known limits

TBD
