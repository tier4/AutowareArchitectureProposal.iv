# Map Based Prediction

## Role

`map_besed_prediction` is a module to predict the future paths of other vehicles and humans according to the shape of the map and the surrounding environment..

## Inner-workings / Algorithms

1. Get lanelet path
   The first step is to get the lanelet of the current position of the car. After that, we obtain several trajectories based on the map.

2. Lane Change Detection
   After finding the current lanelet from the current position of the obstacle, our algorithm try to detect the lane change maneuver from the past positions of the obstacle. Our method uses the deviation between the obstacle's current position and its position one second ago and current position to determine if it is about to change lanes. The parameters used for the lane change decision are obtained by analyzing the data obtained from the experiment. We already confirmed that these parameters give the least number of false positives.

3. Confidence calculation
   We use the following metric to compute the distance to a certain lane.

   ```txt
   d = x^T P x
   ```

   where `x=[lateral_dist, yaw_diff]` and `P` are covariance matrices. Therefore confidence values can be computed as

   ```txt
   confidence = 1/d
   ```

   Finally, we normalize the confidence value to make it as probability value. Note that the standard deviation of the lateral distance and yaw difference is given by the user.

4. Drawing predicted trajectories
   From the current position and reference trajectories that we get in the step1, we create predicted trajectories by using Quintic polynomial. Note that, since this algorithm consider lateral and longitudinal motions separately, it sometimes generates dynamically-infeasible trajectories when the vehicle travels at a low speed. To deal with this problem, we only make straight line predictions when the vehicle speed is lower than a certain value (which is given as a parameter).

## Inputs / Outputs

### Input

| Name                                               | Type                                                 | Description                              |
| -------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------- |
| `~/perception/object_recognition/tracking/objects` | `autoware_auto_perception_msgs::msg::TrackedObjects` | tracking objects without predicted path. |
| `~/vector_map`                                     | `autoware_auto_mapping_msgs::msg::HADMapBin`         | binary data of Lanelet2 Map.             |

### Output

| Name                     | Type                                                   | Description                           |
| ------------------------ | ------------------------------------------------------ | ------------------------------------- |
| `~/objects`              | `autoware_auto_perception_msgs::msg::PredictedObjects` | tracking objects with predicted path. |
| `~/objects_path_markers` | `visualization_msgs::msg::MarkerArray`                 | marker for visualization.             |

## Parameters

| Parameter                        | Type   | Description                                |
| -------------------------------- | ------ | ------------------------------------------ |
| `has_subscribed_map`             | bool   | true when subscribed path exists           |
| `prediction_time_horizon`        | double | predict time duration for predicted path   |
| `prediction_sampling_delta_time` | double | sampling time for points in predicted path |

## Assumptions / Known limits

`map_based_prediction` can only predict future trajectories for cars, tracks and buses.

## Reference

1. M. Werling, J. Ziegler, S. Kammel, and S. Thrun, “Optimal trajectory generation for dynamic street scenario in a frenet frame,” IEEE International Conference on Robotics and Automation, Anchorage, Alaska, USA, May 2010.
2. A. Houenou, P. Bonnifait, V. Cherfaoui, and Wen Yao, “Vehicle trajectory prediction based on motion model and maneuver recognition,” in 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, nov 2013, pp. 4363–4369.
