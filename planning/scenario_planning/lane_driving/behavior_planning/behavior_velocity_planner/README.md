# Behavior Velocity Planner

## Overview

`behavior_velocity_planner` is a planner that adjust velocity based on the traffic rules.
It consists of several modules.

- Blind Spot
- Crosswalk
- Detection Area
- [Intersection](intersection-design.md)
- Stop Line
- [Traffic Light](traffic-light-design.md)
- Occlusion Spot

When each module plans velocity, it considers based on `base_link`(center of rear-wheel axis) pose.
So for example, in order to stop at a stop line with the vehicles' front on the stop line, it calculates `base_link` position from the distance between `base_link` to front and modifies path velocity from the `base_link` position.

![set_stop_velocity](./docs/set_stop_velocity.drawio.svg)

## Input topics

| Name                          | Type                                                   | Description          |
| ----------------------------- | ------------------------------------------------------ | -------------------- |
| `~input/path_with_lane_id`    | autoware_auto_planning_msgs::msg::PathWithLaneId       | path with lane_id    |
| `~input/vector_map`           | autoware_auto_mapping_msgs::msg::HADMapBin             | vector map           |
| `~input/vehicle_odometry`     | nav_msgs::msg::Odometry                                | vehicle velocity     |
| `~input/dynamic_objects`      | autoware_auto_perception_msgs::msg::PredictedObjects   | dynamic objects      |
| `~input/no_ground_pointcloud` | sensor_msgs::msg::PointCloud2                          | obstacle pointcloud  |
| `~input/traffic_signals`      | autoware_auto_perception_msgs::msg::TrafficSignalArray | traffic light states |

## Output topics

| Name                   | Type                                         | Description                            |
| ---------------------- | -------------------------------------------- | -------------------------------------- |
| `~output/path`         | autoware_auto_planning_msgs::msg::Path       | path to be followed                    |
| `~output/stop_reasons` | autoware_planning_msgs::msg::StopReasonArray | reasons that cause the vehicle to stop |

## Node parameters

| Parameter               | Type   | Description                                                                         |
| ----------------------- | ------ | ----------------------------------------------------------------------------------- |
| `launch_blind_spot`     | bool   | whether to launch blind_spot module                                                 |
| `launch_crosswalk`      | bool   | whether to launch crosswalk module                                                  |
| `launch_detection_area` | bool   | whether to launch detection_area module                                             |
| `launch_intersection`   | bool   | whether to launch intersection module                                               |
| `launch_traffic_light`  | bool   | whether to launch traffic light module                                              |
| `launch_stop_line`      | bool   | whether to launch stop_line module                                                  |
| `launch_occlusion_spot` | bool   | whether to launch occlusion_spot module                                             |
| `forward_path_length`   | double | forward path length                                                                 |
| `backward_path_length`  | double | backward path length                                                                |
| `max_accel`             | double | (to be a global parameter) max acceleration of the vehicle                          |
| `delay_response_time`   | double | (to be a global parameter) delay time of the vehicle's response to control commands |

## Modules

### Stop Line

#### Role

This module plans velocity so that the vehicle can stop right before stop lines and restart driving after stopped.

#### Module Parameters

| Parameter         | Type   | Description                                                                                    |
| ----------------- | ------ | ---------------------------------------------------------------------------------------------- |
| `stop_margin`     | double | a margin that the vehicle tries to stop before stop_line                                       |
| `stop_check_dist` | double | when the vehicle is within `stop_check_dist` from stop_line and stopped, move to STOPPED state |

#### Flowchart

```plantuml
@startuml
title modifyPathVelocity
start

:find collision between path and stop_line;

if (collision is found?) then (yes)
else (no)
  stop
endif

:find offset segment;

:calculate stop pose;

:calculate distance to stop line;

if (state is APPROACH) then (yes)
  :set stop velocity;

  if (vehicle is within stop_check_dist?) then (yes)
    if (vehicle is stopped?) then (yes)
      :change state to STOPPED;
    endif
  endif
else if (state is STOPPED) then (yes)
  if (vehicle started to move?) then (yes)
    :change state to START;
  endif
else if (state is START) then (yes)
  if ([optional] far from stop line?) then (yes)
    :change state to APPROACH;
  endif
endif

stop
@enduml
```

This algorithm is based on `segment`.
`segment` consists of two node points. It's useful for removing boundary conditions because if `segment(i)` exists we can assume `node(i)` and `node(i+1)` exist.

![node_and_segment](./docs/stop_line/node_and_segment.drawio.svg)

First, this algorithm finds a collision between reference path and stop line.
Then, we can get `collision segment` and `collision point`.

![find_collision_segment](./docs/stop_line/find_collision_segment.drawio.svg)

Next, based on `collision point`, it finds `offset segment` by iterating backward points up to a specific offset length.
The offset length is `stop_margin`(parameter) + `base_link to front`(to adjust head pose to stop line).
Then, we can get `offset segment` and `offset from segment start`.

![find_offset_segment](./docs/stop_line/find_offset_segment.drawio.svg)

After that, we can calculate a offset point from `offset segment` and `offset`. This will be `stop_pose`.

![calculate_stop_pose](./docs/stop_line/calculate_stop_pose.drawio.svg)



### CrossWalk

#### Role

Judgement whether a vehicle can go into a crosswalk and plan a velocity of the start/stop.

#### Launch Timing

Launches when there is a crosswalk on the target lane.

#### Module Parameters

| Parameter                                                | Type   | Description                                                              |
| -------------------------------------------------------- | ------ | ------------------------------------------------------------------------ |
| `crosswalk/stop_line_distance`                           | double | [m] make stop line away from crosswalk when no explicit stop line exists |
| `crosswalk/stop_margin`                                  | double | [m] a margin that the vehicle tries to stop before stop_line             |
| `crosswalk/slow_margin`                                  | bool   | [m] a margin that the vehicle tries to slow down before stop_line        |
| `crosswalk/slow_velocity`                                | double | [m] a slow down velocity                                                 |
| `crosswalk/stop_predicted_object_prediction_time_margin` | double | [s] time margin for decision of ego vehicle to stop or not               |
| `walkway/stop_line_distance`                             | double | [m] make stop line away from crosswalk when no explicit stop line exists |
| `walkway/stop_margin`                                    | double | [m] a margin that the vehicle tries to stop before walkway               |
| `walkway/stop_duration_sec`                              | double | [s] time margin for decision of ego vehicle to stop                      |

#### Flowchart

flow chart is almost the same as stop line.

### Detection Area

#### Role

If pointcloud is detected in a detection area defined on a map, the stop planning will be executed at the predetermined point.

![brief](./docs/detection_area/detection_area.svg)

#### Launch Timing

Launches if there is a detection area on the target lane.

### Algorithm

1. Gets a detection area and stop line from map information and confirms if there is pointcloud in the detection area
2. Inserts stop point l[m] in front of the stop line
3. Inserts a pass judge point to a point where the vehicle can stop with a max deceleration
4. Sets velocity as zero behind the stop line when the ego-vehicle is in front of the pass judge point
5. If the ego vehicle has passed the pass judge point already, it doesn’t stop and pass through.

#### Module Parameters

| Parameter             | Type   | Description                                                                                        |
| --------------------- | ------ | -------------------------------------------------------------------------------------------------- |
| `stop_margin`         | double | [m] a margin that the vehicle tries to stop before stop_line                                       |
| `use_dead_line`       | bool   | [-] weather to use dead line or not                                                                |
| `dead_line_margin`    | double | [m] ignore threshold that vehicle behind is collide with ego vehicle or not                        |
| `use_pass_judge_line` | bool   | [-] weather to use pass judge line or not                                                          |
| `state_clear_time`    | double | [s] when the vehicle is stopping for certain time without incoming obstacle, move to STOPPED state |

#### Flowchart

```plantuml
@startuml
title modifyPathVelocity
start

:get obstacle point cloud in detection area;

if (no obstacle point cloud in detection area?) then (yes)
else (no)
  :set last time obstacle found;
endif

:get clear stop state duration;

if (clear stop state duration is more than state_clear_time?) then (yes)
  :set current state GO;
  :reset clear stop state duration;
  stop
else (no)
endif

if (use dead line?) then (yes)
  :create dead line point;
  if (Is there dead line point?) then (yes)
    if (Is over dead line point?) then (yes)
      stop
    endif
  endif
endif

:calculate stop point;

if (stop point?) then (yes)
else (no)
  stop
endif


if (state is not stop and ego vehicle over line?) then (yes)
  stop
endif

if (use pass judge line?) then (yes)
  if (state is not STOP and not enough braking distance?) then (yes)
    stop
  endif
endif

:set state STOP;

:inset stop point;

:append stop reason and stop factor;

stop
@enduml
```

### Blind Spot

#### Role

Blind spot check while turning right/left by a dynamic object information, and planning and planning of a velocity of the start/stop.

![brief](./docs/blind_spot/blind_spot.svg)

### Definition

Sets a stop line, a pass judge line, a detection area and conflict area based on a map information and a self position.

- Stop line : Automatically created based on crossing lane information.

- Pass judge line : A position to judge if stop or not to avoid a rapid brake.

- Detection area : Right/left side area of the self position.

- Conflict area : Right/left side area from the self position to the stop line.

#### Module Parameters

| Parameter                       | Type   | Description                                                                 |
| ------------------------------- | ------ | --------------------------------------------------------------------------- |
| `stop_line_margin`              | double | [m] a margin that the vehicle tries to stop before stop_line                |
| `backward_length`               | double | [m] distance from closest path point to the edge of beginning point.        |
| `ignore_width_from_center_line` | double | [m] ignore threshold that vehicle behind is collide with ego vehicle or not |
| `max_future_movement_time`      | double | [s] maximum time for considering future movement of object                  |

#### Flowchart

```plantuml
@startuml
title modifyPathVelocity
start

if (ego is turning right or left ?) then (yes)
else (no)
  stop
endif

:calculate pass judge Line;

if (ego vehicle is not after pass judge line?) then (yes)
else (no)
  stop
endif

:check obstacle in blind spot;

if (obstacle found in blind spot?) then (yes)
  :set current state as STOP;
else (no)
  :set current state as GO;
endif

:set state with margin time;

if (current state is same as previous state) then (yes)
  :reset timer;
else if (state is GO->STOP) then (yes)
  :set state as STOP;
  :reset timer;
else if (state is STOP -> GO) then (yes)
  if (start time is not set) then (yes)
    :set start time;
  else(no)
   :calculate duration;
   if(duration is more than margin time)then (yes)
    :set state GO;
    :reset timer;
   endif
  endif
endif


if (state is STOP) then (yes)
  :set stop velocity;

  :set stop reason and factor;
endif

stop
@enduml
```

### Occlusion Spot

#### Role

This module plans safe velocity to slow down before reaching collision point that hidden object is darting out from `occlusion spot` where driver can't see clearly because of obstacles.

![brief](./docs/occlusion_spot/occlusion_spot.svg)

#### Occlusion Spot Private

This module only works in private road and use occupancy grid map to detect occlusion spots.

#### Occlusion Spot Public

This module only works in public road and use dynamic objects to detect occlusion spots.

Considering all occupancy grid cells inside focus range requires a lot of computation cost, so this module ignores to search farther occlusion spot which is longitudinally or laterally slice once occlusion spot is found.

![brief](./docs/occlusion_spot/sidewalk_slice.svg)

##### Definition

This module insert safe velocity at collision point and show virtual wall at intersection below.

![brief](./docs/occlusion_spot/possible_collision_info.svg)

#### Module Parameters

| Parameter            | Type   | Description                                                               |
| -------------------- | ------ | ------------------------------------------------------------------------- |
| `pedestrian_vel`     | double | [m/s] maximum velocity assumed pedestrian coming out from occlusion point |
| `safety_time_buffer` | double | [m/s] time buffer for the system delay                                    |

| Parameter /threshold    | Type   | Description                                               |
| ----------------------- | ------ | --------------------------------------------------------- |
| `detection_area_length` | double | [m] the length of path to consider occlusion spot         |
| `stuck_vehicle_vel`     | double | [m/s] velocity below this value is assumed to stop        |
| `lateral_distance`      | double | [m] maximum lateral distance to consider hidden collision |

| Parameter /(public or private)\_road | Type   | Description                                                          |
| ------------------------------------ | ------ | -------------------------------------------------------------------- |
| `min_velocity`                       | double | [m/s] minimum velocity to ignore occlusion spot                      |
| `ebs_decel`                          | double | [m/s^2] maximum deceleration to assume for emergency braking system. |
| `pbs_decel`                          | double | [m/s^2] deceleration to assume for predictive braking system         |

| Parameter /sidewalk       | Type   | Description                                                     |
| ------------------------- | ------ | --------------------------------------------------------------- |
| `min_occlusion_spot_size` | double | [m] the length of path to consider occlusion spot               |
| `focus_range`             | double | [m] buffer around the ego path used to build the sidewalk area. |

| Parameter /grid  | Type   | Description                                                     |
| ---------------- | ------ | --------------------------------------------------------------- |
| `free_space_max` | double | [-] maximum value of a free space cell in the occupancy grid    |
| `occupied_min`   | double | [-] buffer around the ego path used to build the sidewalk area. |

#### Flowchart

```plantuml
@startuml
title modifyPathVelocity (Private/Public) Road
start

:get current road type;

if (road type is PUBLIC) then (yes)
  :use dynamic object array info;
else if (road type is PRIVATE) then (yes)
  :use occupancy grid map info;
else (no)
  stop
endif

:generate possible collision;

:find possible collision between path and occlusion spot;

if (possible collision is found?) then (yes)
else (no)
  stop
endif

:calculate collision path point;

:calculate safe velocity consider lateral distance and safe velocity;

:insertSafeVelocityToPath;

stop
@enduml
```

### No Stopping Area

#### Role

This module plans to avoid stop in 'no stopping area`.

![brief](./docs/no_stopping_area/NoStoppingArea.svg)

#### ModelParameter

| Parameter                    | Type   | Description                                                         |
| ---------------------------- | ------ | ------------------------------------------------------------------- |
| `state_clear_time`           | double | [s] time to clear stop state                                        |
| `stuck_vehicle_vel_thr`      | double | [m/s] vehicles below this velocity are considered as stuck vehicle. |
| `stop_margin`                | double | [m] margin to stop line at no stopping area                         |
| `dead_line_margin`           | double | [m] if ego pass this position GO                                    |
| `stop_line_margin`           | double | [m] margin to auto-gen stop line at no stopping area                |
| `detection_area_length`      | double | [m] length of searching polygon                                     |
| `stuck_vehicle_front_margin` | double | [m] obstacle stop max distance                                      |

#### Flowchart

```plantuml
@startuml
title modifyPathVelocity
start

if (ego path has "no stopping area" ?) then (yes)
else (no)
  stop
endif

partition pass_through_condition {
if (ego vehicle is not after dead line?) then (yes)
else (no)
  stop
endif
if (ego vehicle is stoppable before stop line consider jerk limit?) then (yes)
else (no)
  stop
endif
}
note right
  - ego vehicle is already over dead line(1.0[m] forward stop line) Do Not Stop.
  - "pass through or not" considering jerk limit is judged only once to avoid chattering.
end note

:generate ego "stuck_vehicle_detect_area" polygon;
note right
"stuck_vehicle_detect_area" polygon includes space of
 vehicle_length + obstacle_stop_max_distance
 after "no stopping area"
end note

:generate ego "stop_line_detect_area" polygon;
note right
"stop_line_detect_area" polygon includes space of
 vehicle_length + margin
 after "no stopping area"
end note

:set current judgement as GO;
if (Is stuck vehicle inside "stuck_vehicle_detect_area" polygon?) then (yes)
note right
only consider stuck vehicle following condition.
- below velocity 3.0 [m/s]
- semantic type of car bus truck or motorbike
only consider stop line as following condition.
- low velocity that is in path with lane id is considered.
end note
if (Is stop line inside "stop_line_detect_area" polygon?) then (yes)
  :set current judgement as STOP;
endif
endif

partition set_state_with_margin_time {

if (current judgement is same as previous state) then (yes)
  :reset timer;
else if (state is GO->STOP) then (yes)
  :set state as STOP;
  :reset timer;
else if (state is STOP -> GO) then (yes)
  if (start time is not set) then (yes)
    :set start time;
  else(no)
   :calculate duration;
   if(duration is more than margin time)then (yes)
    :set state GO;
    :reset timer;
  else(no)
   endif
  endif
else(no)
endif

}

note right
  - it takes 2 seconds to change state from STOP -> GO
  - it takes 0 seconds to change state from GO -> STOP
  - reset timer if no state change
end note

if (state is STOP) then (yes)
  :set stop velocity;
  :set stop reason and factor;
  else(no)
endif
stop


@enduml
```
