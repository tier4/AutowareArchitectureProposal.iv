# AWAPI_AWIV_ADAPTER

✓: confirmed, (blank): not confirmed

## get topic

### /awapi/vehicle/get/status

- get vehicle status
- MessageType: awapi_awiv_adapter/AwapiVehicleStatus

| ✓   | type                      | name                     | unit                                          | note                                     |
| --- | :------------------------ | :----------------------- | :-------------------------------------------- | :--------------------------------------- |
| ✓   | std_msgs/Header           | header                   |                                               |                                          |
| ✓   | geometry_msgs/Pose        | pose                     | position:[m]                                  |                                          |
| ✓   | awapi_awiv_adapter/Euler  | eulerangle               | [rad]                                         | roll/pitch/yaw                           |
|     | awapi_awiv_adapter/Latlon | latlon                   |                                               | lat/lon/alt                              |
| ✓   | float64                   | velocity                 | [m/s]                                         |                                          |
| ✓   | float64                   | acceleration             | [m/ss]                                        | calculate from velocity in awapi_adapter |
| ✓   | float64                   | steering                 | [rad]                                         |                                          |
| ✓   | float64                   | steering_velocity        | [rad/s]                                       | calculate from steering in awapi_adapter |
| ✓   | float64                   | angular_velocity         | [rad/s]                                       |                                          |
|     | int32                     | gear                     | accroding to autoware_vehicle_msgs/Shift      |                                          |
|     | float32                   | energy_level             |                                               | available only for golf-cart             |
| ✓   | int32                     | turn_signal              | accroding to autoware_vehicle_msgs/TurnSignal |                                          |
| ✓   | float64                   | target_velocity          | [m/s]                                         |                                          |
| ✓   | float64                   | target_acceleration      | [m/ss]                                        |                                          |
| ✓   | float64                   | target_steering          | [rad]                                         |                                          |
| ✓   | float64                   | target_steering_velocity | [rad/s]                                       |                                          |

### /awapi/autoware/get/status

- get autoware status
- MessageType: awapi_awiv_adapter/AwapiVehicleStatus

| ✓   | type                                   | name              | unit                                           | note                                                                                                                                                                        |
| --- | :------------------------------------- | :---------------- | :--------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✓   | std_msgs/Header                        | header            |                                                |                                                                                                                                                                             |
| ✓   | string                                 | autoware_state    |                                                |                                                                                                                                                                             |
| ✓   | int32                                  | control_mode      | accroding to autoware_vehicle_msgs/ControlMode | manual/auto (changed by /awapi/autoware/put/engage)                                                                                                                         |
|     | int32                                  | gate_mode         | autoware_vehicle_msgs/GateMode                 | auto/remote (it is valid only when control_mode=auto))                                                                                                                      |
| ✓   | bool                                   | emergency_stopped | True in emergency mode                         |                                                                                                                                                                             |
| ✓   | autoware_planning_msgs/StopReasonArray | stop_reason       |                                                | "stop_pose" represents the position of "base_link" (not the head of the car)                                                                                                |
| ✓   | diagnostic_msgs/DiagnosticStatus[]     | diagnostics       |                                                | output only diag. of leaf node (diag. of parent node are cut)                                                                                                               |
| ✓   | bool                                   | arrived_goal      |                                                | True if the autoware_state is changed from Driving to ArrivedGoal or WairingForRoute. False if the autoware_state is changed to WaitingForEngage or Driving. Default False. |

- specification of stop_reason
  - stop_reason is output only when the following conditions are met.
    - stop_point in stop_reason is close to /planning/scenario_planning/trajectory (within 10m).
    - The distance between current position and stop_point is less than stop_reason_thresh_dist.

### /awapi/autoware/get/route

- get route
- MessageType: autoware_planning_msgs/Route

| ✓   | type                         | name | unit | note |
| --- | :--------------------------- | :--- | :--- | :--- |
| ✓   | autoware_planning_msgs/Route |      |      |      |

### /awapi/prediction/get/objects

- get predicted object
- MessageType: autoware_perception_msgs/DynamicObjectArray

| ✓   | type                                        | name | unit | note |
| --- | :------------------------------------------ | :--- | :--- | :--- |
| ✓   | autoware_perception_msgs/DynamicObjectArray |      |      |      |

### /awapi/lane_change/get/status

- get lane change information
- MessageType: awapi_awiv_adapter/LaneChangeStatus

| ✓   | type                        | name                        | unit                                     | note                                                                               |
| --- | :-------------------------- | :-------------------------- | :--------------------------------------- | :--------------------------------------------------------------------------------- |
|     | std_msgs/Header             | header                      |                                          |                                                                                    |
|     | bool                        | force_lane_change_available | True when lane change is avilable        | available: Physically lane changeable state (do not consider other vehicle)        |
|     | bool                        | lane_change_ready           | True when lane change is ready           | ready: State that ego-vehicle can change lane without collision with other vehicle |
|     | autoware_planning_msgs/Path | candidate_path              | according to autoware_planning_msgs/Path |                                                                                    |

### /awapi/object_avoidance/get/status

- get obstacle avoidance information
- MessageType: awapi_awiv_adapter/ObstacleAvoidanceStatus

| ✓   | type                              | name                     | unit                                           | note                                                  |
| --- | :-------------------------------- | :----------------------- | :--------------------------------------------- | :---------------------------------------------------- |
|     | std_msgs/Header                   | header                   |                                                |                                                       |
|     | bool                              | obstacle_avoidance_ready | True when obstacle avoidance is ready          |                                                       |
|     | autoware_planning_msgs/Trajectory | candidate_path           | according to autoware_planning_msgs/Trajectory | Msg type is different from lane change candidate path |

### /awapi/traffic_light/get/status

- get recognition result of traffic light
- MessageType: autoware_perception_msgs/TrafficLightStateArray

| ✓   | type                                            | name | unit | note |
| --- | :---------------------------------------------- | :--- | :--- | :--- |
|     | autoware_perception_msgs/TrafficLightStateArray |      |      |      |

## put topic

### /awapi/vehicle/put/velocity

- set upper velocity
- MessageType: std_msgs/Float32

| ✓   | type             | name | unit | note         |
| --- | :--------------- | :--- | :--- | :----------- |
| ✓   | std_msgs/Float32 |      |      | max velocity |

### /awapi/vehicle/put/stop

- set temporary stop signal
- MessageType: std_msgs/bool
- Specification

  - send True: send upper velocity to 0
  - send False: resend last received upper velocity
    - (if upper velocity have never received, send _default_max_velocity_ value.)
    - _default_max_velocity_ refers to the param: _/planning/scenario_planning/motion_velocity_optimizer/max_velocity_

  | ✓   | type          | name | unit | note |
  | --- | :------------ | :--- | :--- | :--- |
  | ✓   | std_msgs/Bool |      |      |      |

### /awapi/autoware/put/gate_mode

- send gate mode (auto/remote)
- MessageType: autoware_control_msgs/GateMode

| ✓   | type                           | name | unit | note |
| --- | :----------------------------- | :--- | :--- | :--- |
|     | autoware_control_msgs/GateMode |      |      |      |

### /awapi/autoware/put/emergency

- send emergency signal
- MessageType: autoware_control_msgs/EmergencyMode
- <font color="Cyan">**To enable this functionality, autoware have to be in the Remote Mode or set _/control/vehicle_cmd_gate/use_emergency_handling_ to true.**</font>

| ✓   | type                                | name | unit | note |
| --- | :---------------------------------- | :--- | :--- | :--- |
| ✓   | autoware_control_msgs/EmergencyMode |      |      |      |

### /awapi/autoware/put/engage

- send engage signal (both of autoware/engage and vehicle/engage)
- MessageType: autoware_control_msgs/EngageMode

| ✓   | type                             | name | unit | note |
| --- | :------------------------------- | :--- | :--- | :--- |
| ✓   | autoware_control_msgs/EngageMode |      |      |      |

### /awapi/autoware/put/route

- send goal pose
- MessageType: autoware_planning_msgs/Route

| ✓   | type                         | name | unit | note |
| --- | :--------------------------- | :--- | :--- | :--- |
| ✓   | autoware_planning_msgs/Route |      |      |      |

### /awapi/lane_change/put/approval

- send lane change approval flag
- send True: start lane change when **lane_change_ready** is true
- MessageType: std_msgs/Bool

| ✓   | type          | name | unit | note |
| --- | :------------ | :--- | :--- | :--- |
|     | std_msgs/Bool |      |      |      |

### /awapi/lane_change/put/force

- send force lane change flag
- send True: start lane change when **force_lane_change_available** is true
- MessageType: std_msgs/Bool

| ✓   | type          | name | unit | note |
| --- | :------------ | :--- | :--- | :--- |
|     | std_msgs/Bool |      |      |      |

### /awapi/object_avoidance/put/approval

- send object avoidance approval flag
- MessageType: std_msgs/Bool

| ✓   | type          | name | unit | note |
| --- | :------------ | :--- | :--- | :--- |
|     | std_msgs/Bool |      |      |      |

### /awapi/object_avoidance/put/force

- send force object avoidance flag
- <font color="Red">**not implemented (Autoware does not have corresponded topic)**</font>

| ✓   | type | name | unit | note |
| --- | :--- | :--- | :--- | :--- |


### /awapi/traffic_light/put/traffic_light

- Overwrite the recognition result of traffic light
- <font color="Red">**not implemented (Autoware does not have corresponded topic)**</font>

| ✓   | type | name | unit | note |
| --- | :--- | :--- | :--- | :--- |

