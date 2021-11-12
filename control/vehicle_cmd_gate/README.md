# Vehicle Cmd Gate

## Purpose

## Inputs / Outputs

### Input

| Name                                       | Type                                                       | Description                  |
| ------------------------------------------ | ---------------------------------------------------------- | ---------------------------- |
| `/input/emergency_state`                   | `autoware_auto_system_msgs::msg::EmergencyState`           | Control command for vehicle. |
| `/input/steering`                          | `autoware_auto_vehicle_msgs::msg::SteeringReport`          | Control command for vehicle. |
| `/input/auto/control_cmd`                  | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `/input/auto/turn_indicators_cmd`          | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`   | Control command for vehicle. |
| `/input/auto/hazard_lights_cmd`            | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | Control command for vehicle. |
| `/input/auto/gear_cmd`                     | `autoware_auto_vehicle_msgs::msg::GearCommand`             | Control command for vehicle. |
| `/input/external/control_cmd`              | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `/input/external/turn_indicators_cmd`      | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`   | Control command for vehicle. |
| `/input/external/hazard_lights_cmd`        | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | Control command for vehicle. |
| `/input/external/gear_cmd`                 | `autoware_auto_vehicle_msgs::msg::GearCommand`             | Control command for vehicle. |
| `/input/external_emergency_stop_heartbeat` | `autoware_external_api_msgs::msg::Heartbeat`               | Control command for vehicle. |
| `/input/gate_mode`                         | `autoware_control_msgs::msg::GateMode`                     | Control command for vehicle. |
| `/input/emergency/control_cmd`             | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `/input/emergency/hazard_lights_cmd`       | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | Control command for vehicle. |
| `/input/emergency/gear_cmd`                | `autoware_auto_vehicle_msgs::msg::GearCommand`             | Gate mode (AUTO or EXTERNAL) |
| `/input/engage`                            | `autoware_auto_vehicle_msgs::msg::Engage`                  | Control command for vehicle. |

### Output

| Name                                   | Type                                                       | Description                  |
| -------------------------------------- | ---------------------------------------------------------- | ---------------------------- |
| `/output/vehicle_cmd_emergency`        | `autoware_auto_system_msgs::msg::EmergencyState`           | Control command for vehicle. |
| `/output/external/control_cmd`         | `autoware_auto_control_msgs::msg::AckermannControlCommand` | Control command for vehicle. |
| `/output/external/turn_indicators_cmd` | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`   | Control command for vehicle. |
| `/output/external/hazard_lights_cmd`   | `autoware_auto_vehicle_msgs::msg::HazardLightsCommand`     | Control command for vehicle. |
| `/output/external/gear_cmd`            | `autoware_auto_vehicle_msgs::msg::GearCommand`             | Control command for vehicle. |
| `/output/gate_mode`                    | `autoware_control_msgs::msg::GateMode`                     | Gate mode (AUTO or EXTERNAL) |
| `/output/engage`                       | `autoware_auto_vehicle_msgs::msg::Engage`                  | Engage signal                |
| `/output/external_emergency`           | `autoware_external_api_msgs::msg::Emergency`               | Control command for vehicle. |

## Parameters

| Parameter                                    | Type   | Description                        |
| -------------------------------------------- | ------ | ---------------------------------- |
| `update_period`                              | bool   | true when subscribed path exists   |
| `use_emergency_handling_`                    | bool   | true when subscribed path exists   |
| `use_external_emergency_stop_`               | bool   | true when subscribed path exists   |
| `system_emergency_heartbeat_timeout_`        | bool   | true when subscribed path exists   |
| `external_emergency_stop_heartbeat_timeout_` | bool   | true when subscribed path exists   |
| `stop_hold_acceleration_`                    | bool   | true when subscribed path exists   |
| `emergency_acceleration_`                    | bool   | true when subscribed path exists   |
| `vel_lim`                                    | double | limit of longitudinal velocity     |
| `lon_acc_lim`                                | bool   | limit of longitudinal acceleration |
| `lon_jerk_lim`                               | bool   | limit of longitudinal jerk         |
| `lat_acc_lim`                                | bool   | limit of lateral acceleration      |
| `lat_jerk_lim`                               | bool   | limit of lateral jerk              |

## Assumptions / Known limits

TBD.
