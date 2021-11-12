# Gyro Odometer

## Purpose

`vehicle_cmd_gate` is the package to get information from emergency handler and planning module and external controller and send a msg to vehicle.

## Inputs / Outputs

### Input

| Name                            | Type                                                       | Description                                  |
| ------------------------------- | ---------------------------------------------------------- | -------------------------------------------- |
| `vehicle/twist`                 | `autoware_auto_vehicle_msgs::msg::SteeringReport`          | steering status                              |
| `vehicle/twist_with_covariance` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | control command from planning module         |
| `imu`                           | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand`   | turn indicators command from planning module |

### Output

| Name                    | Type                                                       | Description                                             |
| ----------------------- | ---------------------------------------------------------- | ------------------------------------------------------- |
| `twist`                 | `autoware_auto_system_msgs::msg::EmergencyState`           | emergency state which was originally in vehicle command |
| `twist_with_covariance` | `autoware_auto_control_msgs::msg::AckermannControlCommand` | gear command to vehicle                                 |

## Parameters

| Parameter                                    | Type   | Description                                                    |
| -------------------------------------------- | ------ | -------------------------------------------------------------- |
| `output_frame`                               | bool   | true when subscribed path exists                               |
| `use_emergency_handling_`                    | bool   | true when emergency handler is used                            |
| `use_external_emergency_stop_`               | bool   | true when external emergency stop information is used          |
| `system_emergency_heartbeat_timeout_`        | bool   | timeout for system emergency                                   |
| `external_emergency_stop_heartbeat_timeout_` | bool   | timeout for external emergency                                 |
| `stop_hold_acceleration_`                    | double | longitudinal acceleration cmd when vehicle should stop         |
| `emergency_acceleration_`                    | bool   | longitudinal acceleration cmd when vehicle stop with emergency |
| `vel_lim`                                    | double | limit of longitudinal velocity                                 |
| `lon_acc_lim`                                | bool   | limit of longitudinal acceleration                             |
| `lon_jerk_lim`                               | bool   | limit of longitudinal jerk                                     |
| `lat_acc_lim`                                | bool   | limit of lateral acceleration                                  |
| `lat_jerk_lim`                               | bool   | limit of lateral jerk                                          |

## Assumptions / Known limits

TBD.
