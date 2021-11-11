# autoware_joy_controller

## Role

`autoware_joy_controller` operate a vehicle by a joy controller via Autoware (e.g. steering wheel, shift, turn signal, engage) by several kinds of joy sticks.

## Input / Output

### Input topics

| Name               | Type                    | Description                       |
| ------------------ | ----------------------- | --------------------------------- |
| `~/input/joy`      | sensor_msgs::msg::Joy   | joy controller command            |
| `~/input/odometry` | nav_msgs::msg::Odometry | ego vehicle odometry to get twist |

### Output topics

| Name                                | Type                                                     | Description                              |
| ----------------------------------- | -------------------------------------------------------- | ---------------------------------------- |
| `~/output/control_command`          | autoware_auto_control_msgs::msg::AckermannControlCommand| lateral and longitudinal control command |
| `~/output/external_control_command` | autoware_external_api_msgs::msg::ControlCommandStamped   | lateral and longitudinal control command |
| `~/output/shift`                    | autoware_external_api_msgs::msg::GearShiftStamped        | gear command                             |
| `~/output/turn_signal`              | autoware_external_api_msgs::msg::TurnSignalStamped       | turn signal command                      |
| `~/output/gate_mode`                | autoware_control_msgs::msg::GateMode                     | gate mode (Auto or External)             |
| `~/output/heartbeat`                | autoware_external_api_msgs::msg::Heartbeat               | heartbeat                                |
| `~/output/vehicle_engage`           | autoware_auto_vehicle_msgs::msg::Engage                  | vehicle engage                           |

## Parameters

| Parameter                 | Type   | Description                                                                                                        |
| ------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------ |
| `joy_type`                | string | joy controller type (default: DS4)                                                                                 |
| `update_rate`             | double | update rate to publish control commands                                                                            |
| `accel_ratio`             | double | ratio to calculate acceleration (commanded acceleration is ratio \* operation)                                     |
| `brake_ratio`             | double | ratio to calculate deceleration (commanded acceleration is -ratio \* operation)                                    |
| `steer_ratio`             | double | ratio to calculate deceleration (commanded steer is ratio \* operation)                                            |
| `steering_angle_velocity` | double | steering angle velocity for operation                                                                              |
| `accel_sensitivity`       | double | sensitivity to calculate acceleration for external API (commanded acceleration is pow(operation, 1 / sensitivity)) |
| `brake_sensitivity`       | double | sensitivity to calculate deceleration for external API (commanded acceleration is pow(operation, 1 / sensitivity)) |
| `velocity_gain`           | double | ratio to calculate velocity by acceleration                                                                        |
| `max_forward_velocity`    | double | absolute max velocity to go forward                                                                                |
| `max_backward_velocity`   | double | absolute max velocity to go backward                                                                               |
| `backward_accel_ratio`    | double | ratio to calculate deceleration (commanded acceleration is -ratio \* operation)                                    |
