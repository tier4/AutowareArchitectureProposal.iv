# dummy_infrastructure

This is a debug node for infrastructure communication.

## Usage

```sh
ros2 launch dummy_infrastructure dummy_infrastructure.launch.xml
ros2 run rqt_reconfigure rqt_reconfigure
```

## Inputs / Outputs

### Inputs

| Name                    | Type                                                 | Description            |
| ----------------------- | ---------------------------------------------------- | ---------------------- |
| `~/input/command_array` | `autoware_v2x_msgs::msg::InfrastructureCommandArray` | Infrastructure command |

### Outputs

| Name                   | Type                                                    | Description                 |
| ---------------------- | ------------------------------------------------------- | --------------------------- |
| `~/output/state_array` | `autoware_v2x_msgs::msg::VirtualTrafficLightStateArray` | Virtual traffic light array |

## Parameters

### Node Parameters

| Name                | Type   | Default Value | Explanation                                       |
| ------------------- | ------ | ------------- | ------------------------------------------------- |
| `update_rate`       | int    | `10`          | Timer callback period [Hz]                        |
| `use_first_command` | bool   | `true`        | Use first command or not                          |
| `instrument_id`     | string | ``            | Used as command id                                |
| `approval`          | bool   | `false`       | Has right of way or not                           |
| `is_finalized`      | bool   | `false`       | Stop at stop_line if finalization isn't completed |

## Assumptions / Known limits

TBD.
