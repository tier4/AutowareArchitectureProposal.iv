# dummy_diag_publisher

## Purpose

This package outputs a dummy diag for debugging and developing.

## Inputs / Outputs

### Outputs

| Name               | Type                                    | Description                                                                                                                                                                                |
| ------------------ | --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `/diagnostics`     | `diagnostic_msgs/DiagnosticArray`       | Diagnostics outputs                                                                                                                                                                        |
| `/diagnostics_agg` | `diagnostic_msgs::msg::DiagnosticArray` | Use aggregated diagnostic information based on [diagnostic_aggregator setting](https://github.com/tier4/autoware.iv/tree/main/system/autoware_error_monitor/config/diagnostic_aggregator). |
| `/diagnostics_err` | `diagnostic_msgs::msg::DiagnosticArray` | This information is used to visualize HazardStatus.                                                                                                                                        |

## Parameters

### Node Parameters

| Name          | Type | Default Value | Explanation            |
| ------------- | ---- | ------------- | ---------------------- |
| `update_rate` | int  | `10`          | Timer callback period. |

## Assumptions / Known limits

TBD.

## Usage

```sh
ros2 launch dummy_diag_publisher dummy_diag_publisher.launch.xml
```
