# Autoware Vehicle Rviz Plugin

This package is including jsk code.  
Note that jsk_overlay_utils.cpp and jsk_overlay_utils.hpp are BSD license.
## Purpose
This plugin provides a visual and easy-to-understand display of vehicle speed, turn signal and steering status.

## Inputs / Outputs
### Input
| Name                              | Type                                                  | Description                                       |
| --------------------------------- | ----------------------------------------------------- | ------------------------------------------------- |
| `/input/twist` | `geometry_msgs::msg::TwistStamped`             | The topic is vehicle twist |
| `/input/turn_signal_cmd` | `autoware_auto_vehicle_msgs::msg::SteeringReport`             | The topic is status of turn signal |
| `/input/steering` | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport`             | The topic is status of steering |
## Parameter
### Core Parameters
| Name          | Type   | Default Value | Description                 |
| ------------- | ------ | ------------- | --------------------------- |
| `property_text_color_` | QColor | QColor(25, 255, 240)          | Text color |
| `property_left_` | int | 128          | Left of the plotter window [px] |
| `property_top_` | int | 128          | Top of the plotter window [px] |
| `property_width_` | int | 128          | Left of the plotter window [px] |
| `property_height_` | int | 128          | Width of the plotter window [px] |
| `property_length_` | int | 256          | Height of the plotter window [px] |
| `property_value_height_offset_` | int | 0          | Height offset of the plotter window [px] |
| `property_value_scale_` | float | 1.0 / 6.667          | Value scale |
| `property_handle_angle_scale_` | float | 3.0          | Scale is steering angle to handle angle |
| `velocity_` | float | 0.0          | Velocity [m/s] |
| `property_velocity_timeout_` | float | 10.0          | Timeout of velocity [s] |
| `property_velocity_alpha_` | float | 1.0          | Alpha of velocity |
| `property_velocity_scale_` | float | 0.3          | Scale of velocity |
| `property_velocity_color_view_` | bool | false          | Use Constant Color or not |
| `property_velocity_color_` | QColor | Qt::black          | Color of velocity history |
| `property_vel_max_` | float | 3.0          | Color Border Vel Max [m/s] |


## Assumptions / Known limits
Using a planning simulator

## Usage
1. Start rviz and select Add under the Displays panel.  

![select_add](./images/select_add.png)

2. Select any one of the autoware_vehicle_rviz_plugin and press OK.  

![select_planning_plugin](./images/select_planning_plugin.png)

3. Select the new item in the tool tab (2D Dummy Car in the example) and click on it in rviz.  

![select_dummy_car](./images/select_dummy_car.png)
