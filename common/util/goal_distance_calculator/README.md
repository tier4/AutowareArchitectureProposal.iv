# goal_distance_calculator

## Usage

```sh
# cyclic
roslaunch goal_distance_calculator goal_distance_calculator.launch
rosrun goal_distance_calculator goal_distance_calculator_node _oneshot:=false # or, use rosrun

# oneshot
rosrun goal_distance_calculator goal_distance_calculator_node
rosrun goal_distance_calculator goal_distance_calculator_node _oneshot:=true # or, explicitly specify oneshot parameter
```

This package also contains some utilitis for setting a goal position.

- Generate goal-publish-command from `current goal`

```sh
$ rosrun goal_distance_calculator current_goal2goal_pub_cmd
rostopic pub /planning/mission_planning/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 3793.77303283
    y: 73762.1491577
    z: 1.00100634995
  orientation:
    x: 0.0
    y: 0.0
    z: 0.254885328566
    w: 0.966971286689"
```

- Generate goal-publish-command from `current pose`

```sh
$ rosrun goal_distance_calculator current_pose2goal_pub_cmd
rostopic pub /planning/mission_planning/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 3780.00698161
    y: 73755.3040841
    z: -0.474905626495
  orientation:
    x: 1.22935737252e-05
    y: 7.96361049582e-05
    z: 0.218644126932
    w: 0.975804662453"
```
