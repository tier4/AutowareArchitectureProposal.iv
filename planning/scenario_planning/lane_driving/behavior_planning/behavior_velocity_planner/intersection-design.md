# Intersection

## Role

Judgement whether a vehicle can go into an intersection or not by a dynamic object information, and planning a velocity of the low-down/stop.
This module is designed for rule-based intersection velocity decision that is easy for developers to design its behavior. It generates proper velocity for intersection scene.

In addition, the external users / modules (e.g. remote operation) to can intervene the STOP/GO decision for the vehicle behavior. The override interface is expected to be used, for example, for remote intervention in emergency situations or gathering information on operator decisions during development.

![brief](./docs/intersection/intersection.svg)

## Activation Timing

This function is activated when the attention lane conflicts with the ego vehicle's lane.

## Limitations

This module allows developers to design vehicle velocity in intersection module using specific rules. This module is affected by object detection and prediction accuracy considering as stuck vehicle in this intersection module.

## Inner-workings / Algorithms

### How To Select Attention Target Objects

Objects that satisfy all of the following conditions are considered as target objects (possible collision objects):

- The type of object type is **car**, **truck**, **bus** or **motorbike**. (Bicycle, pedestrian, animal, unknown are not.)
- The center of gravity of object is **located within a certain distance** from the attention lane (threshold = `detection_area_margin`) .
  - In the past, the decision was made based on whether any points of the object's polygon exists in the attention lane, but since there were many false positives, the logic has changed to the current one.
- The posture of object is **the same direction as the attention lane** (threshold = `detection_area_angle_threshold`).
  - The orientation of the target is recalculated in this module according to the `orientation_reliable` and the sign of the velocity of the target.
- Not being **in the same lane as the ego vehicle**.
  - To avoid judging the vehicle ahead as a collision target. This logic needs to be improved.

### How to Define Attention Lanes

Target objects are limited by lanelets to prevent unexpected behavior. A lane that satisfies the following conditions is defined as an "Attention Lane" and used to define the target object.

- The lane crosses with the driving lane of the ego-vehicle
- The lane has high priority for the driving lane of the ego-vehicle (priority tags are needed to be configured in Lanelet-map according to the situation).

See the following figures to know how to create an attention area and its rationale.

![intersection_straight](docs/intersection/intersection_straight.png)

![intersection_right](docs/intersection/intersection_right.png)

![intersection_left](docs/intersection/intersection_left.png)

### Crossing Judgement

（TODO　木村）
Attention target に対して以下の処理を行い、egoとの衝突判定を行う。余裕を持って交差点を通過することができないと判断した場合は、停止線に停止速度を埋め込む

1. 自車の交差点の通過時間を計算. この通過時間を$t_s$ ~ $t_e$とする
2. 対象物体の予測経路の中でconfidenceが `min_predicted_path_confidence` 以上のものを抽出の
3. 以下の方法で抽出された予測経路とegoの予測経路の衝突検知をする
 - 自車の$t_s$ ~ $t_e$における通過領域$A_{ego}$を求める
 - 対象物体の$t_s$ - `collsiion_start_margin_time` ~ $t_e$ + `collision_end_margin_time`における通過領域$A_{target}$を各予測経路に対して求める*
 - $A_{ego}$と$A_{target}$領域が重なっていたら衝突検知.
4. 衝突検知された場合に交差点前で停止する。なお、停止解除にはマージンが存在する（下参照）

* なお. 3.の`collsiion_start_margin_time`, `collision_end_margin_time`は以下のような意味を持つ
- 自車の方が対象物体より早く交差点を通過する場合, 両者の交差点通過が`collsiion_start_margin_time`以下の時間差である場合, 衝突検知される
- 対象物体の方が自車より早く交差点を通過する場合, 両者の交差点通過が`collsiion_end_margin_time`以下の時間差である場合, 衝突検知される

### State Transition (go / stop)

If there is no crossing more than a certain period (default : 2.0[s]), the state transits to “go”. If crossing is detected even once, the state transits to “stop”.

### Stop Line Automatic Generation

The driving lane is complemented at a certain intervals (default : 20 [cm]), and the line which is a margin distance (default : 100cm) in front of the attention lane is defined as a stop line. (Also the length of the vehicle is considered and the stop point is set at the base_link point in front of the stop lane.)

### Pass Judge Line

To avoid a rapid braking, in case that a deceleration more than a threshold (default : 0.5[G]) is needed, the ego vehicle doesn’t stop. In order to judge this condition, pass judge line is set a certain distance (default : 0.5 \* v_current^2 / a_max) in front of the stop line.
To prevent a chattering, once the ego vehicle passes this line, “stop” decision in the intersection won’t be done any more.
To prevent going over the pass judge line before the traffic light stop line, the distance between stop line and pass judge line become 0m in case that there is a stop line between the ego vehicle and an intersection stop line.

### Stuck Vehicle Detection

If there is any object in a certain distance (default : 5[m]) from the end point of the intersection lane on the driving lane and the object velocity is less than a threshold (default 3.0[km/h]), the object is regarded as a stuck vehicle. If the stuck vehicle exists, the ego vehicle cannot enter the intersection.

## Module Parameters

| Parameter                                     | Type   | Description                                                                   |
| --------------------------------------------- | ------ | ----------------------------------------------------------------------------- |
| `intersection/state_transit_margin_time`      | double | [m] time margin to change state                                               |
| `intersection/decel_velocity`                 | double | [m] deceleration velocity in intersection                                     |
| `intersection/path_expand_width`              | bool   | [m] path area to see with expansion                                           |
| `intersection/stop_line_margin`               | double | [m] margin before stop line                                                   |
| `intersection/stuck_vehicle_detect_dist`      | double | [m] this should be the length between cars when they are stopped.             |
| `intersection/stuck_vehicle_ignore_dist`      | double | [m] obstacle stop max distance(5.0[m]) + stuck vehicle size / 2.0[m])         |
| `intersection/stuck_vehicle_vel_thr`          | double | [m/s] velocity below 3[km/h] is ignored by default                            |
| `intersection/intersection_velocity`          | double | [m/s] velocity to pass intersection. 10[km/h] is by default                   |
| `intersection/intersection_max_accel`         | double | [m/s^2] acceleration in intersection                                          |
| `intersection/detection_area_margin`          | double | [m] range for expanding detection area                                        |
| `intersection/detection_area_length`          | double | [m] range for lidar detection 200[m] is by default                            |
| `intersection/detection_area_angle_threshold` | double | [rad] threshold of angle difference between the detection object and lane     |
| `intersection/min_predicted_path_confidence`  | double | [-] minimum confidence value of predicted path to use for collision detection |
| `merge_from_private_road/stop_duration_sec`   | double | [s] duration to stop                                                          |

## How To Tune Parameters

- The time to change state form `Stop` to `GO` is too long.
  - Change `state_transit_margin_time` to lower value. Be careful if this margin is too small then vehicle is going to change state many times and cause chattering.
- The distance to stuck vehicle is too long.
  - Change `stuck_vehicle_detect_dist` to lower value. Note this module consider obstacle stop max distance as detection distance.
- The speed in intersection is too slow
- Change `intersection_velocity` to higher value.

## Flowchart

```plantuml
@startuml
title modifyPathVelocity
start

:get object polygon;

partition get_objective_polygon {

:get "right-of-way" lanelets from ego lanelet;

:get previous and following ego lanelet from routing graph;

:get all conflicting lanelets with ego lane;

:exclude yield and ego lanelets from conflicting lanelets;

:update conflicting and objective polygon;
}

:get external Input;

:generate stop line;

if (not generate stop line) then (yes)
  stop
else (no)
endif

if (stop line index or pass judge index is same lane as ego?) then (yes)
  stop
else (no)
endif

if (not found closest index?) then (yes)
  stop
else (no)
endif

if (state is GO and over pass judge line and no external stop) then (yes)
  stop
else (no)
endif

if (has collision or is stuck vehicle in intersection?) then (yes)
  :set is_entry_prohibited true;
  :set state from external input;
else (no)
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

if (state is STOP?) then (yes)
  :set velocity;
  if (stop required?) then (yes)
    :publish stop reason and stop factor;
  endif
else(no)
endif
stop

@enduml
```

NOTE current state is treated as `STOP` if `is_entry_prohibited` = `true` else `GO`

## Known Limits

- This module generate intersection stop line and ignoring lanelet automatically form lanelet map , however if you want to set intersection stop line and ignoring lanelet manually you need to tag `right_of_way` and `yield` to all conflicting lanes properly.

---

## How to Set Lanelet Map fot Intersection

![intersection_fig](docs/intersection/intersection_fig.png)

### Set a turn_direction tag (Fig. 1)

IntersectionModule will be activated by this tag. If this tag is not set, ego-vehicle don’t recognize the lane as an intersection. Even if it’s a straight lane, this tag is mandatory if it is located within intersection.

Set a value in `turn_direction` tag to light up turn signals.

Values of `turn_direction` must be one of “straight”(no turn signal), “right” or “left”. Autoware will light up respective turn signals 30[m] before entering the specified lane. You may also set optional tag “turn_signal_distance” to modify the distance to start lighting up turn signals.

Lanes within intersections must be defined as a single Lanelet. For example, blue lane in Fig.3 cannot be split into 2 Lanelets.

### Explicitly describe a stop position [RoadMarking] (Optional) (Fig. 2)

As a default, IntersectionModule estimates a stop position by the crossing point of driving lane and attention lane. But there are some cases like Fig.2 in which we would like to set a stop position explicitly. When a `stop_line` is defined as a `RoadMarking` item in the intersection lane, it overwrites the stop position. (Not only creating `stop_line`, but also defining as a `RoadMaking` item are needed.)

## Exclusion setting of attention lanes [RightOfWay] (Fig.3)

By default, IntersectionModule treats all lanes crossing with the registered lane as attention targets (yellow and green lanelets). But in some cases (e.g. when driving lane is priority lane or traffic light is green for the driving lane), we want to ignore some of the yield lanes. By setting `RightOfWay` of the `RegulatoryElement` item, we can define lanes to be ignored. Register ignored lanes as “yield” and register the attention lanes and driving lane as “right_of_way” lanelets in `RightOfWay` RegulatoryElement (For an intersection with traffic lights, we need to create items for each lane in the intersection. Please note that it needs a lot of man-hours.)
