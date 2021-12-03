## CrossWalk

### Role For Crosswalk

Judgement whether a vehicle can go into a crosswalk and plan a velocity of the start/stop.

![crosswalk](docs/crosswalk/crosswalk.png)

### Role For Walkway

tbd.

### Activation Timing

Launches when there is a crosswalk on the target lane.

### Limitations

#### For Crosswalk

横断歩道の減速エリアや停止エリアはパラメータで定義されており、このエリアの外にいる歩行者や自転車には対応できない。

#### For Walkway

tbd.

### Inner-workings / Algorithms

#### Scene Crosswalk

The crosswalk module considers the following objects as target objects.

- pedestrian
- cyclist

##### Stop condition

If any of conditions below is met, the vehicle will stop at stop point.

- A target object exists in the **stop area**.
- A target object in the **crosswalk area** is predicted to enter the stop area within 3 seconds based on its prediction path.

##### Decelerate condition

If any of conditions below is met, the vehicle will decelerate to be 10 km/h at slow point.

- A target object exists in the **deceleration area**.

#### Scene Walkway

- TBD

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

### Known Issues

#### Crosswalk

横断歩道の減速エリアで埋め込む速度が一律 10[km/h]となっているが、衝突点と歩行者との距離が加味されていないため、高速走行しているときに歩行者が横断歩道に飛び出してきても停止できない可能性がある。

#### Walkway

停止せずに停止線を超えてしまった場合 STOP の state に入ったままでその後に永遠に動き出すことはない。
