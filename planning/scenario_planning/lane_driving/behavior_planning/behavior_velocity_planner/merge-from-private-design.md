## Walkway

### Role

私有地（例えば駐車場）から公道へ出る際には面出しする必要がある. もし交差点のレーンに私有地タグ(location = private)がついていて、次のレーンが私有地ではない場合は必ず一時停止をして交差点に侵入する. 停止位置の算出方法は交差点モジュールと同じで、注視領域も同じ
i.e. 必ず一時停止を行う以外は交差点モジュールと同じ機能になる.

![walkway](docs/intersection/merge_from_private.png)

### Launch Timing

Launches when there is a conflicting lanelet in ego private tag lane and other no private lane.

### Module Parameters

| Parameter                                   | Type   | Description                     |
| ------------------------------------------- | ------ | ------------------------------- |
| `merge_from_private_road/stop_duration_sec` | double | [m] time margin to change state |

### Known Issue

If ego vehicle surpassed stop line without stopping, then ego vehicle can't restore from STOP state and can't move any more.
