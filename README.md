# simple_planner

- [Nodes](#nodes)
  - [simple_planner/simple_planner_node](#simple_plannersimple_planner_node)
- [Usage of docker-ros Images](#usage-of-docker-ros-images)
  - [Available Images](#available-images)
  - [Default Command](#default-command)
  - [Environment Variables](#environment-variables)
  - [Launch Files](#launch-files)
  - [Configuration Files](#configuration-files)


## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `simple_planner` | `simple_planner_node` | plans trajectory based on route with constant velocity |

### simple_planner/simple_planner_node

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/ego_data` | `perception_msgs/msg/ego_data` | Input EgoData |
| `~/route` | `route_planning_msgs/msg/route` | Input Route |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/trajectory` | `trajectory_planning_msgs/msg/trajectory` | Output Trajectory |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `trajectory_frame_id` | `string` | Frame ID of published reference trajectory |
| `fixed_over_time_frame_id` | `string` | Frame ID of frame that is fixed over time for finding temporal transforms |
| `frequency` | `double` | frequency for publishing output trajectory |
| `route_timeout` | `double` | time after which a received route is considered invalid (s) (use `-1.0` for no timeout) |
| `ego_data_timeout` | `double` | time after which a received ego vehicle data is considered invalid (s) (use `-1.0` for no timeout) |
| `trajectory_horizon` | `double` | covered time horizon of the published trajectory (s) |
| `n_states` | `int` | defines the number of states in the published trajectory |
| `internal_route_update` | `bool` | true: the route is received once and then updated locally in this node (cutting off the traveled route, etc.); false: the route is received cyclically (it is updated externally). |
| `interpolation_type` | `int` | defines the interpolation type of the received route, which is important for sampling it time equidistantly (0: linear, 1: spline) |
| `standstill_threshold` | `double` | if the velocity is below this threshold, the vehicle is considered to be in standstill (m/s) |
| `v_ref` | `double` | reference velocity (m/s). If set to a positive value, the trajectory will be generated with this constant velocity. If set to `-1.0`, the velocity from the route will be used, which is defined in the route message. |
| `a_decel` | `double` | desired deceleration for braking at stop lines or end of route (m/s^2) - must be < 0.0 |
| `a_max_decel` | `double` | maximum deceleration for safe-stop trajectories (m/s^2) - must be < 0.0 and < a_decel |
| `consider_traffic_lights` | `bool` | true: planner will consider traffic lights; false: planner will ignore traffic lights |
| `offset_to_stop_line` | `double` | additional distance to stop in front of a stop line (m) (default: 0.0 -> stops with front of vehicle at stop line) |
| `consider_future_states` | `bool` | true: trajectory will consider forecast of traffic light states; false: trajectory will only consider current traffic light state |
| `lane_change_distance_factor` | `double` | factor multiplied with the current velocity to determine the lane change distance (m) |
| `lane_change_min_distance_factor` | `double` | factor multiplied with the vehicle length to determine the minimum lane change distance (m) |

## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| `latest` | latest version |

### Default Command

```bash
ros2 launch simple_planner simple_planner.launch.py
```

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `simple_planner` | `simple_planner.launch.py` | `/docker-ros/ws/install/simple_planner/share/simple_planner/launch/` | launchs the simple planner node with default config |

### Configuration Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `simple_planner` | `params.yml` | `/docker-ros/ws/install/simple_planner/share/simple_planner/config/` | config defaultly loaded by `simple_planner.launch.py` |