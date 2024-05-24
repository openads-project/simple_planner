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
| `drivable_mode` | `bool` | publish trajectory in `drivable` (true, recommended) or `reference` format (false) |
| `n_states` | `int` | defines the (maximum) number of states of the published trajectory |
| `v_ref` | `double` | defines the constant velocity of the published trajectory |
| `a_max_decel` | `double` | defines the maximum deceleration for the braking maneuver before the end of the route (has to be `<= 0.0`; if `= 0.0`, no braking) |
| `consider_traffic_lights` | `bool` | consider traffic lights for trajectory planning (stop at red traffic lights or not) |
| `offset_to_stop_line` | `double` | additional distance to stop in front of a stop line (m) (default: 0.0 -> stops with front of vehicle at stop line) |

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