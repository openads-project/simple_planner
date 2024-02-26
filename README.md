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
| `~/demo_trajectory` | `trajectory_planning_msgs/msg/trajectory` | Demo Trajectory (published with 0.1 Hz) |

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `frequency` | `double` | frequency for publishing output trajectory |
| `drivable_mode` | `bool` | publish trajectory in `drivable` (true) or `reference` format (false) |

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