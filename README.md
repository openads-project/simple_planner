# simple_planner

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/simple_planner/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/simple_planner"/></a>
  <a href="https://github.com/openads-project/simple_planner/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/simple_planner"/></a>
  <br>
  <a href="https://github.com/openads-project/simple_planner/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/simple_planner/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/simple_planner/actions/workflows/compose-oci.yml"><img src="https://github.com/openads-project/simple_planner/actions/workflows/compose-oci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/simple_planner"><img src="https://github.com/openads-project/simple_planner/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/simple_planner/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/simple_planner/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**ROS 2 Reference Trajectory Planning for Automated Driving**

This repository provides a lightweight ROS 2 planner that combines route information, the current ego state, and optional environment information to periodically generate a reference trajectory. It is intended as a deterministic upstream reference planner for downstream trajectory optimization and control modules. Key features:
- **Route following**: creates reference trajectories from route lane geometry and speed limits.
- **Safe-stop behavior**: publishes standstill or safe-stop trajectories when required inputs are missing, outdated, or a stop is needed.
- **Traffic-light handling**: stops at relevant traffic-light regulatory elements with configurable stop-line offsets and state prediction support.
- **Lane changes and indicators**: inserts simple lane-change transitions and trigger turn-signal or hazard-light services based on route semantics.
- **Object-aware speed handling**: checks perceived objects and predictions against the ego trajectory and applies a conservative speed cap when conflicts are detected.

The ROS 2 node uses the open-source ROS 2 message definitions [perception_interfaces](https://github.com/ika-rwth-aachen/perception_interfaces) and [planning_interfaces](https://github.com/ika-rwth-aachen/planning_interfaces) for its in- and outputs, making it straightforward to integrate into larger ROS 2-based automated-driving systems.

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>

> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Systems* project. *OpenADS* and its modules have been initiated and are currently being maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/de/).

## 🚀 Quick Start

1. Launch the `demo/docker-compose.yml` setup. This will open RViz with a visualization of a Lanelet2 map. 
    ```bash
    cd demo
    xhost +local: # allow GUI forwarding from containers
    docker compose up -d
    ```
2. Select the *Plan Route* tool in RViz and click on a destination on the map to plan a route, which will be visualized as a green line. This is the basis for the reference trajectory planned by the `simple_planner`. The reference trajectory is visualized in RViz as a red line with red dots.

3. Stop the demo and clean up.
    ```bash
    docker compose down
    xhost -local: # revoke GUI forwarding permissions
    ```

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/simple_planner.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd simple_planner
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

Package and node interfaces are documented in the respective package READMEs listed below. Implementation details are found in the [Source Code Documentation](https://openads-project.github.io/simple_planner).

| Package | Description |
| --- | --- |
| [simple_planner](simple_planner/README.md) | Generates route-following reference trajectories with safe-stop, traffic-light, turn-signal, and object-aware speed handling. |

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| [AIGGREGATE](https://aiggregate.eu/) | 🇪🇺 European Union | 101202457 |
| [AIthena](https://aithena.eu/) | 🇪🇺 European Union | 101076754 |
| [autotech.agil](https://www.autotechagil.de/) | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 01IS22088A |

<p>
  <img src="https://www.drought.uni-freiburg.de/stressres/images/bmftr-logo/image" height=70>
  <img src="https://ec.europa.eu/regional_policy/images/information-sources/logo-download-center/eu_funded_en.jpg" height=70>
</p>

<sup><sub>Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor CINEA can be held responsible for them.</sup></sup>
