# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package is

`theta_star_planner_node` is a standalone Theta* planner: a vendored, adapted copy of
`nav2_theta_star_planner`'s `ThetaStar` search driven directly by an `rclcpp` node, without
the rest of the Nav2 stack. It is the **Planning** module alternative to the sibling
`smac_planner_node` (Hybrid-A*) and deliberately keeps the **same ROS interface**: same
traversability-cloud costmap construction and parameter names, same `ComputePathToPose`
action (incl. error codes and the `respect_heading` planner_id convention), same TF usage,
same `~/plan`, `~/costmap`, `~/planning_start`, `~/planning_goal` debug topics. When
changing shared behaviour (costmap construction, action handling, TF), keep the two
packages in sync.

## Build & run

From the **workspace root** (`ws_goal`):

```bash
bash ./build.sh --packages-select theta_star_planner_node
. /opt/ros/kilted/setup.sh && . install/setup.bash
ros2 launch theta_star_planner_node theta_star_planner_node.launch.py   # serves /theta_star/compute_path_to_pose
```

No dependency on `nav2_theta_star_planner` or `nav2_core`; nothing in `src/navigation2`
needs building. No test suite beyond `ament_lint_auto`.

## Layout

- `include/theta_star_planner_node/theta_star.hpp`, `src/theta_star.cpp` — the search. ROS-free
  (only `nav2_costmap_2d::Costmap2D`), built as the static `theta_star_core` library. The header
  comment lists every deviation from upstream; the cost function and lazy parent reset are
  upstream's. Key invariants:
  - the costmap is read-only; the start cell is reported as free by `cost()` (upstream wrote
    FREE_SPACE into the costmap);
  - `resetContainers()` sizes `node_position_`/`nodes_data_` for the current map on every
    search and `nodes_data_` is reserved to the cell count, so `TreeNode*` pointers stay valid
    (the costmap size changes on nearly every traversability message);
  - `losCheck()` walks cells crossed by the segment between cell **centers** (where
    `backtrace()` puts vertices), and diagonal expansion can't cut corners — so every path
    segment only crosses safe cells. Upstream's corner-based Bresenham check did not
    guarantee this.
- `include/.../costmap_inflation.hpp`, `src/costmap_inflation.cpp` — optional nav2
  `InflationLayer`-style inflation (`inflation_radius > 0`), inscribed radius = `robot_radius`.
- `src/theta_star_planner_node.cpp` — the node. `buildCostmap`, `handleGoal`/`execute`, TF
  helpers are copies of `smac_planner_node`'s; `createPlan` mirrors upstream
  `ThetaStarPlanner::createPlan`/`getPlan`. Search params are live-updatable via
  `dynamicParametersCallback` (takes `_planning_mutex`).
