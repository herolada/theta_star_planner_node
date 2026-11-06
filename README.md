# theta_star_planner_node

A standalone **Theta\*** (any-angle, Lazy Theta\*) planner node, adapted from
[`nav2_theta_star_planner`](../navigation2/nav2_theta_star_planner), **without** the rest
of the Nav2 stack — no `Costmap2DROS`, no costmap layers, no lifecycle/planner-server
machinery.

It is a drop-in alternative to [`smac_planner_node`](../smac_planner_node): same
traversability-cloud costmap, same `ComputePathToPose` action, same TF usage, same
parameter names for everything that isn't planner-specific.

## Interfaces

| Direction | Name | Type | Notes |
|---|---|---|---|
| Subscribe | `traversability` | `sensor_msgs/PointCloud2` | Square grid; float field `traversability` in `[0, 1]` (0 = most traversable). Only the **latest** cloud is kept. |
| Action server | `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | Goal `PoseStamped` in → result `nav_msgs/Path` out. |
| Publish | `~/plan` | `nav_msgs/Path` | Last planned path (debug/visualization). |
| Publish | `~/plan_raw` | `nav_msgs/Path` | Last path's any-angle vertices, before interpolation. |
| Publish | `~/costmap` | `nav_msgs/OccupancyGrid` | Planning costmap (after inflation, if enabled). |
| Publish | `~/planning_start`, `~/planning_goal` | `geometry_msgs/PoseStamped` | Start/goal of the last request, in the global frame. |
| TF | — | — | Looks up `global_frame → robot_base_frame` for the start pose when `use_start=false`. |

The **global frame** is taken from the `frame_id` of the latest traversability cloud.
Goal (and `start`, when `use_start=true`) poses are transformed into that frame via TF.
Error codes are the `ComputePathToPose` ones (`GOAL_OCCUPIED`, `GOAL_OUTSIDE_MAP`,
`START_OUTSIDE_MAP`, `NO_VALID_PATH`, `TIMEOUT`, ...). A `planner_id` containing
`respect_heading` forces the goal orientation on the final pose, as in `smac_planner_node`.

The launch file serves the action on `/theta_star/compute_path_to_pose` so it can run
beside `smac_planner_node` (`/hybrid/compute_path_to_pose`); pass
`action:=/hybrid/compute_path_to_pose` to swap it in for smac instead.

## Costmap construction

Identical to `smac_planner_node` (same parameters): the cloud is a regular grid with cell
spacing `resolution`; `t >= lethal_threshold` → `LETHAL_OBSTACLE`, otherwise linearly
scaled into `[0, 252]`, missing/non-finite → `NO_INFORMATION`; optional minimum
robot-centered rectangle, margins and `trav_max_dist`.

**Optional inflation** (`inflation_radius > 0`, off by default): Theta\* plans for a point
and its any-angle paths hug obstacles, so lethal cells can be inflated with Nav2's
`InflationLayer` cost model — cells within `robot_radius` become untraversable (253),
cost decays as `252·exp(-cost_scaling_factor·(d − robot_radius))` out to
`inflation_radius`.

## Path

Theta\* vertices are joined by straight segments and interpolated at the costmap
resolution. Every segment is line-of-sight checked against the cells it actually crosses,
so the returned path only passes through traversable cells (cost ≤ 252, or unknown with
`allow_unknown`). The first/last pose is the exact start/goal position when that keeps
this guarantee (otherwise the cell center). Each pose faces the next one; the final pose
takes the goal orientation, or the last segment's direction with
`use_final_approach_orientation: true`.

## Parameters

See `config/theta_star_planner_node.yaml` (and the `_taros` / `_helhest` variants, which
mirror the corresponding `smac_planner_node` configs). Planner-specific:

| Parameter | Default | Meaning |
|---|---|---|
| `how_many_corners` | 8 | 4- or 8-connected expansion |
| `w_euc_cost` | 1.0 | weight on path length — higher gives tauter paths |
| `w_traversal_cost` | 2.0 | weight on `(cost/252)²` — higher keeps to low-cost cells |
| `use_final_approach_orientation` | false | final pose orientation = last segment direction |
| `tolerance` | 0.25 | m; if the goal is unreachable, end at the closest reachable cell within this |
| `max_iterations` / `max_planning_time` | 1e6 / 5.0 | search limits (`TIMEOUT`, unless something is within `tolerance`) |
| `inflation_radius` / `cost_scaling_factor` | 0.0 / 10.0 | see above |

Search parameters (the table above minus inflation, plus `allow_unknown` and
`terminal_checking_interval`) can be changed at runtime, e.g. with `rqt_reconfigure`.

## Build & run

```bash
# from the workspace root
bash ./build.sh --packages-select theta_star_planner_node

. /opt/ros/kilted/setup.sh && . install/setup.bash
ros2 launch theta_star_planner_node theta_star_planner_node.launch.py
# or
ros2 run theta_star_planner_node theta_star_planner_node --ros-args \
  --params-file install/theta_star_planner_node/share/theta_star_planner_node/config/theta_star_planner_node.yaml
```

The dev helpers in `smac_planner_node/scripts` work with this node too:
`rviz_goal_relay.py --ros-args -p action_name:=theta_star/compute_path_to_pose`, and
`sim_costmap_gui.py --ros-args -p config_file:=<theta_star_planner_node_*.yaml>`.

## Relationship to `nav2_theta_star_planner`

The search (`include/theta_star_planner_node/theta_star.hpp`, `src/theta_star.cpp`) is a
vendored copy of the upstream `theta_star::ThetaStar`, kept separate from the ROS node and
depending only on `nav2_costmap_2d::Costmap2D`. Nothing in `navigation2` needs to be
modified or built. The cost function and lazy parent reset are unchanged; the fixes and
additions (costmap-size changes between plans, read-only costmap, center-consistent
line-of-sight, no corner cutting, tolerance/limits) are listed at the top of
`theta_star.hpp`.
