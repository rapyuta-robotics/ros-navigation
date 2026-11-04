# global_planner

Nav2 global planner plugin (`nav2_core::GlobalPlanner`) running the `global_planner` navigation function on the
global costmap: a Dijkstra (or A*) potential with a quadratic update, gradient-descent path extraction, a goal
tolerance search and configurable path orientations. Built for ROS 2 Lyrical / Nav2 1.5.2, C++20.

## Contents

| Part | Files | Description |
|---|---|---|
| Plugin | `global_planner.h/.cpp`, `bgp_plugin.xml` | `global_planner::GlobalPlanner`: parameters, publishers, locking, outcome → exception mapping |
| Core | `planner_core.h/.cpp` | `global_planner::PlannerCore`: node-free planner on a `nav2_costmap_2d::Costmap2D`; returns `rr_nav_msgs/msg/GetPathOutcome` codes |
| Parameters | `parameter_handler.h/.cpp` | `nav2_util::ParameterHandler` subclass: declaration, ranges, runtime updates |
| Algorithm | `astar`, `dijkstra`, `expander`, `gradient_path`, `grid_path`, `orientation_filter`, `potential_calculator`, `quadratic_calculator`, `traceback` | potential calculation, path extraction, orientation filter |

The core can be used without a node: `PlannerCore(costmap, frame_id, options, config, clock, logger)` and
`makePlan(start, goal, tolerance, plan, message)`. Its publish hooks (`publishPlan`, `publishPotential`,
`showFootprintRadii`, `clearFootprintRadii`) do nothing; the plugin overrides them.

## Usage

```yaml
planner_server:
  ros__parameters:
    costmap_update_timeout: 0.0
    planner_plugins: ["GlobalPlanner"]
    GlobalPlanner:
      plugin: "global_planner::GlobalPlanner"
      neutral_cost: 100
      cost_factor: 0.55
      allow_unknown: false
```

`createPlan(start, goal, viapoints, cancel_checker)`:

- Plans from `start` to `goal` (both in the costmap global frame, as `planner_server` provides them) with the current
  `tolerance` parameter.
- Holds the costmap mutex and the plugin parameter mutex for the whole computation, so parameter updates wait for
  the running plan and apply to the next one.
- Returns the plan stamped in the costmap global frame; it is never empty.
- `cancel_checker` is not polled: the plan is one uninterruptible computation.
- `viapoints` are not supported.

## Plan

- The first pose is the start position shifted by half a cell per axis; interior poses lie on the gradient path
  (cell coordinates + 0.5).
- The last pose is the goal, or the displaced goal found by the tolerance search, with the goal orientation.
- Start and goal cells are found by truncation (`Costmap2D::worldToMap`).
- Cells with cost `< lethal_cost` are passable; unknown cells (255) are passable only with `allow_unknown`.
- With `outline_map`, the border cells of the global costmap are set to lethal before planning.

Tolerance search: if the goal cell has no potential, the planner samples a disc of radius `tolerance` around the goal
(step `min(resolution / 2, 0.999 * tolerance)`) and moves the goal to the nearest sample with a potential, strictly
closer than `tolerance`.

## Outcomes

Failures throw `nav2_core` exceptions whose message starts with `[rr_code=<N>] `, `N` being the
`rr_nav_msgs/msg/GetPathOutcome` code. The `rr_nav_adapter` error codec decodes it from the action `error_msg`.

| Code | Condition | Exception |
|---|---|---|
| 54 `BLOCKED_START` | start cell cost ≥ 253 | `StartOccupied` |
| 55 `BLOCKED_GOAL` | `tolerance == 0` and goal cell cost ≥ 253 (checked before planning, nothing published) | `GoalOccupied` |
| 55 `BLOCKED_GOAL` | no potential at the goal and no sample in the tolerance disc has one, and the goal cell cost is ≥ 253 or `tolerance == 0` (the potential and the radii are published first) | `GoalOccupied` |
| 56 `NO_PATH_FOUND` | `tolerance > 0`, goal cell cost < 253, nothing reachable in the disc; or path extraction failed | `NoValidPathCouldBeFound` |
| 63 `OUT_OF_MAP` | start outside the costmap | `StartOutsideMapBounds` |
| 63 `OUT_OF_MAP` | goal outside the costmap | `GoalOutsideMapBounds` |
| 52 `INVALID_START` / 53 `INVALID_GOAL` | start / goal frame differs from the costmap global frame | `PlannerException` |
| 53 `INVALID_GOAL` | non-empty `viapoints` | `PlannerException` |
| 60 `NOT_INITIALIZED` | `createPlan` before `configure` | `PlannerException` |
| 62 `INTERNAL_ERROR` | any other exception during planning | `PlannerException` |

With `tolerance == 0`, a free but unreachable goal is reported as `BLOCKED_GOAL`, not `NO_PATH_FOUND`. With
`tolerance > 0`, only the goal cell decides between `BLOCKED_GOAL` and `NO_PATH_FOUND`.

## Parameters

Declared on the planner node as `<plugin name>.<parameter>`, e.g. `GlobalPlanner.tolerance`.

Runtime-writable (validated against the range, applied under the plugin mutex):

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `tolerance` | double | 0.0 | ≥ 0 | Goal tolerance [m] for the next plan (no per-request tolerance in Nav2; clients write it before each request) |
| `orientation_mode` | int | 1 | 0–9 | Orientation of the plan poses, see below |
| `orientation_window_size` | int | 1 | 1–255 | Window of the position derivative used for the orientation |
| `lethal_cost` | int | 253 | 1–255 | Lethal cost |
| `neutral_cost` | int | 50 | 1–255 | Neutral cost |
| `cost_factor` | double | 3.0 | 0.01–5.0 | Factor to multiply each cost from costmap by |
| `publish_potential` | bool | true | | Publish the potential after every plan (only built when subscribed) |
| `show_footprint_radii` | bool | true | | Publish the inscribed/circumscribed radii markers; disabling publishes `DELETEALL` |

Read-only (read at configure; runtime sets are rejected):

| Parameter | Type | Default | Description |
|---|---|---|---|
| `old_navfn_behavior` | bool | false | Exactly mirror the navfn coordinate convention |
| `use_quadratic` | bool | true | Quadratic potential approximation |
| `use_dijkstra` | bool | true | Dijkstra expansion, otherwise A* |
| `use_grid_path` | bool | false | Grid-boundary path, otherwise gradient descent |
| `allow_unknown` | bool | true | Plan through unknown cells |
| `publish_scale` | int | 100 | Scale of the published potential |
| `outline_map` | bool | true | Outline the global costmap with lethal obstacles |

Orientation modes (the last pose always keeps the goal orientation):

| Value | Mode | Orientation of the other poses |
|---|---|---|
| 0 | None | unchanged (identity) |
| 1 | Forward | path tangent |
| 2 | Interpolate | linear blend from the start to the goal orientation |
| 3 | ForwardThenInterpolate | tangent until the last straightaway, then a blend to the goal (increment indexed from the path start) |
| 4 | Backward | tangent + π |
| 5 | Leftward | tangent − π/2 |
| 6 | Rightward | tangent + π/2 |
| 7 | Bidirectional | tangent, flipped by π if turning start→path plus path→goal exceeds π (uses the second pose from each end for paths of 5+ poses) |
| 8 | FixedOrientation | start orientation |
| 9 | Sideward | tangent ± π/2, whichever needs less turning from start to path and from path to goal |

## Topics

Published relative to the planner node under the plugin name (default `StandardTopicQoS`):

| Topic | Type | When |
|---|---|---|
| `GlobalPlanner/plan` | `nav_msgs/msg/Path` | after every plan that reaches path extraction, also empty ones |
| `GlobalPlanner/potential` | `nav_msgs/msg/OccupancyGrid` | after the potential calculation, with `publish_potential` and at least one subscriber |
| `GlobalPlanner/footprint_radii` | `visualization_msgs/msg/Marker` | two `LINE_STRIP` markers (`inscribed_radius`, `circumscribed_radius`) in the robot base frame with `show_footprint_radii` |

## Tests

```bash
colcon test --packages-select global_planner
```

- `test_orientation_filter`: all orientation modes.
- `test_planner_core`: node-free core on hand-built costmaps (blocked start/goal, tolerance disc, cell borders,
  lethal threshold, unknown cells, publications).
- `test_global_planner`: plugin export, exceptions and tags, parameters, locking, topics.
