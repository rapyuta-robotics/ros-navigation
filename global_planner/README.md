# global_planner

Nav2 global planner plugin (`nav2_core::GlobalPlanner`) running the `global_planner` navigation function on the
global costmap: a Dijkstra (or A*) potential with a quadratic update, gradient-descent path extraction and a goal
tolerance search. Built for ROS 2 Lyrical / Nav2 1.5.2, C++20.

It is the inner planner for `rr_nav_planner_wrapper::PlannerWrapper`; it is not meant to be used standalone. The
wrapper owns the RR planning contract (outcome codes and tags, frame, viapoint, start and goal checks, the
blocked-goal / no-path decision, orientations, the `plan` and `footprint_radii` topics); see the
`rr_nav_planner_wrapper` README.

## Contents

| Part | Files | Description |
|---|---|---|
| Plugin | `global_planner.h/.cpp`, `bgp_plugin.xml` | `global_planner::GlobalPlanner`: parameters, potential publisher, locking, status → exception mapping |
| Core | `planner_core.h/.cpp` | `global_planner::PlannerCore`: node-free planner on a `nav2_costmap_2d::Costmap2D`; returns a `PlanStatus` |
| Parameters | `parameter_handler.h/.cpp` | `nav2_util::ParameterHandler` subclass: declaration, ranges, runtime updates |
| Algorithm | `astar`, `dijkstra`, `expander`, `gradient_path`, `grid_path`, `potential_calculator`, `quadratic_calculator`, `traceback` | potential calculation, path extraction |

The core can be used without a node: `PlannerCore(costmap, frame_id, options, config, clock, logger)` and
`makePlan(start, goal, tolerance, plan)`. Its `publishPotential` hook does nothing; the plugin overrides it.

## Usage

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GlobalPlanner"]
    GlobalPlanner:
      plugin: "rr_nav_planner_wrapper::PlannerWrapper"
      primary_planner:
        plugin: "global_planner::GlobalPlanner"
        neutral_cost: 100
        cost_factor: 0.55
        allow_unknown: false
```

`createPlan(start, goal, viapoints, cancel_checker)`:

- Plans from `start` to `goal` with the current `tolerance` parameter. Poses are taken as given in the costmap global
  frame; `viapoints` and `cancel_checker` are ignored.
- Holds the costmap mutex and the plugin parameter mutex for the whole computation, so parameter updates wait for
  the running plan and apply to the next one.
- Returns the plan stamped in the costmap global frame.

| Result | Condition |
|---|---|
| non-empty `Path` | the goal or a displaced goal inside the tolerance disc has a potential and the path extraction succeeded |
| empty `Path` | a legal potential was found but the path extraction failed |
| `nav2_core::NoValidPathCouldBeFound` | no potential at the goal cell nor at any sample of the tolerance disc |
| `nav2_core::StartOutsideMapBounds` / `GoalOutsideMapBounds` | start / goal outside the costmap |
| `nav2_core::PlannerException` | `createPlan` before `configure` |

Exception messages carry no outcome tag.

## Plan

- The first pose is the start position shifted by half a cell per axis; interior poses lie on the gradient path
  (cell coordinates + 0.5) with identity orientation.
- The last pose is the goal, or the displaced goal found by the tolerance search, with the goal orientation.
- Start and goal cells are found by truncation (`Costmap2D::worldToMap`).
- Cells with cost `< lethal_cost` are passable; unknown cells (255) are passable only with `allow_unknown`. Start and
  goal cell costs are not checked.
- With `outline_map`, the border cells of the global costmap are set to lethal before planning.

Tolerance search: if the goal cell has no potential, the planner samples a disc of radius
`tolerance - 1e-6 * max(1, tolerance, |goal x|, |goal y|)` around the goal (step `min(resolution / 2, 0.999 * radius)`)
and moves the goal to the nearest sample with a potential, strictly closer than `tolerance`. A radius ≤ 0 samples
nothing.

## Parameters

Declared on the planner node as `<plugin name>.<parameter>`, e.g. `GlobalPlanner.primary_planner.tolerance`.

Runtime-writable (validated against the range, applied under the plugin mutex):

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `tolerance` | double | 0.0 | ≥ 0 | Goal tolerance [m] for the next plan |
| `lethal_cost` | int | 253 | 1–255 | Lethal cost |
| `neutral_cost` | int | 50 | 1–255 | Neutral cost |
| `cost_factor` | double | 3.0 | 0.01–5.0 | Factor to multiply each cost from costmap by |
| `publish_potential` | bool | true | | Publish the potential after every plan (only built when subscribed) |

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

## Topics

Published relative to the planner node under the plugin name, dots replaced by slashes (default `StandardTopicQoS`):

| Topic | Type | When |
|---|---|---|
| `GlobalPlanner/primary_planner/potential` | `nav_msgs/msg/OccupancyGrid` | after the potential calculation, with `publish_potential` and at least one subscriber |

## Tests

```bash
colcon test --packages-select global_planner
```

- `test_planner_core`: node-free core on hand-built costmaps (map bounds, cell borders, outline, tolerance disc,
  zero-radius disc, lethal threshold, unknown cells, potential grid, extraction failure).
- `test_global_planner`: plugin export, exceptions, empty path on extraction failure, parameters, locking, topic.
