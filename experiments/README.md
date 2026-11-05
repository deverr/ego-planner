# Controlled experiments on EGO-Planner

This directory adds a small, reproducible study on top of
[ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner):
**how does obstacle density, and how do dynamic limits, affect the planner's
success rate, flight time and clearance?**

Everything here is additive — no upstream source file is modified. The
parameterised launch file is a copy of the stock `simulator.xml` with the
`mockamap` noise parameters exposed as ROS launch args.

## Why the stock launch file cannot be swept as-is

Two things in the stock configuration get in the way of a sweep:

1. `mockamap` hardcodes `seed = 127`. Launching the simulator twice produces
   the *same* map, so N repeated runs are not N independent samples.
2. `c_num` / `p_num` in `simulator.xml` only feed the `random_forest` node,
   which is commented out inside a CDATA block. They have no effect in the
   default (perlin) configuration.

The real density knob for the perlin map is `fill` (infill percentage), plus
`complexity` (base noise frequency). `simulator_exp.xml` exposes `seed`,
`fill`, `complexity`, `fractal` and `attenuation` as launch arguments.

## Design

| Group | Swept variable | Fixed | Trials per condition |
|---|---|---|---|
| A | `fill` ∈ {0.12, 0.20, 0.28} | `max_vel=2.0`, `max_acc=3.0` | 12 seeds |
| B | `max_vel` ∈ {3.0, 4.0} | `fill=0.12` | 8 seeds |

Each trial starts a fresh `roscore` and a fresh simulator, flies the stock
5-waypoint loop (a 40 m × 40 m square at z = 1 m), and is terminated either
when the goal is reached or after a timeout.

### Metrics

| Metric | How it is obtained |
|---|---|
| `success` | goal reached within tolerance before timeout |
| `flight_time` | time from take-off to goal, in sim time |
| `path_len` | integrated odometry path length |
| `max_speed` | peak velocity magnitude |
| `min_clearance` | minimum distance from the trajectory to the ground-truth obstacle cloud (`/map_generator/global_cloud`), voxel-downsampled at 0.25 m |
| `replans` | number of `EXEC_TRAJ → REPLAN_TRAJ` transitions in the roslaunch log |

### Two details that matter for correctness

- **Goal detection uses cumulative path length, not a mid-point visit.**
  The first waypoint is ~3 m from the start, so "reached goal" also fires at
  take-off. A naive mid-point check is worse: the B-spline cuts corners and
  the drone passes up to ~1.6 m away from the nominal waypoint, so a
  "must pass within 0.8 m of waypoint 2" test never triggers and every trial
  is scored as a timeout. Requiring `cum_path > 40 m` before accepting the
  goal fixes both cases.
- **`pcl_render_node` needs an OpenGL context.** The launch must run with a
  `DISPLAY` pointing at an X server (Xvfb is fine); without it the node dies
  with `boost::lock_error`.

## Run

```bash
# from the catkin workspace root, after catkin_make
source devel/setup.bash

cd src/ego-planner
./experiments/run_all.sh --quick     # 2 seeds, 1 condition, ~4 min
./experiments/run_all.sh             # full sweep, ~2 h
FILLS="0.12 0.20" SEEDS="1 2 3" ./experiments/run_all.sh
```

Outputs:

```
results/summary.csv     one row per trial
results/table.md        aggregated markdown table
results/fig_fill.png    flight-time boxplot
results/raw/            per-trial odometry csv + obstacle cloud npy (gitignored)
logs/                   per-trial roslaunch logs (gitignored)
```

Plots need `matplotlib`; the metrics themselves only need `numpy`.

## Results

### Obstacle density (`max_vel = 2.0`)

| fill | trials | success | flight time (s) | mean clearance (m) | replans |
|---|---|---|---|---|---|
| 0.12 | 12 | 10/12 (83%) | 59.9 ± 2.4 | 0.14 | 46 |
| 0.20 | 12 | 9/12 (75%) | 60.7 ± 2.3 | 0.12 | 63 |
| 0.28 | 10 | 9/10 (90%) | 62.1 ± 4.3 | 0.09 | 76 |

### Velocity limit (`fill = 0.12`)

| max_vel | trials | success | flight time (s) | mean clearance (m) | replans |
|---|---|---|---|---|---|
| 2.0 | 12 | 10/12 (83%) | 59.9 ± 2.4 | 0.14 | 46 |
| 3.0 | 8 | 7/8 (88%) | 41.3 ± 1.2 | 0.12 | 61 |
| 4.0 | 8 | 6/8 (75%) | 35.1 ± 3.6 | 0.08 | 92 |

### What the numbers say

- **Success rate does not move.** 10/12, 9/12, 9/10 across densities and 10/12,
  7/8, 6/8 across speeds. At n ≈ 10 the 95% interval on a 80% rate is roughly
  ±25 points, so these differences are not distinguishable from noise. Reporting
  them as a trend would be wrong.
- **The cost shows up in replanning and clearance instead.** Going from 2 to
  4 m/s cuts flight time by 41%, but doubles the replan rate (46 → 92) and
  halves mean clearance (0.14 → 0.08 m). Density shows the same signature more
  weakly: flight time is flat (60 → 62 s) while replans climb 65% and clearance
  falls from 0.14 to 0.09 m.
- **Reading:** the planner keeps its nominal success rate by replanning more
  often, and the safety margin is what absorbs the extra demand. The margin is
  being bought, not held. If the group's real constraint is close-proximity
  flight in narrow gaps, success rate is the wrong metric — clearance and replan
  rate are the ones that move first.

### Excluded trials

`fill = 0.28`, seeds 4 and 7 were rejected after 3 attempts: the drone never
left the ground, because `mockamap` places obstacles without guaranteeing start
clearance and the start pose landed inside one. That is a map-generation
artifact, not a planner failure, and it is recorded in `results/skipped.txt`
rather than scored as a failure. 2 of 12 maps at the highest density were
affected — worth knowing if this map generator is ever used for evaluation.

## Known limitations

- `seed` fixes the map, but execution is **not** deterministic: ROS uses wall
  clock and the planner is multi-threaded, so the same seed can produce
  different trajectories. Repeated runs therefore sample map *and* timing
  noise. Both facts are reported rather than hidden.
- `min_clearance` is measured against the sensing-resolution point cloud, so
  it is accurate to roughly the voxel size (0.25 m).
- The waypoint loop is fixed. A hard evaluation would randomise start/goal
  per trial as well.
- n = 8–12 per condition is enough to see the replan and clearance shifts, and
  not enough to resolve success-rate differences. Extending to 30 seeds per
  condition is the obvious next step.
