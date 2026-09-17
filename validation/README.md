# Validation: repeating the mission and reporting the spread

Every measured number elsewhere in this project comes from a single run. One run
cannot tell "the system works" from "the system worked once", so this directory
produces the thing that can: repeats, and the distribution across them.

```bash
# 20 runs, each on its own slightly different world, headless
./scripts/run_native.sh python3 validation/run_batch.py --n 20

# table + chart, and a LaTeX table for the report
./scripts/run_native.sh python3 validation/summarize.py --latex --chart
```

## Two records per run, because they answer different questions

A run leaves behind **its log** and **a rosbag**.

The log says what the robot *believed and reported*. That is the right basis for
"did the mission succeed": a success has to be something the system claims of
itself, not something the analysis grants it afterwards.

The bag says what *actually happened*, and it is recorded whether or not anyone
has yet thought of the question. That matters because the log can only ever
contain quantities someone decided in advance to print, in a form meant to be
read on the fly. Every question that came back on the report — how far from the
table did it really drive, how wrong was the localiser, where did the box
physically end up — is answered from the bag, offline, by
`validation/analyze_runs.py`.

Recorded topics: ground truth (`/debug/gz_dynamic_pose`, bridged only for these
runs), `/amcl_pose`, `/tf`, `/scan`, `/scan_filtered`, `/joint_states`,
`/cmd_vel`, `/plan`, the navigator and mission status topics, the box state and
the four fingertip contacts. Cameras and point clouds are left out: they are the
bulk of the data and nothing in the analysis reads them.

`--no-bag` turns recording off, and `--no-truth` drops the ground-truth bridge.
Both make a run cheaper and unmeasurable afterwards.

## What analyze_runs.py measures

It opens each bag and measures it against **that run's own world file**, not
against constants:

| column | meaning |
|---|---|
| `table_clear_transit_min_m` | closest the robot came to a table's **top edge** while crossing a room. This is the driven counterpart of the report's 0.835 m, which was planned on the saved map outside the simulator |
| `table_clear_approach_min_m` | the same on the deliberate approach to a table, reported separately — the dock pose is 10 cm from the plate, and a metre of clearance there would mean the robot never arrived |
| `wall_clear_open_min_m` | closest approach to a wall away from the doorways |
| `door_clear_min_m`, `door_passes` | the squeeze through each doorway, and how many were passed |
| `amcl_err_max_cm`, `amcl_err_rms_cm`, `amcl_yaw_max_deg` | AMCL against ground truth |
| `place_err_truth_mm`, `place_dx_mm`, `place_dy_mm` | where the box physically came to rest, against the marker in the world file — independent of the robot's own `PLACE VERIFIED` |
| `cube_lift_m`, `path_len_m`, `drive_time_s` | how far the box was raised, how far and how long the robot drove |

The split between crossing a room and approaching a table is made at the last
moment the robot was still 1.20 m from the table it then docks at. That is above
the figure being checked, so the check is made on the crossing and cannot be
flattered by the approach.

Where a run never moved the box, the placement columns stay empty. The distance
from where the box still sits to the marker is not a placement error.

## How a run is judged

`run_batch.py` never talks to the mission node. It reads the run's log — the
same text a person would read — and classifies the outcome from what the robot
itself printed:

| outcome | meaning |
|---|---|
| `success` | `PLACE VERIFIED` **and** `MISSION COMPLETE` both present |
| `aborted` | the mission stopped itself and recorded a reason |
| `stalled` | it started but reached neither an abort nor a completion before the timeout |
| `never_started` | the stack never came up far enough to wait for a command |

`PLACE VERIFIED` is required for a success, not just `MISSION COMPLETE`: that
line is the independent re-measurement of where the box actually ended up.

## Why each run gets its own world

With an identical world every repeat measures the same thing N times, and the
result looks far more certain than it is. `--jitter` (default 0.03 m) moves the
box by a few centimetres per run via `scripts/gen_world.py`, so the perception
and the grasp are solving a slightly different problem each time. `--jitter 0`
turns that off when the point is to isolate something else.

## Where it all lands

One directory per batch, one folder per run inside it, so any number can be
traced back to the exact world, log and recording it came from. Nothing is
committed — it is measurement data, regenerated on demand.

```
validation/results/
    latest -> 2026-09-18_0132/          symlink to the most recent batch
    2026-09-18_0132/
        results.csv                     one row per run: outcome, duration,
                                        place error, tool-tip errors, carriage
                                        lift, phase it ended in, abort reason
        metrics.csv                     one row per run, measured from the bags
        results_table.tex               with --latex
        results.png                     with --chart
        run_001/
            run.log                     everything that run printed
            world.sdf                   the exact world it was driven in
            bag/                        the recorded topics
            metrics.json                that run's measurements
        run_002/
            ...
```

A batch is named after the time it started, or by `--batch <name>`. Naming an
existing batch adds to it and the run numbering continues, so `run_007` is
always the seventh run of that batch and nothing else. `analyze_runs.py` and
`summarize.py` both read `latest` unless given `--batch`, and both write their
output **inside** the batch, where the next batch cannot overwrite it.

The world is copied into the run folder rather than referenced, including with
`--stock-world`: a run folder that does not contain the world it used stops
being measurable the moment that world changes.

The numbers quoted in the report should come from a batch's `results.csv` and
`metrics.csv`, so the text and the measurements cannot drift apart.

## Cost

A run is a full simulation: a few minutes each, so 20 runs is roughly an hour.
It is headless and one at a time on purpose — two simulators at once share
`/clock` and both become worthless.

The bags are the other cost: a few hundred MB per run, so a batch of 20 is
measured in gigabytes. They are not committed, and once `metrics.csv` is written
a bag is only needed to ask a question the metrics do not already answer.
