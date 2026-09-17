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

## Output

`validation/results/` (not committed — it is measurement data, regenerated on
demand):

- `results.csv` — one row per run: outcome, duration, place error, tool-tip
  errors, carriage lift, which phase it ended in, and the abort reason
- `run_NNN.log` — the full log of each run, kept so any row can be traced back
- `worlds/run_NNN.sdf` — the exact world that run used
- `results_table.tex`, `results.png` — with `--latex` / `--chart`

The numbers quoted in the report should come from `results.csv`, so the text and
the measurements cannot drift apart.

## Cost

A run is a full simulation: a few minutes each, so 20 runs is roughly an hour.
It is headless and one at a time on purpose — two simulators at once share
`/clock` and both become worthless.
