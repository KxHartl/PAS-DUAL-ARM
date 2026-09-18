#!/usr/bin/env python3
"""Turn a batch's results.csv into a table, a chart and LaTeX.

The point of the batch is to replace single-run numbers with a distribution, so
this prints the distribution and nothing that hides it: N, how many succeeded,
and for each measure the median with the full observed range rather than a mean
on its own. Where a run failed, the phase it failed in is counted, because
"failed" without "where" cannot be acted on.

    ./scripts/run_native.sh python3 validation/summarize.py
    ./scripts/run_native.sh python3 validation/summarize.py --latex --chart
    ./scripts/run_native.sh python3 validation/summarize.py --batch 2026-09-18_0132

Without an argument it reads the batch `validation/results/latest` points at,
and writes the table and the chart into that same batch directory - so a figure
in the report can always be traced back to the runs it was made from.

--latex writes a table that can be included in the report directly, so the
numbers in the text and the numbers in the CSV cannot drift apart.
"""
import argparse
import csv
import os
import statistics
import sys
from collections import Counter

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(REPO, 'validation', 'results')
LATEST = os.path.join(RESULTS, 'latest')

# What the robot SAID about itself, parsed from its own log (results.csv).
# `place_error_mm` here is the robot's claim, and it is labelled as one: the
# 20-run series of 18 Sep measured it against ground truth and found it
# optimistic by a constant 15.4 mm (sd 0.8 mm over 13 runs). Reporting it as the
# placement result would be reporting the claim instead of the measurement,
# which is the one thing D-24 exists to prevent.
CLAIMED = [
    ('duration_s', 'Trajanje misije', 's', 1),
    ('place_error_mm', 'Odstupanje odlaganja (robot tvrdi)', 'mm', 1),
    ('tip_left_mm', 'Vrh lijevog alata od cilja', 'mm', 1),
    ('tip_right_mm', 'Vrh desnog alata od cilja', 'mm', 1),
]

# What was RECORDED (metrics.csv, measured from the bag against the run's own
# world). These are the numbers that belong in the report.
#
# The names say what is measured from what. "Zazor pri prelasku sobe" was the
# old label and it misled a reader straight away - it sounds like a doorway
# figure, so 0.70 m in a 1.0 m doorway looks impossible. It is not a doorway
# figure at all: it is the distance from the robot to the nearest TABLE while
# crossing a room, and the report's 0.835 m refers to exactly that.
#
# Both clearances are measured from the robot's CENTRE, because the planned
# path that 0.835 m came from is a path of centre points. The robot is 0.821 m
# wide in DRIVE_V4, so subtract ~0.41 m for the distance from its side.
MEASURED = [
    ('table_clear_transit_min_m',
     'Udaljenost od ruba stola pri prelasku sobe (od centra robota)', 'm', 3),
    ('table_clear_approach_min_m',
     'Udaljenost od ruba stola pri prilazu stolu (od centra robota)', 'm', 3),
    ('wall_clear_open_min_m',
     'Udaljenost od zida u otvorenom prostoru (od centra robota)', 'm', 3),
    ('door_clear_min_m',
     'Udaljenost od dovratnika u prolazu (od centra robota)', 'm', 3),
    ('place_err_truth_mm',
     'Odstupanje odlaganja (izmjereno, ground truth)', 'mm', 1),
    ('amcl_err_max_cm', 'Greska lokalizacije, najveca', 'cm', 1),
    ('amcl_err_rms_cm', 'Greska lokalizacije, RMS', 'cm', 1),
    ('amcl_yaw_max_deg', 'Greska lokalizacije, zakret', 'deg', 1),
    ('path_len_m', 'Duljina puta', 'm', 2),
]

MEASURES = CLAIMED          # kept for callers that still expect the old name


def load(path):
    if not os.path.exists(path):
        print(f'No results at {path}. Run validation/run_batch.py first.', file=sys.stderr)
        return None
    with open(path) as handle:
        return list(csv.DictReader(handle))


def numbers(rows, key, only_success=True):
    out = []
    for row in rows:
        if only_success and row.get('outcome') != 'success':
            continue
        try:
            out.append(float(row[key]))
        except (KeyError, ValueError, TypeError):
            pass
    return out


def describe(values, digits):
    if not values:
        return None
    return {
        'n': len(values),
        'median': round(statistics.median(values), digits),
        'lo': round(min(values), digits),
        'hi': round(max(values), digits),
        'spread': (round(statistics.stdev(values), digits) if len(values) > 1 else None),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv_path', nargs='?', default=None,
                        help='a results.csv to read instead of the batch\'s own')
    parser.add_argument('--batch', default=LATEST,
                        help='batch directory to summarise (default: results/latest)')
    parser.add_argument('--latex', action='store_true', help='also write results_table.tex')
    parser.add_argument('--chart', action='store_true', help='also write results.png')
    args = parser.parse_args()

    batch = args.batch if os.path.isabs(args.batch) else os.path.join(RESULTS, args.batch)
    csv_path = args.csv_path or os.path.join(batch, 'results.csv')
    # Output belongs beside the runs it describes, not in a shared directory
    # where the next batch would quietly overwrite it.
    out_dir = os.path.dirname(os.path.abspath(csv_path))

    rows = load(csv_path)
    # Join the recorded measurements onto the runs they belong to. Without this
    # the only placement figure available is the robot's own claim.
    metrics_path = os.path.join(out_dir, 'metrics.csv')
    measured = {}
    if os.path.exists(metrics_path):
        with open(metrics_path) as handle:
            for row in csv.DictReader(handle):
                measured[str(row.get('run', '')).lstrip('0') or '0'] = row
    if rows:
        for row in rows:
            extra = measured.get(str(row.get('run', '')).lstrip('0') or '0')
            if extra:
                for key, value in extra.items():
                    if key != 'run':
                        row.setdefault(key, value)
    if rows is None:
        return 1
    total = len(rows)
    outcomes = Counter(row.get('outcome', '?') for row in rows)
    ok = outcomes.get('success', 0)
    # A run whose stack never came up, or whose world could not be generated,
    # says nothing about the robot. It is reported, but it is not counted
    # against the mission - and it is not quietly dropped either.
    invalid = outcomes.get('not_ready', 0) + outcomes.get('world_failed', 0)
    valid = total - invalid

    print(f'\nRuns: {total}')
    if valid:
        print(f'Successful: {ok}/{valid} ({100.0 * ok / valid:.0f} %) '
              f'of runs that actually started')
    if invalid:
        print(f'Not counted: {invalid} (stack never came up / world not generated)')
    print('\nIshodi:')
    for name, count in outcomes.most_common():
        print(f'  {name:<14} {count:3d}')

    failed = [r for r in rows
              if r.get('outcome') not in ('success', 'not_ready', 'world_failed')]
    if failed:
        print('\nGdje su neuspjesi stali:')
        for phase, count in Counter(
                (r.get('last_phase') or 'nepoznato') for r in failed).most_common():
            print(f'  {count:3d} x  {phase[:70]}')
        reasons = Counter(r['reason'] for r in failed if r.get('reason'))
        if reasons:
            print('\nRazlozi prekida:')
            for reason, count in reasons.most_common():
                print(f'  {count:3d} x  {reason[:70]}')

    print('\nIZMJERENO iz snimke (samo uspješni runovi; medijan i raspon):')
    measured_summary = []
    for key, label, unit, digits in MEASURED:
        stats = describe(numbers(rows, key), digits)
        if stats is None:
            continue
        spread = f', sd {stats["spread"]}' if stats['spread'] is not None else ''
        print(f'  {label:<62} {stats["median"]} {unit}  '
              f'[{stats["lo"]} – {stats["hi"]}]{spread}   n={stats["n"]}')
        measured_summary.append((label, unit, stats))
    if not measured_summary:
        print('  (nema metrics.csv - pokreni validation/analyze_runs.py)')

    print('\nŠTO JE ROBOT TVRDIO o sebi (iz vlastitog loga):')
    summary = []
    for key, label, unit, digits in MEASURES:
        stats = describe(numbers(rows, key), digits)
        if stats is None:
            print(f'  {label:<32} -')
            continue
        spread = f', sd {stats["spread"]}' if stats['spread'] is not None else ''
        print(f'  {label:<32} {stats["median"]} {unit}  '
              f'[{stats["lo"]} – {stats["hi"]}]{spread}   n={stats["n"]}')
        summary.append((label, unit, stats))

    if args.latex:
        path = os.path.join(out_dir, 'results_table.tex')
        with open(path, 'w') as handle:
            handle.write('% Generated by validation/summarize.py - do not edit by hand.\n')
            handle.write('\\begin{table}[!htb]\n    \\centering\n')
            handle.write(f'    \\caption{{Rezultati {total} uzastopnih izvo\\dj{{}}enja '
                         f'cjelovite misije}}\n')
            handle.write('    \\label{tab:statistika}\n    \\scriptsize\n')
            handle.write('    \\begin{tabularx}{\\textwidth}{@{}X l l l@{}}\n        \\toprule\n')
            handle.write('        \\textbf{Mjera} & \\textbf{Medijan} & '
                         '\\textbf{Raspon} & \\textbf{$N$} \\\\\n        \\midrule\n')
            handle.write(f'        Uspje\\v{{s}}nost & {ok}/{valid} & --- & {valid} \\\\\n')
            for label, unit, stats in measured_summary + summary:
                handle.write(f'        {label} & {stats["median"]} {unit} & '
                             f'{stats["lo"]}--{stats["hi"]} {unit} & {stats["n"]} \\\\\n')
            handle.write('        \\bottomrule\n    \\end{tabularx}\n\\end{table}\n')
        print(f'\nWrote {path}')

    if args.chart:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print('matplotlib is not installed; skipping the chart', file=sys.stderr)
            return 0
        errors = numbers(rows, 'place_error_mm')
        figure, axes = plt.subplots(1, 2, figsize=(9, 3.4))
        if errors:
            axes[0].hist(errors, bins=min(12, max(3, len(errors))),
                         color='#2b6cb0', edgecolor='white')
            axes[0].set_xlabel('odstupanje odlaganja [mm]')
            axes[0].set_ylabel('broj izvođenja')
        axes[0].set_title(f'Točnost odlaganja (n={len(errors)})')

        # Every run on the axis, not just the ones that worked: a duration plot
        # that silently drops the failures is the chart equivalent of quoting
        # only the good run.
        colours = {'success': '#276749'}
        for index, row in enumerate(rows, start=1):
            try:
                duration = float(row['duration_s'])
            except (KeyError, ValueError, TypeError):
                continue
            outcome = row.get('outcome', '?')
            axes[1].plot(index, duration, 'o',
                         color=colours.get(outcome, '#9b2c2c'),
                         label=('uspjeh' if outcome == 'success' else 'neuspjeh'))
        axes[1].set_xlabel('redni broj izvođenja')
        axes[1].set_ylabel('trajanje [s]')
        axes[1].set_title('Trajanje po izvođenju')
        handles, labels = axes[1].get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            axes[1].legend(unique.values(), unique.keys(), fontsize=8)
        for axis in axes:
            axis.grid(alpha=0.3)
        figure.tight_layout()
        path = os.path.join(out_dir, 'results.png')
        figure.savefig(path, dpi=150)
        print(f'Wrote {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
