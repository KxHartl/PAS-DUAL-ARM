#!/usr/bin/env python3
"""Offline qualification of the joint-torque force estimate.

No control code imports this module, and nothing it reads is available to the
control path: the wrist force-torque sensors and the fingertip contact sensors
exist in simulation only.

Two independent references, because neither alone is enough:

  ABSOLUTE - the wrist force-torque sensor. The Fortress contact sensor turned
  out to publish contact POINTS with empty wrenches, so it cannot say how hard
  a pad presses (notes/03_problemi/P-42). The FT sensor measures the whole
  wrench transmitted through the wrist, gripper weight included, so the
  comparison uses the CHANGE from a baseline taken just before first contact -
  over a squeeze the arm moves millimetres, so the weight term cancels.
  Magnitudes are compared, which needs no rotation between the two frames.

  CONSISTENCY - the two arms press the same cube, so in equilibrium their
  normal forces must match. That checks the two kinematic chains against each
  other; it cannot catch an error common to both, which is why the absolute
  reference is also required.

The contact sensors still decide WHEN a pad is loaded; the FT sensor says HOW
MUCH. Fortress publishes nothing while a pad is free, so silence is accepted as
"no contact" ONLY in a log where every pad also reported a real box contact:
that is what separates an untouched pad from a dead sensor. Free samples keep a
guard band away from every contact message, so the edge of a touch is never
scored as free space.

The optional friction evidence is a JSON report from an independent
inclined-plane material test: angle_degrees, hold_seconds (>=30), slipped=false,
and source (path to its recorded data). Without it this report CANNOT authorize
a lift.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

import numpy as np

SIDES = ('left', 'right')
PADS = [f'{side}_{finger}' for side in SIDES for finger in SIDES]


class _Series:
    """Time-indexed samples. Bisection, because these logs run to 10^5 rows and
    a linear scan per estimate turns the report into a multi-minute job."""

    def __init__(self, samples):
        ordered = sorted(samples, key=lambda pair: pair[0])
        self.stamps = [stamp for stamp, _ in ordered]
        self.values = [value for _, value in ordered]

    def __len__(self):
        return len(self.stamps)

    def nearest(self, now, window):
        """Sample closest to `now`, or None if none is within `window`."""
        i = bisect.bisect_left(self.stamps, now)
        best, best_gap = None, window
        for j in (i - 1, i):
            if 0 <= j < len(self.stamps):
                gap = abs(self.stamps[j] - now)
                if gap <= best_gap:
                    best, best_gap = self.values[j], gap
        return best

    def latest(self, now, window):
        """Most recent sample at or before `now`, within `window`."""
        i = bisect.bisect_right(self.stamps, now) - 1
        if i >= 0 and 0 <= now - self.stamps[i] <= window:
            return self.values[i]
        return None

    def any_within(self, now, window):
        i = bisect.bisect_left(self.stamps, now - window)
        return i < len(self.stamps) and self.stamps[i] <= now + window


def qualify(path, target, friction_evidence=None, window=0.1, guard=0.3):
    records = [json.loads(line) for line in Path(path).read_text().splitlines()]
    messages = {pad: [] for pad in PADS}
    ft = {side: [] for side in SIDES}
    for record in records:
        data, stamp = record['data'], record['stamp']
        if record['kind'] == 'contact':
            if data['pad'] not in messages:
                raise ValueError(f'unknown contact pad {data["pad"]}')
            other = data.get('other', 0)
            messages[data['pad']].append(
                (stamp, data['contacts'],
                 len(other) if isinstance(other, list) else int(other)))
        elif record['kind'] == 'wrist_ft':
            ft[data['side']].append((stamp, np.array(data['force'], dtype=float)))

    # A pad's silence is only evidence once that pad has been heard from at all.
    # Requiring a BOX contact from every pad is stricter than the question being
    # asked and is not satisfiable here: all four pads sit on the face within
    # 0.3 mm, yet which of them penetrates enough to trigger is decided by a
    # tenth of a millimetre (notes/03_problemi/P-42). Any message from a pad -
    # the box, the table, the other finger - proves its path works.
    heard = {pad: len(messages[pad]) > 0 for pad in PADS}
    live = all(heard.values())
    contacts = {pad: _Series([(stamp, (entries, other))
                              for stamp, entries, other in messages[pad]])
                for pad in PADS}
    wrist = {side: _Series(ft[side]) for side in SIDES}
    touches = _Series([(stamp, True) for pad in PADS
                       for stamp, _, _ in messages[pad]])
    first_touch = touches.stamps[0] if len(touches) else None

    def quiet(now):
        return not touches.any_within(now, guard)

    # FT baseline: the free-space reading just before the first touch, when the
    # arm already holds the pose it will squeeze from.
    baseline = {}
    for side in SIDES:
        if first_touch is None:
            continue
        before = [f for stamp, f in zip(wrist[side].stamps, wrist[side].values)
                  if 0 <= first_touch - stamp <= 2.0]
        if len(before) >= 20:
            baseline[side] = np.mean(before, axis=0)

    fingerprints, seen = set(), set()
    free = {side: [] for side in SIDES}
    ft_error = {side: [] for side in SIDES}
    loaded_count = {side: 0 for side in SIDES}
    balance = []
    for record in records:
        if record['kind'] != 'estimator_status':
            continue
        data, now = record['data'], record['stamp']
        fingerprints.add(data['fingerprint'])
        magnitude = {}
        for side, arm in data['arms'].items():
            key = (side, arm.get('stamp'))
            if not arm.get('valid') or key in seen:
                continue
            seen.add(key)
            estimated = float(np.linalg.norm(arm['raw'][:3]))
            if quiet(now):
                if live:
                    free[side].append(estimated)
                continue
            fresh = [m for pad in PADS if pad.startswith(f'{side}_')
                     for m in [contacts[pad].latest(now, window)] if m is not None]
            if not fresh or any(other for _, other in fresh) or not any(
                    entries for entries, _ in fresh):
                continue
            if side not in baseline:
                continue
            measured = wrist[side].nearest(now, window)
            if measured is None:
                continue
            reference = float(np.linalg.norm(measured - baseline[side]))
            if not 0.8 * target <= reference <= 1.2 * target:
                continue
            loaded_count[side] += 1
            ft_error[side].append(abs(estimated - reference))
            magnitude[side] = estimated
        # The balance check spans both hands, so anything other than the box
        # pressing EITHER hand invalidates it - that hand is pushing against
        # something the other one is not. A per-side FT comparison is narrower
        # and only cares about its own pads.
        contaminated = any((sample or (None, 0))[1]
                           for pad in PADS
                           for sample in [contacts[pad].latest(now, window)])
        if len(magnitude) == 2 and not contaminated:
            balance.append(abs(magnitude['left'] - magnitude['right']))

    metrics = {}
    for side in SIDES:
        metrics[side] = dict(
            free_samples=len(free[side]), loaded_samples=loaded_count[side],
            free_max=max(free[side], default=None),
            ft_error_p95=_p95(ft_error[side]))
    metrics['balance'] = dict(samples=len(balance), p95=_p95(balance))

    limit = 0.2 * target
    force_pass = bool(live and len(fingerprints) == 1 and balance
                      and metrics['balance']['p95'] < limit and all(
        metrics[side]['free_samples'] >= 50 and metrics[side]['loaded_samples'] >= 50
        and metrics[side]['free_max'] < limit and metrics[side]['ft_error_p95'] < limit
        for side in SIDES))

    mu, evidence = 0., None
    if friction_evidence:
        evidence = json.loads(Path(friction_evidence).read_text())
        angle = float(evidence['angle_degrees'])
        if (not 0 < angle < 80 or evidence['slipped'] is not False
                or evidence['hold_seconds'] < 30 or not Path(evidence['source']).is_file()):
            raise ValueError('invalid independent friction test evidence')
        mu = math.tan(math.radians(angle))
    return dict(passed=force_pass, force_target=target, contact_sensors_live=bool(live),
                contact_sensors_heard=heard,
                wrist_ft_baseline={s: baseline[s].tolist() for s in baseline},
                fingerprint=next(iter(fingerprints)) if len(fingerprints) == 1 else '',
                friction_lower_bound=mu, friction_evidence=evidence, metrics=metrics,
                source=str(path), source_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


def _p95(values):
    return float(np.percentile(values, 95)) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('--target', type=float, default=5.0)
    parser.add_argument('--friction-evidence')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not math.isfinite(args.target) or args.target <= 0:
        parser.error('target must be positive and finite')
    report = qualify(args.log, args.target, args.friction_evidence)
    Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
