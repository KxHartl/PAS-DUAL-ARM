#!/usr/bin/env python3
"""Write arm JTC gains derived from the MEASURED effective joint inertia.

The KDL mass-matrix diagonal is not a usable design basis here: for the roll
joints DART behaves as if the arm were ~30x lighter about their own axis (see
notes/03_problemi/P-41). The inertia below is measured instead, by finding the
damping at which a held joint starts to buzz: explicit integration of a pure
damper is stable while d < 2*I/dt, so that boundary gives I directly.

Design: critically damped second order at OMEGA rad/s. Both p and d scale with
the measured inertia, so d/d_max = OMEGA*dt stays constant and safe for every
joint. The integrator is the arm's only gravity compensation, so its clamp is
the actuator's own torque limit.
"""
import argparse
from pathlib import Path

import yaml

DT = 1e-3                      # controller_manager update_rate: 1000 Hz
OMEGA = 150.0                  # rad/s; OMEGA*DT = 0.15 of the damping limit
INTEGRAL_SECONDS = 2.0         # time for the integrator to absorb constant gravity

# Measured on this model at ARM_HOME, both arms symmetric (kg m^2).
INERTIA = {1: 0.0172, 2: None, 3: 0.0118, 4: 0.2011,
           5: 0.0079, 6: 0.0177, 7: 0.00155}
LIMIT = {i: 39.0 if i < 5 else 9.0 for i in range(1, 8)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint2-inertia', type=float, required=True,
                        help='measured effective inertia of joint 2 (kg m^2)')
    parser.add_argument('--config', default='src/pas_dual_arm_bringup/config/force_controllers.yaml')
    args = parser.parse_args()
    inertia = dict(INERTIA)
    inertia[2] = args.joint2_inertia

    path = Path(args.config)
    config = yaml.safe_load(path.read_text())
    for side in ('left', 'right'):
        gains = config[f'{side}_arm_controller']['ros__parameters']['gains']
        for index in range(1, 8):
            value = inertia[index]
            p = value * OMEGA ** 2
            gains[f'{side}_joint_{index}'] = dict(
                p=round(p, 3), i=round(p / INTEGRAL_SECONDS, 3),
                d=round(2 * value * OMEGA, 4), i_clamp=LIMIT[index])
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    print(f'{"joint":>7} {"I [kg m^2]":>11} {"p":>10} {"i":>10} {"d":>9} '
          f'{"d/d_max":>9} {"i_clamp":>8}')
    for index in range(1, 8):
        value = inertia[index]
        p = value * OMEGA ** 2
        print(f'{index:>7} {value:>11.5f} {p:>10.2f} {p/INTEGRAL_SECONDS:>10.2f} '
              f'{2*value*OMEGA:>9.4f} {OMEGA*DT:>9.3f} {LIMIT[index]:>8.1f}')


if __name__ == '__main__':
    main()
