"""Independent mechanics and fail-closed control tests."""
import numpy as np
import pytest

from pas_dual_arm_scripts.force_model import estimate
from pas_dual_arm_scripts.force_model import ArmModel
from pas_dual_arm_scripts.force_grasp import squeeze_steps


def jacobian():
    return np.random.default_rng(12).normal(size=(6, 7))


def test_external_force_sign_and_bias():
    j = jacobian()
    g = np.arange(7, dtype=float)
    bias = np.full(7, 0.03)
    external = np.array([4., -5., 1., .2, .3, -.1])
    measured = g - j.T @ external + bias
    recovered, _, _ = estimate(j, g, measured, bias)
    np.testing.assert_allclose(recovered, external, atol=0.002)


def test_gravity_is_not_contact():
    j = jacobian()
    g = np.arange(7, dtype=float)
    np.testing.assert_allclose(estimate(j, g, g, np.zeros(7))[0], 0., atol=1e-12)


def test_singular_pose_and_invalid_effort_are_rejected():
    with pytest.raises(ValueError, match='ill-conditioned'):
        estimate(np.zeros((6, 7)), np.zeros(7), np.zeros(7), np.zeros(7))
    with pytest.raises(ValueError, match='non-finite'):
        estimate(jacobian(), np.zeros(7), np.full(7, np.nan), np.zeros(7))


def test_unexplained_joint_torque_is_rejected():
    j = jacobian()
    _, _, vt = np.linalg.svd(j)
    with pytest.raises(ValueError, match='residual'):
        estimate(j, np.zeros(7), vt[-1]*10, np.zeros(7))


def test_balanced_hold_and_force_limits():
    np.testing.assert_allclose(squeeze_steps([5, 5], 5, .0003, .25), 0.)
    steps = squeeze_steps([1, 12], 5, .0003, .25)
    assert steps[0] > 0 and steps[1] < 0
    assert max(abs(steps)) <= .0005
    with pytest.raises(ValueError):
        squeeze_steps([np.nan, 5], 5, .0003, .25)


def test_kdl_gravity_matches_potential_energy_with_off_chain_payload():
    import PyKDL as kdl
    # Independently differentiate gravitational potential, including an
    # off-chain mass with a rotated inertial frame (the open gripper case).
    xml = '<robot name="test"><link name="left_base_link"/>'
    previous = 'left_base_link'
    for i in range(1, 8):
        child = f'link{i}'
        xml += f'''<link name="{child}"><inertial><mass value="0.2"/>
          <origin xyz="0.04 0.01 0.02"/><inertia ixx=".01" iyy=".02" izz=".03"
          ixy="0" ixz="0" iyz="0"/></inertial></link>
          <joint name="left_joint_{i}" type="continuous"><parent link="{previous}"/>
          <child link="{child}"/><origin xyz=".05 .02 .03" rpy=".3 .5 .2"/>
          <axis xyz="0 0 1"/></joint>'''
        previous = child
    xml += '''<link name="left_end_effector_link"/>
      <joint name="tip" type="fixed"><parent link="link7"/>
      <child link="left_end_effector_link"/></joint>
      <link name="payload"><inertial><origin xyz=".1 .2 .05" rpy=".4 .2 .1"/>
      <mass value="1.2"/><inertia ixx=".01" iyy=".02" izz=".03"
      ixy="0" ixz="0" iyz="0"/></inertial></link>
      <joint name="payload_joint" type="fixed"><parent link="left_end_effector_link"/>
      <child link="payload"/><origin xyz=".05 0 .1" rpy=".5 .2 .1"/></joint></robot>'''
    model = ArmModel(xml, 'left')
    gravity = np.array([1., -3., -9.])
    positions = np.arange(7)*.1

    def potential(q):
        pose, energy, index = kdl.Frame.Identity(), 0., 0
        for n in range(model.chain.getNrOfSegments()):
            segment = model.chain.getSegment(n)
            movable = segment.getJoint().getType() != kdl.Joint.Fixed
            pose = pose * segment.pose(float(q[index]) if movable else 0.)
            index += int(movable)
            inertia = segment.getInertia()
            p = pose * inertia.getCOG()
            energy -= inertia.getMass() * np.dot(gravity, [p[0], p[1], p[2]])
        return energy
    expected = []
    for i in range(7):
        delta = np.zeros(7)
        delta[i] = 1e-6
        expected.append((potential(positions+delta)-potential(positions-delta))/2e-6)
    np.testing.assert_allclose(model.terms(positions, gravity)[1], expected, atol=1e-7)
    mass = sum(model.chain.getSegment(i).getInertia().getMass()
               for i in range(model.chain.getNrOfSegments()))
    assert mass == pytest.approx(2.6)


def _qualifier():
    """Import the offline qualifier, which lives with the scripts, not the package."""
    import importlib.util
    from pathlib import Path
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        'qualify_force_log', root / 'scripts' / 'qualify_force_log.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_log(tmp_path, records):
    import json
    path = tmp_path / 'log.jsonl'
    path.write_text(''.join(json.dumps(r) + '\n' for r in records))
    return path


def _status(stamp, raw):
    arm = dict(valid=True, stamp=stamp, raw=list(raw) + [0., 0., 0.],
               world_to_base=[0., 0., 0., 1.])
    return dict(stamp=stamp, kind='estimator_status',
                data=dict(stamp=stamp, fingerprint='f',
                          arms={'left': dict(arm), 'right': dict(arm)}))


def _touch(stamp, pad, force, other=0):
    contacts = [] if force is None else [dict(forces=[list(force)])]
    return dict(stamp=stamp, kind='contact',
                data=dict(pad=pad, contacts=contacts, other=other))


def _ft(stamp, side, force):
    return dict(stamp=stamp, kind='wrist_ft',
                data=dict(side=side, force=list(force), torque=[0., 0., 0.]))


PADS = ['left_left', 'left_right', 'right_left', 'right_right']


def test_silence_is_not_free_space_without_a_live_contact_sensor(tmp_path):
    # A pad that never reports anything could be untouched or unbridged. With no
    # proof the sensors work, silence must not be scored as verified free space.
    qualify = _qualifier().qualify
    records = [_status(0.1 * i, [0., 0., 0.]) for i in range(200)]
    report = qualify(_write_log(tmp_path, records), target=5.0)
    assert report['contact_sensors_live'] is False
    assert report['passed'] is False
    assert report['metrics']['left']['free_samples'] == 0


def test_silence_counts_as_free_once_every_pad_has_reported(tmp_path):
    qualify = _qualifier().qualify
    records = [_status(0.02 * i, [0., 0., 0.]) for i in range(300)]
    # One late burst of real contact, far past the free window's guard band.
    for step in range(60):
        for pad in PADS:
            records.append(_touch(20.0 + 0.02 * step, pad, [2.5, 0., 0.]))
    report = qualify(_write_log(tmp_path, records), target=5.0)
    assert report['contact_sensors_live'] is True
    assert report['metrics']['left']['free_samples'] == 300


def test_guard_band_keeps_the_edge_of_a_touch_out_of_free_space(tmp_path):
    qualify = _qualifier().qualify
    records = [_status(0.02 * i, [0., 0., 0.]) for i in range(300)]
    for pad in PADS:                      # a touch in the middle of the sweep
        records.append(_touch(3.0, pad, [2.5, 0., 0.]))
    report = qualify(_write_log(tmp_path, records), target=5.0)
    free = report['metrics']['left']['free_samples']
    # 0.3 s of guard on each side of the touch removes those samples, no more.
    assert 260 <= free <= 285


def _loaded_log(estimated, reference, steps=200, other_pad=None):
    """A free stretch, then a squeeze with the wrist FT carrying `reference` N."""
    records = []
    for step in range(200):                      # verified free space
        stamp = 6.02 + 0.02 * step               # runs right up to the squeeze
        records.append(_status(stamp, [0., 0., 0.]))
        for side in ('left', 'right'):
            records.append(_ft(stamp, side, [10., 0., 0.]))   # gripper weight
    for step in range(steps):                    # squeeze
        stamp = 10.02 + 0.02 * step
        records.append(_status(stamp, [estimated, 0., 0.]))
        for side in ('left', 'right'):
            records.append(_ft(stamp, side, [10. + reference, 0., 0.]))
        for pad in PADS:
            records.append(_touch(stamp, pad, [2.5, 0., 0.],
                                  other=1 if pad == other_pad else 0))
    return records


def test_a_pad_pressed_by_something_other_than_the_box_is_not_scored(tmp_path):
    # A non-box contact on the RIGHT hand leaves the LEFT hand's own comparison
    # against its own wrist sensor intact, but it does invalidate the balance
    # check and the right hand's own samples, so the report cannot pass.
    qualify = _qualifier().qualify
    records = _loaded_log(estimated=5.0, reference=5.0, other_pad='right_left')
    report = qualify(_write_log(tmp_path, records), target=5.0)
    assert report['metrics']['left']['loaded_samples'] > 0
    assert report['metrics']['right']['loaded_samples'] == 0
    assert report['metrics']['balance']['samples'] == 0
    assert report['passed'] is False


def test_an_estimate_matching_the_wrist_force_torque_sensor_passes(tmp_path):
    qualify = _qualifier().qualify
    records = _loaded_log(estimated=5.0, reference=5.0)
    report = qualify(_write_log(tmp_path, records), target=5.0)
    assert report['metrics']['left']['loaded_samples'] >= 50
    assert report['metrics']['left']['ft_error_p95'] < 0.1
    assert report['metrics']['balance']['p95'] < 0.1
    assert report['passed'] is True


def test_an_estimate_that_disagrees_with_the_sensor_fails(tmp_path):
    # Both arms are wrong in the SAME way, so the balance check cannot see it.
    # This is exactly why an absolute reference is required as well.
    qualify = _qualifier().qualify
    records = _loaded_log(estimated=3.0, reference=5.0)
    report = qualify(_write_log(tmp_path, records), target=5.0)
    assert report['metrics']['balance']['p95'] < 0.1
    assert report['metrics']['left']['ft_error_p95'] > 1.0
    assert report['passed'] is False


def test_resolved_rate_stays_on_the_current_branch():
    """A half-millimetre request must move the joints by a half-millimetre worth.

    This is the property a global IK solver did not have: seeded with the current
    state it still returned a configuration 3 rad away (P-25).
    """
    from pas_dual_arm_scripts.force_model import resolved_rate
    rng = np.random.default_rng(7)
    for _ in range(20):
        jacobian = rng.normal(size=(6, 7))
        want = np.concatenate([rng.normal(size=3) * 5e-4, rng.normal(size=3) * 1e-3])
        step = resolved_rate(jacobian, want)
        assert np.max(np.abs(step)) < 0.02, 'a sub-millimetre step must stay small'
        # Damping deliberately gives up some of the requested motion near a
        # singularity; measured worst case at this damping is 5.6%.
        shortfall = np.linalg.norm(jacobian @ step - want) / np.linalg.norm(want)
        assert shortfall < 0.10, f'lost {shortfall:.1%} of the requested motion'


def test_resolved_rate_refuses_a_step_it_cannot_take_safely():
    from pas_dual_arm_scripts.force_model import resolved_rate
    jacobian = np.random.default_rng(11).normal(size=(6, 7))
    with pytest.raises(ValueError, match='exceeds'):
        resolved_rate(jacobian, np.array([1.0, 0., 0., 0., 0., 0.]))
    with pytest.raises(ValueError, match='non-finite'):
        resolved_rate(jacobian, np.array([np.nan, 0., 0., 0., 0., 0.]))
