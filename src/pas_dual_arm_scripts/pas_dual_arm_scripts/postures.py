"""Named arm postures, one source of truth for every node that moves the arms.

The values are copied verbatim from the posture registry in `notes/08_poze.md`,
which is where they are *measured*. Do not edit them here without updating that
registry (and vice versa): the width figures the project reasons about come from
`scripts/fit_test.py` / `scripts/mesh_extent.py` runs against these exact numbers.

Postures are stored per side because the arms are not symmetric - the right arm
is mirrored, not copied, so a single dict applied to both sides cannot express
them (see ARM_CARRY_V2 below).
"""

import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# All arm joints at zero: what the robot looks like right after it spawns in
# Gazebo. Width 2.28 m - it does not fit through a 1.0 m doorway, and the arms
# stick out sideways from the side-mounted carriages. Anything that drives the
# base must leave this posture first.
_ZERO = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0}

# Reachable 'ready' posture (SRDF Home state), used as a known-good joint-space
# target. Measured width 1.41 m - also too wide for the doorway.
_HOME = {1: 0.0, 2: 0.26, 3: 3.14, 4: -2.27, 5: 0.0, 6: 0.96, 7: 1.57}

# The posture the user defined by hand in RViz (13. 9. 2026.). Joint values are
# multiples of 45 deg, so it is easy to reproduce. Measured width 85.4 cm by two
# independent methods (a slotted corridor in the MoveIt scene, and mesh vertices
# through TF) - the narrowest validated posture we have, and the only reason the
# robot fits through the 1.0 m doorways at all (7.3 cm clearance per side).
#
# Height still needs work: the torso carriages should set it, which is a separate
# job. Grip width and squeeze need adapting too. See notes/08_poze.md.
_CARRY_V2_LEFT = {1: 0.0, 2: 1.571, 3: 2.356, 4: -1.571, 5: -0.785, 6: 1.571, 7: 1.571}
_CARRY_V2_RIGHT = {1: 0.0, 2: 1.571, 3: 0.785, 4: 1.571, 5: 0.785, 6: -1.571, 7: -1.571}

POSTURES = {
    'ARM_ZERO': {'left': _ZERO, 'right': _ZERO},
    'ARM_HOME': {'left': _HOME, 'right': _HOME},
    'ARM_CARRY_V2': {'left': _CARRY_V2_LEFT, 'right': _CARRY_V2_RIGHT},
}

# The posture to hold whenever the base drives: mapping tours, doorway transits,
# any travel. Named separately so call sites read as intent, not as a pose name.
ARM_DRIVE = 'ARM_CARRY_V2'


def posture_names():
    return sorted(POSTURES)


def joint_constraints(name, label=None, tolerance=0.02):
    """MoveIt Constraints pinning both arms to the named posture.

    Raises KeyError with the list of known names, so a typo fails loudly instead
    of silently leaving the arms wherever they were.
    """
    try:
        sides = POSTURES[name]
    except KeyError:
        raise KeyError(
            f'unknown posture {name!r}; known: {", ".join(posture_names())}') from None

    c = Constraints(name=label or name)
    for side, joints in sides.items():
        for j, val in joints.items():
            jc = JointConstraint()
            jc.joint_name = f'{side}_joint_{j}'
            jc.position = float(val)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            c.joint_constraints.append(jc)
    return c


def switch_arm_hold(node, freeze):
    """Release or reclaim the arm command interfaces around base travel."""
    client = node.create_client(SwitchController, '/controller_manager/switch_controller')
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error('controller_manager switch service unavailable')
            return False
        request = SwitchController.Request()
        names = ['left_arm_controller', 'right_arm_controller']
        if freeze:
            request.deactivate_controllers = names
        else:
            request.activate_controllers = names
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        response = future.result() if future.done() else None
        if response is None or not response.ok:
            node.get_logger().error(
                f'could not {"freeze" if freeze else "release"} arm hold')
            return False
        node._arms_frozen = freeze
        node.get_logger().info('arm controllers ' + ('released for travel' if freeze else 'reactivated'))
        return True
    finally:
        node.destroy_client(client)


def move_to_posture(node, name, timeout=30.0, label=None, freeze=False):
    """Plan and execute both arms to the named posture. Blocks; returns success.

    Goes through MoveIt (`both_arms`) rather than commanding the joint trajectory
    controllers directly: leaving the spawn posture is a large motion and has to
    be checked against the robot's own geometry first.
    """
    # A mapping-ready simulation can spawn at the existing measured carry pose.
    # Do not swing both arms through the world merely to command the pose they
    # already occupy; that motion has destabilized DART and flung the box.
    initial = None
    def initial_cb(msg):
        nonlocal initial
        initial = msg
    initial_sub = node.create_subscription(JointState, '/joint_states', initial_cb, 10)
    try:
        deadline = time.monotonic() + 3.0
        while initial is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(initial_sub)
    if initial is not None:
        measured = dict(zip(initial.name, initial.position))
        errors = [abs((measured[f'{side}_joint_{index}'] - target
                       + 3.141592653589793) % (2 * 3.141592653589793)
                      - 3.141592653589793)
                  for side, joints in POSTURES[name].items()
                  for index, target in joints.items()
                  if f'{side}_joint_{index}' in measured]
        if len(errors) == 14 and max(errors) <= 0.10:
            node.get_logger().info(f'{name}: already at measured posture; no arm motion')
            if not freeze:
                return True
            if not switch_arm_hold(node, True):
                return False
            # Continue to the physical verification below after freeze.
            return _verify_posture(node, name, freeze=True)

    client = ActionClient(node, MoveGroup, '/move_action')
    has_moveit = client.wait_for_server(timeout_sec=2.0)
    if not has_moveit:
        node.get_logger().info(
            f'/move_action unavailable; moving both arms to {name} directly via JointTrajectoryController')
        ok = _move_via_jtc(node, name)
        if not ok:
            return False
        if freeze and not switch_arm_hold(node, True):
            return False
        return _verify_posture(node, name, freeze=freeze)

    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = 'both_arms'
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.2
    req.max_acceleration_scaling_factor = 0.2
    req.goal_constraints.append(joint_constraints(name, label))
    goal.planning_options.plan_only = False

    node.get_logger().info(f'planning both_arms -> {name}')
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        node.get_logger().error(f'{name}: goal rejected')
        return False
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
    if not ok:
        node.get_logger().error(f'{name}: MoveIt execution failed')
        return False
    if freeze and not switch_arm_hold(node, True):
        return False

    return _verify_posture(node, name, freeze=freeze)


def _move_via_jtc(node, name, duration_sec=4.0):
    """Direct trajectory execution to arm controllers without MoveIt."""
    try:
        sides = POSTURES[name]
    except KeyError:
        raise KeyError(
            f'unknown posture {name!r}; known: {", ".join(posture_names())}') from None

    clients = {}
    for side in ('left', 'right'):
        ac = ActionClient(
            node, FollowJointTrajectory, f'/{side}_arm_controller/follow_joint_trajectory')
        if not ac.wait_for_server(timeout_sec=5.0):
            node.get_logger().error(f'/{side}_arm_controller/follow_joint_trajectory unavailable')
            return False
        clients[side] = ac

    handles = []
    for side in ('left', 'right'):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'{side}_joint_{j}' for j in range(1, 8)]
        pt = JointTrajectoryPoint()
        pt.positions = [float(sides[side][j]) for j in range(1, 8)]
        pt.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points.append(pt)

        future = clients[side].send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            node.get_logger().error(f'Direct JTC goal rejected on {side}_arm_controller')
            return False
        handles.append(handle)

    all_ok = True
    for handle in handles:
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_future, timeout_sec=duration_sec + 5.0)
        result = res_future.result() if res_future.done() else None
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            all_ok = False

    return all_ok


def _verify_posture(node, name, freeze=False):
    """Verify the physical joint state, not the controller action result."""

    # An action result is not proof that Gazebo's soft position interfaces are
    # physically holding the arm.  The old carry pose sagged by >1 rad at the
    # wrists while driving, making the robot wider than the doorway.
    latest = None

    def joint_cb(msg):
        nonlocal latest
        latest = msg

    sub = node.create_subscription(JointState, '/joint_states', joint_cb, 10)
    try:
        deadline = time.monotonic() + 8.0
        stable_since = None
        worst_joint, worst_error = 'joint_states', float('inf')
        while time.monotonic() < deadline:
            latest = None
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest is None:
                continue
            measured = dict(zip(latest.name, latest.position))
            errors = []
            for side, joints in POSTURES[name].items():
                for index, target in joints.items():
                    joint = f'{side}_joint_{index}'
                    if joint not in measured:
                        errors.append((joint, float('inf')))
                    else:
                        error = (measured[joint] - target + 3.141592653589793) % (
                            2 * 3.141592653589793) - 3.141592653589793
                        errors.append((joint, abs(error)))
            worst_joint, worst_error = max(errors, key=lambda pair: pair[1])
            if worst_error <= 0.10:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 3.0:
                    node.get_logger().info(f'{name}: stable measured posture OK ({worst_error:.3f} rad max)')
                    return True
            else:
                stable_since = None
        node.get_logger().error(
            f'{name}: {worst_joint} differs by '
            f'{worst_error:.3f} rad; unsafe to drive')
        if freeze:
            switch_arm_hold(node, False)
        return False
    finally:
        node.destroy_subscription(sub)
