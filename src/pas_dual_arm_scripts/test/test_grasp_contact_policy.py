import importlib.util
from pathlib import Path

from geometry_msgs.msg import Point, Pose


SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'grasp_cube.py'
SPEC = importlib.util.spec_from_file_location('grasp_cube_policy', SCRIPT)
GRASP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GRASP)


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _Node:
    def __init__(self, counts, positions):
        self.counts = dict(counts)
        self.positions = positions
        self.moves = []
        self.logger = _Logger()

    def box_contact_snapshot(self, max_age):
        del max_age
        return {
            'left_left': self.counts['left'] >= 1,
            'left_right': self.counts['left'] >= 2,
            'right_left': self.counts['right'] >= 1,
            'right_right': self.counts['right'] >= 2,
        }

    def _ee_pose(self, link):
        return self.positions[link]

    def get_logger(self):
        return self.logger

    def approach_both_linear(self, targets, _label, **_kwargs):
        result = {}
        for side, target in targets.items():
            pose = target[2]
            self.positions[f'{side}_end_effector_link'] = pose
            self.moves.append(side)
            self.counts[side] = 2
            result[side] = True
        return result


def _pose(x):
    pose = Pose()
    pose.position = Point(x=x, y=0.0, z=0.8)
    pose.orientation.w = 1.0
    return pose


def test_fine_contact_corrects_incomplete_hands(monkeypatch):
    monkeypatch.setattr(GRASP.rclpy, 'ok', lambda: True)
    node = _Node(
        {'left': 1, 'right': 1},
        {'left_end_effector_link': _pose(0.0),
         'right_end_effector_link': _pose(0.0)})
    contacts = {'left': _pose(0.0), 'right': _pose(0.0)}
    headings = {'left': (1.0, 0.0), 'right': (-1.0, 0.0)}
    limits = {'left': Point(x=0.010), 'right': Point(x=-0.010)}

    assert GRASP.fine_until_four(node, contacts, headings, limits)
    assert node.moves == ['left', 'right']
    assert node.positions['left_end_effector_link'].position.x == 0.002
    assert node.positions['right_end_effector_link'].position.x == -0.002


def test_fine_contact_holds_completed_hand(monkeypatch):
    monkeypatch.setattr(GRASP.rclpy, 'ok', lambda: True)
    node = _Node(
        {'left': 1, 'right': 2},
        {'left_end_effector_link': _pose(0.0),
         'right_end_effector_link': _pose(0.0)})
    contacts = {'left': _pose(0.0), 'right': _pose(0.0)}
    headings = {'left': (1.0, 0.0), 'right': (-1.0, 0.0)}
    limits = {'left': Point(x=0.010), 'right': Point(x=-0.010)}

    assert GRASP.fine_until_four(node, contacts, headings, limits)
    assert node.moves == ['left']
    assert node.positions['left_end_effector_link'].position.x == 0.002
    assert node.positions['right_end_effector_link'].position.x == 0.0


def test_fine_contact_stops_at_post_face_limit(monkeypatch):
    monkeypatch.setattr(GRASP.rclpy, 'ok', lambda: True)
    node = _Node(
        {'left': 1, 'right': 2},
        {'left_end_effector_link': _pose(0.005),
         'right_end_effector_link': _pose(0.0)})
    contacts = {'left': _pose(0.0), 'right': _pose(0.0)}
    headings = {'left': (1.0, 0.0), 'right': (-1.0, 0.0)}
    limits = {'left': Point(x=0.005), 'right': Point(x=-0.005)}

    assert not GRASP.fine_until_four(node, contacts, headings, limits)
    assert node.moves == []
