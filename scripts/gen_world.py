#!/usr/bin/env python3
"""Generate the Gazebo world from config/world.yaml.

Why a generator at all: every claim in this project about doorways, clearances
and navigation holds for *one* hand-written room layout. Being able to change
the layout is what turns "it worked" into "it works", and it is the thing the
validation batch needs in order to vary anything.

**This is not a from-scratch writer.** It loads the checked-in world as a
template and replaces only the geometry that config/world.yaml describes - the
floors, the walls with their openings, and the poses of the table, box and
marker. Physics settings, the system plugins (contact and force-torque in
particular), and the table/box/marker models with their friction, inertia and
marker textures are carried over untouched. A number in the YAML therefore
cannot quietly change how the box behaves in a grasp.

    ./scripts/run_native.sh python3 scripts/gen_world.py
    ./scripts/run_native.sh python3 scripts/gen_world.py --set 'doors.0.width=0.95'
    ./scripts/run_native.sh python3 scripts/gen_world.py --cube-jitter 0.05 --seed 3

A generated layout needs its own map; drive one of the mapping scenarios once.
"""
import argparse
import os
import random
import sys
import xml.etree.ElementTree as ET

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRINGUP = os.path.join(REPO, 'src', 'pas_dual_arm_bringup')
TEMPLATE = os.path.join(BRINGUP, 'worlds', 'seminar_world.sdf')
CONFIG = os.path.join(BRINGUP, 'config', 'world.yaml')
DEFAULT_OUT = os.path.join(BRINGUP, 'worlds', 'generated_world.sdf')

# Narrower than this and the robot (0.821 m across the tucked arms) does not fit
# at all, so the world would be unsolvable rather than hard.
MIN_DOOR = 0.95
WALL_RGB = '0.6 0.6 0.6 1'


def room_centre(cfg, name):
    cell = cfg['rooms'][name]['cell']
    width, height = cfg['room_size']
    return (cell[0] * width, cell[1] * height)


def table_centre(cfg, name):
    spec = cfg['tables'][name]
    cx, cy = room_centre(cfg, spec['room'])
    return (cx + spec['offset'][0], cy + spec['offset'][1])


def box_link(name, pose, size):
    """One wall slab: collision and visual of the same box."""
    link = ET.Element('link', {'name': name})
    ET.SubElement(link, 'pose').text = pose
    for tag in ('collision', 'visual'):
        node = ET.SubElement(link, tag, {'name': 'c' if tag == 'collision' else 'v'})
        geometry = ET.SubElement(node, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = size
        if tag == 'visual':
            material = ET.SubElement(node, 'material')
            ET.SubElement(material, 'ambient').text = WALL_RGB
            ET.SubElement(material, 'diffuse').text = WALL_RGB
    return link


def wall_edges(cfg):
    """Every wall edge in the layout, deduplicated where two rooms share one.

    An edge is keyed by its axis and position so the wall between two adjacent
    rooms is emitted once, not twice in the same place - overlapping collision
    slabs are a classic source of physics jitter.
    """
    width, height = cfg['room_size']
    edges = {}
    for name in cfg['rooms']:
        cx, cy = room_centre(cfg, name)
        for axis, position, span_centre in (
                ('x', cy + height / 2.0, cx),      # wall running along X, at y = ...
                ('x', cy - height / 2.0, cx),
                ('y', cx + width / 2.0, cy),       # wall running along Y, at x = ...
                ('y', cx - width / 2.0, cy)):
            key = (axis, round(position, 3), round(span_centre, 3))
            edges.setdefault(key, []).append(name)
    return edges


def door_for(cfg, rooms_on_edge):
    """The opening declared between these two rooms, if any."""
    if len(rooms_on_edge) < 2:
        return None
    pair = set(rooms_on_edge)
    for door in cfg.get('doors', []):
        if set(door['between']) == pair:
            return door
    return None


def build_rooms_model(cfg):
    """Floors and walls, with openings where doors are declared."""
    width, height = cfg['room_size']
    thickness = cfg['wall']['thickness']
    wall_h = cfg['wall']['height']
    # +thickness so the corners close instead of leaving a pinhole.
    full = {'x': width + thickness, 'y': height + thickness}

    floors = ET.Element('model', {'name': 'room_floors'})
    ET.SubElement(floors, 'static').text = 'true'
    floors_link = ET.SubElement(floors, 'link', {'name': 'floors'})
    for name, spec in cfg['rooms'].items():
        cx, cy = room_centre(cfg, name)
        visual = ET.SubElement(floors_link, 'visual', {'name': f'{name}_floor'})
        ET.SubElement(visual, 'pose').text = f'{cx:g} {cy:g} 0.001 0 0 0'
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = \
            f'{width:g} {height:g} 0.002'
        colour = ' '.join(f'{c:g}' for c in spec['colour']) + ' 1'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = colour
        ET.SubElement(material, 'diffuse').text = colour

    rooms = ET.Element('model', {'name': 'rooms'})
    ET.SubElement(rooms, 'static').text = 'true'
    index = 0
    for (axis, position, span_centre), on_edge in sorted(wall_edges(cfg).items()):
        door = door_for(cfg, on_edge)
        span = full[axis]
        half = span / 2.0
        if door is None:
            pieces = [(span_centre, span)]
            label = f'wall_{index}'
        else:
            gap = float(door['width'])
            piece = half - gap / 2.0
            pieces = [(span_centre - (half + gap / 2.0) / 2.0, piece),
                      (span_centre + (half + gap / 2.0) / 2.0, piece)]
            label = 'door_' + '_'.join(sorted(on_edge))
        for side, (centre, length) in enumerate(pieces):
            if length <= 0.0:
                continue
            if axis == 'x':                       # wall runs along X, sits at y=position
                pose = f'{centre:g} {position:g} {wall_h / 2.0:g} 0 0 0'
                size = f'{length:g} {thickness:g} {wall_h:g}'
            else:                                 # wall runs along Y, sits at x=position
                pose = f'{position:g} {centre:g} {wall_h / 2.0:g} 0 0 0'
                size = f'{thickness:g} {length:g} {wall_h:g}'
            rooms.append(box_link(f'{label}_{side}', pose, size))
        index += 1
    return floors, rooms


def find_model(root, name):
    for model in root.iter('model'):
        if model.get('name') == name:
            return model
    return None


def set_pose(model, x, y, z=0.0, yaw=0.0):
    pose = model.find('pose')
    if pose is None:
        pose = ET.Element('pose')
        model.insert(1, pose)
    pose.text = f'{x:g} {y:g} {z:g} 0 0 {yaw:g}'


def apply_overrides(cfg, overrides):
    """--set 'doors.0.width=0.95' style edits, so a sweep needs no file editing."""
    for item in overrides:
        path, _, raw = item.partition('=')
        node = cfg
        keys = path.split('.')
        for key in keys[:-1]:
            node = node[int(key)] if isinstance(node, list) else node[key]
        last = keys[-1]
        value = yaml.safe_load(raw)
        if isinstance(node, list):
            node[int(last)] = value
        else:
            node[last] = value
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', default=CONFIG)
    parser.add_argument('--template', default=TEMPLATE)
    parser.add_argument('--out', default=DEFAULT_OUT)
    parser.add_argument('--set', action='append', default=[], metavar='PATH=VALUE',
                        help="override a config value, e.g. --set 'doors.0.width=0.95'")
    parser.add_argument('--cube-jitter', type=float, default=0.0, metavar='M',
                        help='randomise the box position by up to this many metres; '
                             'used by the validation batch so repeats are not identical')
    parser.add_argument('--seed', type=int, default=None)
    args = parser.parse_args()

    with open(args.config, encoding='utf-8') as handle:
        cfg = apply_overrides(yaml.safe_load(handle), args.set)

    for door in cfg.get('doors', []):
        if float(door['width']) < MIN_DOOR:
            print(f'ERROR: door {door["between"]} is {door["width"]} m; the robot is '
                  f'0.821 m wide and needs at least {MIN_DOOR} m to pass at all.',
                  file=sys.stderr)
            return 1

    tree = ET.parse(args.template)
    root = tree.getroot()
    world = root.find('world')
    if world.get('name') != cfg['world_name']:
        print(f'ERROR: template world is "{world.get("name")}" but the config asks for '
              f'"{cfg["world_name"]}". bridge.yaml embeds the world name in the contact '
              f'sensor topics, so they must agree.', file=sys.stderr)
        return 1

    # Replace floors and walls with the generated geometry, in place so the
    # surrounding models keep their order.
    new_floors, new_rooms = build_rooms_model(cfg)
    children = list(world)
    for old_name, replacement in (('room_floors', new_floors), ('rooms', new_rooms)):
        old = find_model(world, old_name)
        if old is None:
            world.append(replacement)
            continue
        world.insert(children.index(old), replacement)
        world.remove(old)
        children = list(world)

    # Tables, box and marker: same models, new places.
    for name in cfg['tables']:
        model = find_model(world, name)
        if model is None:
            print(f'ERROR: template has no model "{name}"', file=sys.stderr)
            return 1
        set_pose(model, *table_centre(cfg, name))

    rng = random.Random(args.seed)
    cube_cfg = cfg['cube']
    cx, cy = table_centre(cfg, cube_cfg['table'])
    bx = cx + cube_cfg['offset'][0]
    by = cy + cube_cfg['offset'][1]
    if args.cube_jitter > 0.0:
        bx += rng.uniform(-args.cube_jitter, args.cube_jitter)
        by += rng.uniform(-args.cube_jitter, args.cube_jitter)
    set_pose(find_model(world, 'aruco_box'), bx, by, cube_cfg['z'], cube_cfg['yaw'])

    marker_cfg = cfg['marker']
    mx, my = table_centre(cfg, marker_cfg['table'])
    set_pose(find_model(world, 'place_marker'),
             mx + marker_cfg['offset'][0], my + marker_cfg['offset'][1], marker_cfg['z'])

    ET.indent(tree, space='  ')
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    tree.write(args.out, encoding='unicode', xml_declaration=True)
    print(f'Wrote {args.out}')
    print(f'  rooms   : {", ".join(cfg["rooms"])}')
    for door in cfg.get('doors', []):
        print(f'  doorway : {" <-> ".join(door["between"])}, {door["width"]:.2f} m')
    print(f'  box at  : ({bx:.3f}, {by:.3f})'
          + (f'  [jittered by up to {args.cube_jitter} m]' if args.cube_jitter else ''))
    print('\nThis layout needs its own map:')
    print('  ros2 launch pas_dual_arm_bringup scenario_manual_map.launch.py '
          f'  # after pointing sim at {os.path.basename(args.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
