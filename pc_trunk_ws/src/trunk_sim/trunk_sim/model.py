"""Builds the MuJoCo model of the trunk as text (MJCF).

Pseudo-rigid-body model: the soft flex is a chain of short rigid slices.
Each slice has the same three joints as in the URDF, and same names:
    <unit>_seg<i>_squeeze   slides closer to the slice before   (compression)
    <unit>_seg<i>_bend_x    tilts about x                        (bending)
    <unit>_seg<i>_bend_y    tilts about y                        (bending)
Springs on these joints stand in for the stiffness of the TPU lattice.
Three tendons run through small rings (sites) on every slice and end at
the tip, like the real cables.
"""

import math

UNITS = ('unit1', 'unit2')
SLICES_PER_UNIT = 8

# Dimensions in metres, the same as in the URDF.
POST_DEPTH = 0.005
POST_LENGTH = 0.006
SLICE_HEIGHT = 0.0125
STACK_STEP = 0.012
CONNECTOR_HEIGHT = 0.016
FLEX_RADIUS = 0.0265
CONNECTOR_RADIUS = 0.031


def build_mjcf(tendon_radius=0.0194, tendon_angles_deg=(90.0, 210.0, 330.0),
               bend_stiffness=0.05, squeeze_stiffness=300.0,
               bend_damping=0.002, squeeze_damping=2.0,
               slice_mass=0.004, connector_mass=0.015,
               bend_limit=0.2, max_squeeze=0.006, max_tension=50.0):
    """Return the MJCF text. Stiffness: [N m/rad] per hinge, [N/m] per slide."""
    angles = [math.radians(a) for a in tendon_angles_deg]

    def rings(prefix, z):
        """Three small tendon guides at height z: one per tendon."""
        return ''.join(
            f'<site name="{prefix}_t{k + 1}" pos="{tendon_radius * math.cos(a):.6f} '
            f'{tendon_radius * math.sin(a):.6f} {z:.6f}" size="0.0015"/>\n'
            for k, a in enumerate(angles))

    def connector(z_centre):
        return (f'<geom type="cylinder" size="{CONNECTOR_RADIUS} {CONNECTOR_HEIGHT / 2}" '
                f'pos="0 0 {z_centre:.6f}" mass="{connector_mass}" rgba="0.6 0.6 0.6 1"/>\n')

    site_names = []          # tendon route, from base to tip, in order

    def add_rings(prefix):
        site_names.append(prefix)

    def slice_body(unit, i, offset, inner):
        name = f'{unit}_seg{i}'
        add_rings(name)
        return (
            f'<body name="{name}" pos="0 0 {offset:.6f}">\n'
            f'<joint name="{name}_squeeze" type="slide" axis="0 0 1" limited="true" '
            f'range="{-max_squeeze} 0" stiffness="{squeeze_stiffness}" '
            f'damping="{squeeze_damping}" armature="0.0001"/>\n'
            f'<joint name="{name}_bend_x" type="hinge" axis="1 0 0" limited="true" '
            f'range="{-bend_limit} {bend_limit}" stiffness="{bend_stiffness}" '
            f'damping="{bend_damping}" armature="0.0001"/>\n'
            f'<joint name="{name}_bend_y" type="hinge" axis="0 1 0" limited="true" '
            f'range="{-bend_limit} {bend_limit}" stiffness="{bend_stiffness}" '
            f'damping="{bend_damping}" armature="0.0001"/>\n'
            f'<geom type="cylinder" size="{FLEX_RADIUS} {SLICE_HEIGHT / 2}" '
            f'pos="0 0 {SLICE_HEIGHT / 2}" mass="{slice_mass}" rgba="0.1 0.1 0.1 0.6"/>\n'
            + rings(name, SLICE_HEIGHT / 2) + inner + '</body>\n')

    def unit_chain(unit, offset, inner):
        """bottom connector -> 8 slices -> top connector -> inner (next unit or tip)."""
        add_rings(f'{unit}_bottom')
        bottom_open = (f'<body name="{unit}_bottom" pos="0 0 {offset:.6f}">\n'
                       + connector(CONNECTOR_HEIGHT / 2)
                       + rings(f'{unit}_bottom', CONNECTOR_HEIGHT))
        # slices are nested inside each other, so build from the last one outwards
        add_rings(f'{unit}_top')
        top = (f'<body name="{unit}_top" pos="0 0 {SLICE_HEIGHT}">\n'
               + connector(CONNECTOR_HEIGHT / 2 - POST_DEPTH)
               + rings(f'{unit}_top', 0.0) + inner + '</body>\n')
        body = top
        for i in reversed(range(1, SLICES_PER_UNIT + 1)):
            off = POST_DEPTH + POST_LENGTH if i == 1 else SLICE_HEIGHT
            body = slice_body(unit, i, off, body)
        return bottom_open + body + '</body>\n'

    # --- the tip: the red ring where all three tendons are tied ---
    tip = ('<body name="tip" pos="0 0 {:.6f}">\n'.format(CONNECTOR_HEIGHT - POST_DEPTH)
           + '<geom type="cylinder" size="0.0205 0.001" pos="0 0 0.001" mass="0.002" '
             'rgba="0.9 0.1 0.1 1"/>\n'
           + rings('tip', 0.0) + '</body>\n')

    # Build the chain. site_names must end up in base-to-tip order.
    site_names.clear()
    unit2 = unit_chain('unit2', STACK_STEP - POST_DEPTH, tip)
    unit2_sites = list(site_names)
    site_names.clear()
    unit1 = unit_chain('unit1', 0.0, unit2)
    unit1_sites = list(site_names)
    route = (['trunk_base'] + sorted(unit1_sites, key=_route_key)
             + sorted(unit2_sites, key=_route_key) + ['tip'])

    tendons = ''.join(
        f'<spatial name="tendon{k}" width="0.0008" rgba="0.9 0.1 0.1 1">\n'
        + ''.join(f'<site site="{s}_t{k}"/>\n' for s in route)
        + '</spatial>\n'
        for k in (1, 2, 3))

    # A position actuator per tendon: it can only pull (force <= 0).
    actuators = ''.join(
        f'<position name="motor{k}" tendon="tendon{k}" kp="300" ctrllimited="false" '
        f'forcelimited="true" forcerange="{-max_tension} 0"/>\n'
        for k in (1, 2, 3))

    return f"""<mujoco model="trunk">
<compiler angle="radian"/>
<option timestep="0.001" gravity="0 0 -9.81">
<flag contact="disable"/>
</option>
<visual><global offwidth="1280" offheight="720"/></visual>
<worldbody>
<light pos="0 0 1.5" dir="0 0 -1"/>
<geom name="floor" type="plane" size="1 1 0.1" rgba="0.85 0.85 0.85 1"/>
<body name="base_link" pos="0 0 0.5">
<geom type="cylinder" size="0.06 0.02" pos="0 0 0.02" rgba="0.6 0.6 0.6 1" mass="0.5"/>
<body name="trunk_base" quat="0 1 0 0">
{rings('trunk_base', 0.0)}{unit1}</body>
</body>
</worldbody>
<tendon>
{tendons}</tendon>
<actuator>
{actuators}</actuator>
</mujoco>
"""


def _route_key(name):
    """Order: bottom connector, slices 1..8, top connector."""
    if name.endswith('_bottom'):
        return 0
    if name.endswith('_top'):
        return 99
    return int(name.rsplit('seg', 1)[1])