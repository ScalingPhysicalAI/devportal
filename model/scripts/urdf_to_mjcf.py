#!/usr/bin/env python3
"""Convert the Humanoid_description_latest_version ROS2/xacro package into a
native MuJoCo MJCF scene: the robot, plus a bare lab-room environment and a
placeholder actuator set so it can actually be driven around in the sim.

The source URDF is a xacro file exported by a SolidWorks-to-URDF plugin. It
only uses xacro for three top-level <xacro:include> directives (materials,
ros2control, gazebo) and $(find Humanoid_description) path substitution --
no macros, properties, or conditionals -- so it's resolved here with plain
string/XML processing instead of pulling in the `xacro` ROS package.

IMPORTANT -- what this script does NOT know:
    Every actuator gain, force range, and velocity limit added below is a
    placeholder, not measured hardware data (see the ACTUATOR SPECS section).
    The source CAD only carries geometry + kinematic limits; it has no motor
    or sensor specs, and none exist anywhere else in this repo. This script
    exists to produce a *bare, movable* dev/demo model -- do not use it (or
    any dataset collected from it) for sim-to-real transfer until real motor
    and tactile-sensor specs replace the placeholders marked below.

Usage:
    python3 model/scripts/urdf_to_mjcf.py
"""

import pathlib
import re
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

MIN_EIGENVALUE = 1e-9
MIN_MASS = 1e-6  # kg

ROOT = pathlib.Path(__file__).resolve().parent.parent
# NOTE: repointed from Humanoid_description -> Humanoid_description_latest_version
# (the CAD revision that adds real joint limit mates -- see conversation/PR notes).
# The old Humanoid_description package is left on disk untouched; it is no
# longer what this pipeline builds from.
DESCRIPTION_DIR = ROOT / "Humanoid_description_latest_version"
MESHES_DIR = DESCRIPTION_DIR / "meshes"
XACRO_PATH = DESCRIPTION_DIR / "urdf" / "Humanoid.xacro"
MATERIALS_PATH = DESCRIPTION_DIR / "urdf" / "materials.xacro"

RESOLVED_URDF_PATH = DESCRIPTION_DIR / "urdf" / "Humanoid.resolved.urdf"
OUTPUT_DIR = ROOT / "mjcf"
OUTPUT_MESHES_DIR = OUTPUT_DIR / "meshes"
OUTPUT_MJCF_PATH = OUTPUT_DIR / "humanoid.xml"

# Height (m) at which the robot's own local +Y axis (its CAD "up") sits above
# the floor once rolled onto world +Z -- i.e. how far the lowest mesh vertex
# (the caster wheel) sits below base_link's origin in this orientation, plus
# a hair of clearance so it isn't embedded in the floor. Unchanged from the
# previous CAD revision -- confirmed the base/wheel/caster geometry these
# numbers depend on did not move (only a chest part was added upstream).
BASE_HEIGHT = 0.115

# ---------------------------------------------------------------------------
# Joint role classification, used only to pick a sensible *placeholder*
# actuator (type + gains) per joint -- see the big warning at the top of this
# file. Every joint named here is cross-checked against Humanoid.xacro by
# `main()` (fails loudly if the set drifts on a future re-export).
# ---------------------------------------------------------------------------

# Real fingertip/knuckle joints (30 total, both hands): small parts, gentle
# placeholder gains. Everything else that got a real <limit> from CAD (arms,
# waist tilt, neck, both wrist flexions, the torso lift slider) is treated as
# a "body" joint with stronger placeholder gains.
FINGER_JOINTS = {
    "Revolute 48", "Revolute 49", "Revolute 50", "Revolute 51", "Revolute 52",
    "Revolute 53", "Revolute 54", "Revolute 55", "Revolute 56", "Revolute 57",
    "Revolute 58", "Revolute 59", "Revolute 60", "Revolute 61", "Revolute 62",
    "Revolute 69", "Revolute 70", "Revolute 71", "Revolute 72", "Revolute 73",
    "Revolute 74", "Revolute 75", "Revolute 76", "Revolute 77", "Revolute 78",
    "Revolute 79", "Revolute 80", "Revolute 81", "Revolute 82", "Revolute 83",
}
BODY_JOINTS = {
    "Slider 2", "Revolute 4", "Revolute 5", "Revolute 6", "Revolute 7",
    "Revolute 8", "Revolute 9", "Revolute 10", "Revolute 11", "Revolute 12",
    "Revolute 24", "Revolute 25", "Revolute 26", "Revolute 27",
    "Revolute 43", "Revolute 64",
}
# Continuous (unlimited) joints that still get a *velocity* actuator (spin-rate
# control avoids angle-wrap issues a position actuator would have here).
SPIN_JOINTS = {"Revolute 3"}  # torso/waist base rotation
# The two drive wheels get their own velocity actuator purely for visual
# spin (see DRIVE_WHEEL_RADIUS below) -- the base's actual motion comes from
# the virtual planar joint, not from wheel/ground rolling friction, so
# without this the wheels would just slide across the floor without turning
# while the robot drives. Casters are left fully passive (real casters don't
# drive); a light damping keeps them from spinning indefinitely on numerical
# noise.
DRIVE_WHEEL_JOINTS = {"Revolute 32", "Revolute 33"}
PASSIVE_SPIN_JOINTS = {"Revolute 36", "Revolute 37", "Revolute 38", "Revolute 39"}
# Measured directly from the wheel mesh's own geometry (bounding-box radius
# of the wheels_1/wheels_2 mesh) -- a real physical dimension, unlike the
# placeholder gains below.
DRIVE_WHEEL_RADIUS = 0.1015  # m

# --- placeholder gains (see file-level warning) -----------------------------
BODY_KP = 60.0
BODY_DAMPING = 2.0
FINGER_KP = 3.0
FINGER_DAMPING = 0.1
SPIN_KV = 15.0
SPIN_DAMPING = 1.0
PASSIVE_DAMPING = 0.3
PLACEHOLDER_FORCERANGE = 100.0  # matches the CAD export's own placeholder effort="100"

# Virtual planar base: three zero-size, near-massless bodies inserted between
# world and base_link, driven by their own velocity actuators. This stands in
# for the base's real wheel/ground drive dynamics, which this CAD cannot
# provide -- see resolve_urdf() for why.
BASE_SLIDE_KV = 60.0
BASE_YAW_KV = 25.0
BASE_SLIDE_FORCE = 150.0
BASE_YAW_FORCE = 60.0
BASE_SLIDE_CTRLRANGE = 1.0  # m/s -- a brisk walking pace, tuned for teleop over the room below
BASE_YAW_CTRLRANGE = 1.5  # rad/s
DRIVE_WHEEL_KV = 4.0
DRIVE_WHEEL_FORCE = 15.0
DRIVE_WHEEL_CTRLRANGE = BASE_SLIDE_CTRLRANGE / DRIVE_WHEEL_RADIUS * 1.2  # rad/s, with headroom

# Bare lab room: a floor (already added below) plus four walls forming a
# rectangular room, dressed with simple placeholder furniture (workbenches,
# a shelving unit, storage crates) so it reads as a lab rather than an empty
# box -- and sized to leave a large open area in the middle to drive/walk
# around in. All primitive geoms (no external mesh assets) -- see
# _build_room().
ROOM_HALF_EXTENT = 6.0  # metres from center to each wall -> 12m x 12m room
ROOM_WALL_HEIGHT = 2.6
ROOM_WALL_THICKNESS = 0.08


def resolve_urdf() -> str:
    """Inline the materials include, drop the ros2control/gazebo includes
    (irrelevant to a physics-only MJCF model), rewrite mesh paths from the
    ROS `$(find pkg)` package syntax to plain paths relative to the resolved
    URDF's own directory, and replace the URDF root link's implicit free
    joint with a constrained planar (x, y, yaw) virtual base."""
    xacro_text = XACRO_PATH.read_text()

    materials_root = ET.fromstring(MATERIALS_PATH.read_text())
    materials_xml = "\n".join(
        ET.tostring(el, encoding="unicode") for el in materials_root.findall("material")
    )

    lines = xacro_text.splitlines()
    out_lines = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('<xacro:include filename="$(find Humanoid_description)/urdf/materials.xacro"'):
            out_lines.append(materials_xml)
        elif stripped.startswith('<xacro:include filename="$(find Humanoid_description)/urdf/Humanoid.ros2control"'):
            continue  # ros2control hardware interface -- not used by MuJoCo
        elif stripped.startswith('<xacro:include filename="$(find Humanoid_description)/urdf/Humanoid.gazebo"'):
            continue  # Gazebo-specific plugins/tags -- not used by MuJoCo
        else:
            out_lines.append(line)

    resolved = "\n".join(out_lines)
    resolved = resolved.replace(' xmlns:xacro="http://www.ros.org/wiki/xacro"', "")
    resolved = resolved.replace("file://$(find Humanoid_description)/meshes/", "../meshes/")

    # A handful of the tiny finger links carry degenerate inertia matrices
    # (a zero principal moment) from the SolidWorks URDF export's floating
    # point rounding on very small parts. `balanceinertia` asks MuJoCo's
    # compiler to project any non-positive-definite inertia to the nearest
    # valid one instead of hard-failing the whole model load.
    resolved = resolved.replace(
        '<robot name="Humanoid">',
        '<robot name="Humanoid">\n<mujoco><compiler balanceinertia="true" discardvisual="false"/></mujoco>',
    )

    # The exported model's own "up" axis is local +Y (wheels/casters sit at
    # low Y, the head/face_cover sits at high Y), not Z. MuJoCo's gravity and
    # floor-plane convention here is Z-up, so this is corrected by rolling
    # +90deg about X (maps local +Y onto world +Z) on the very first joint
    # below; every joint after it inherits that corrected frame.
    #
    # This robot has a narrow two-wheel-plus-caster footprint under a much
    # heavier upper body/arms, with no balance controller -- and the wheel/
    # caster/actuator parts in this CAD are explicitly placeholder geometry
    # (named "*_dummy_*"), not the final chassis. Given a free 6-DOF joint it
    # topples the instant gravity is applied, no matter how gently it's
    # placed. Modeling real balance is a hardware/controls problem this CAD
    # revision can't inform (final wheelbase geometry isn't set yet) -- so as
    # an explicit, documented stand-in, the base gets exactly 3 DOF: slide
    # along world X, slide along world Y, and yaw about world Z, each with
    # its own velocity actuator (see main()). It can drive around the floor
    # like a mobile base, but cannot tip, pitch, or roll. Replace this with
    # real wheel/ground contact dynamics once final chassis geometry and
    # drive-motor specs exist.
    #
    # Axis math for the two follow-on joints (derived, then verified by
    # loading the compiled model and checking data.xpos against known qpos):
    # after the +90deg-about-X frame fix, local +X still maps to world +X,
    # local +Y maps to world +Z, and local +Z maps to world -Y. So the second
    # joint's axis (0 0 -1) yields world +Y, and the third joint's axis
    # (0 1 0) -- the model's own original "up" axis -- yields yaw about
    # world +Z.
    resolved = resolved.replace(
        "</robot>",
        '<link name="world"/>\n'
        '<link name="virtual_base_x"><inertial><mass value="0.001"/>'
        '<inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" iyz="0" ixz="0"/></inertial></link>\n'
        '<link name="virtual_base_y"><inertial><mass value="0.001"/>'
        '<inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" iyz="0" ixz="0"/></inertial></link>\n'
        '<joint name="virtual_base_x" type="prismatic">\n'
        f'  <origin xyz="0 0 {BASE_HEIGHT}" rpy="1.5707963267948966 0 0"/>\n'
        '  <parent link="world"/>\n'
        '  <child link="virtual_base_x"/>\n'
        '  <axis xyz="1 0 0"/>\n'
        '  <limit lower="-50" upper="50" effort="80" velocity="1"/>\n'
        "</joint>\n"
        '<joint name="virtual_base_y" type="prismatic">\n'
        '  <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        '  <parent link="virtual_base_x"/>\n'
        '  <child link="virtual_base_y"/>\n'
        '  <axis xyz="0 0 -1"/>\n'
        '  <limit lower="-50" upper="50" effort="80" velocity="1"/>\n'
        "</joint>\n"
        '<joint name="virtual_base_yaw" type="continuous">\n'
        '  <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        '  <parent link="virtual_base_y"/>\n'
        '  <child link="base_link"/>\n'
        '  <axis xyz="0 1 0"/>\n'
        '  <limit effort="40" velocity="2"/>\n'
        "</joint>\n"
        "</robot>",
    )
    return resolved


def sanitize_inertias(urdf_text: str) -> str:
    """A handful of the tiny finger links carry degenerate inertia matrices
    (a zero principal moment) from the SolidWorks URDF export's floating
    point rounding on very small parts -- MuJoCo requires every inertia
    matrix to be strictly positive-definite. Clamp any non-positive
    eigenvalue up to a small positive floor, per link, rather than hand
    tuning each offending part."""
    root = ET.fromstring(urdf_text)
    fixed = []
    for link in root.findall("link"):
        mass_el = link.find("inertial/mass")
        if mass_el is not None and float(mass_el.get("value")) < MIN_MASS:
            mass_el.set("value", repr(MIN_MASS))
            fixed.append(f"{link.get('name')} (mass)")

        inertia_el = link.find("inertial/inertia")
        if inertia_el is None:
            continue
        m = np.array(
            [
                [float(inertia_el.get("ixx")), float(inertia_el.get("ixy")), float(inertia_el.get("ixz"))],
                [float(inertia_el.get("ixy")), float(inertia_el.get("iyy")), float(inertia_el.get("iyz"))],
                [float(inertia_el.get("ixz")), float(inertia_el.get("iyz")), float(inertia_el.get("izz"))],
            ]
        )
        eigvals, eigvecs = np.linalg.eigh(m)
        if eigvals.min() > MIN_EIGENVALUE:
            continue
        clamped = np.clip(eigvals, MIN_EIGENVALUE, None)
        m_fixed = eigvecs @ np.diag(clamped) @ eigvecs.T
        inertia_el.set("ixx", repr(float(m_fixed[0, 0])))
        inertia_el.set("iyy", repr(float(m_fixed[1, 1])))
        inertia_el.set("izz", repr(float(m_fixed[2, 2])))
        inertia_el.set("ixy", repr(float(m_fixed[0, 1])))
        inertia_el.set("ixz", repr(float(m_fixed[0, 2])))
        inertia_el.set("iyz", repr(float(m_fixed[1, 2])))
        fixed.append(link.get("name"))

    if fixed:
        print(f"sanitized degenerate inertia on {len(fixed)} link(s): {', '.join(fixed)}")
    return ET.tostring(root, encoding="unicode")


def _inject_joint_damping(mjcf_text: str, damping_by_name: dict[str, float]) -> str:
    """mj_saveLastXML emits one self-closing <joint name="..." .../> per
    joint; add a damping="..." attribute to each by exact name match."""

    def repl(match: "re.Match[str]") -> str:
        name = match.group(1)
        damping = damping_by_name.get(name)
        if damping is None:
            return match.group(0)
        return match.group(0)[:-2] + f' damping="{damping}"/>'

    return re.sub(r'<joint name="([^"]+)"[^/]*/>', repl, mjcf_text)


def _build_actuators(model: "mujoco.MjModel") -> str:
    """One placeholder actuator per controllable joint -- see the file-level
    warning: none of these gains are real motor specs."""
    lines = ["<actuator>"]
    seen = set()
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name is None:
            continue
        seen.add(name)
        jtype = model.jnt_type[j]

        if name in FINGER_JOINTS or name in BODY_JOINTS:
            lo, hi = model.jnt_range[j]
            lines.append(
                f'  <position name="act_{name}" joint="{name}" '
                f'kp="{FINGER_KP if name in FINGER_JOINTS else BODY_KP}" '
                f'ctrlrange="{lo:.6f} {hi:.6f}" '
                f'forcerange="-{PLACEHOLDER_FORCERANGE} {PLACEHOLDER_FORCERANGE}"/>'
            )
        elif name in SPIN_JOINTS:
            lines.append(
                f'  <velocity name="act_{name}" joint="{name}" kv="{SPIN_KV}" '
                f'ctrlrange="-2 2" forcerange="-{PLACEHOLDER_FORCERANGE} {PLACEHOLDER_FORCERANGE}"/>'
            )
        elif name in DRIVE_WHEEL_JOINTS:
            # Cosmetic spin only, sized off the wheel's own measured radius --
            # see DRIVE_WHEEL_RADIUS -- driven directly from teleop input in
            # the frontend, not from wheel/ground contact torque.
            lines.append(
                f'  <velocity name="act_{name}" joint="{name}" kv="{DRIVE_WHEEL_KV}" '
                f'ctrlrange="-{DRIVE_WHEEL_CTRLRANGE:.3f} {DRIVE_WHEEL_CTRLRANGE:.3f}" '
                f'forcerange="-{DRIVE_WHEEL_FORCE} {DRIVE_WHEEL_FORCE}"/>'
            )
        elif name in PASSIVE_SPIN_JOINTS:
            continue  # cosmetic caster spin, intentionally unactuated
        elif name.startswith("virtual_base_"):
            continue  # handled explicitly below, in a fixed order
        elif jtype == mujoco.mjtJoint.mjJNT_FREE:
            continue
        else:
            raise ValueError(f"unclassified joint '{name}' -- add it to a role set in urdf_to_mjcf.py")

    lines.append(
        f'  <velocity name="act_base_vx" joint="virtual_base_x" kv="{BASE_SLIDE_KV}" '
        f'ctrlrange="-{BASE_SLIDE_CTRLRANGE} {BASE_SLIDE_CTRLRANGE}" forcerange="-{BASE_SLIDE_FORCE} {BASE_SLIDE_FORCE}"/>'
    )
    lines.append(
        f'  <velocity name="act_base_vy" joint="virtual_base_y" kv="{BASE_SLIDE_KV}" '
        f'ctrlrange="-{BASE_SLIDE_CTRLRANGE} {BASE_SLIDE_CTRLRANGE}" forcerange="-{BASE_SLIDE_FORCE} {BASE_SLIDE_FORCE}"/>'
    )
    lines.append(
        f'  <velocity name="act_base_yaw" joint="virtual_base_yaw" kv="{BASE_YAW_KV}" '
        f'ctrlrange="-{BASE_YAW_CTRLRANGE} {BASE_YAW_CTRLRANGE}" forcerange="-{BASE_YAW_FORCE} {BASE_YAW_FORCE}"/>'
    )
    lines.append("</actuator>")

    expected = FINGER_JOINTS | BODY_JOINTS | SPIN_JOINTS | DRIVE_WHEEL_JOINTS | PASSIVE_SPIN_JOINTS
    missing = expected - seen
    if missing:
        raise ValueError(f"joints classified in urdf_to_mjcf.py but absent from the model: {sorted(missing)}")

    return "\n".join(lines)


_WALL_RGBA = "0.55 0.57 0.6 1"
_TABLETOP_RGBA = "0.78 0.76 0.68 1"
_METAL_RGBA = "0.32 0.33 0.35 1"
_CRATE_RGBAS = ["0.55 0.22 0.18 1", "0.18 0.32 0.5 1", "0.2 0.45 0.28 1"]


def _static_box(name: str, pos: tuple, size: tuple, rgba: str) -> str:
    return (
        f'  <geom name="{name}" type="box" pos="{pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f}" '
        f'size="{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}" rgba="{rgba}" contype="1" conaffinity="1"/>'
    )


def _table(name: str, cx: float, cy: float, length: float = 1.4, depth: float = 0.7, height: float = 0.75) -> list:
    """A workbench: one tabletop box on four leg boxes, axis-aligned."""
    top_t = 0.04
    leg_t = 0.04
    geoms = [
        _static_box(
            f"{name}_top", (cx, cy, height - top_t / 2), (length / 2, depth / 2, top_t / 2), _TABLETOP_RGBA
        )
    ]
    for i, (lx, ly) in enumerate(
        [
            (length / 2 - leg_t, depth / 2 - leg_t),
            (length / 2 - leg_t, -(depth / 2 - leg_t)),
            (-(length / 2 - leg_t), depth / 2 - leg_t),
            (-(length / 2 - leg_t), -(depth / 2 - leg_t)),
        ]
    ):
        geoms.append(
            _static_box(
                f"{name}_leg{i}", (cx + lx, cy + ly, (height - top_t) / 2), (leg_t / 2, leg_t / 2, (height - top_t) / 2), _METAL_RGBA
            )
        )
    return geoms


def _shelf_unit(name: str, cx: float, cy: float, width: float = 1.6, depth: float = 0.4, height: float = 1.8) -> list:
    """A 3-shelf storage unit: two side panels + three horizontal shelves."""
    panel_t = 0.03
    geoms = [
        _static_box(f"{name}_sidea", (cx - width / 2, cy, height / 2), (panel_t / 2, depth / 2, height / 2), _METAL_RGBA),
        _static_box(f"{name}_sideb", (cx + width / 2, cy, height / 2), (panel_t / 2, depth / 2, height / 2), _METAL_RGBA),
    ]
    for i, frac in enumerate([0.05, 0.5, 0.95]):
        geoms.append(
            _static_box(f"{name}_shelf{i}", (cx, cy, height * frac), (width / 2, depth / 2, panel_t / 2), _METAL_RGBA)
        )
    return geoms


def _crate(name: str, cx: float, cy: float, half: float = 0.22, rgba: str = _CRATE_RGBAS[0]) -> list:
    return [_static_box(name, (cx, cy, half), (half, half, half), rgba)]


def _build_room() -> str:
    """A rectangular lab room (floor + 4 walls) dressed with simple
    placeholder furniture -- workbenches along two walls, a shelving unit,
    and a few storage crates -- with a large open area left clear in the
    middle for the mobile base to actually drive/walk around in. All
    primitive box geoms (no external mesh assets); purely a placeholder
    layout, not a real lab floorplan -- swap in real dimensions/furniture
    once there's a target environment to match."""
    e = ROOM_HALF_EXTENT
    h = ROOM_WALL_HEIGHT
    t = ROOM_WALL_THICKNESS
    lines = [
        _static_box("wall_north", (0, e, h / 2), (e + t, t, h / 2), _WALL_RGBA),
        _static_box("wall_south", (0, -e, h / 2), (e + t, t, h / 2), _WALL_RGBA),
        _static_box("wall_east", (e, 0, h / 2), (t, e + t, h / 2), _WALL_RGBA),
        _static_box("wall_west", (-e, 0, h / 2), (t, e + t, h / 2), _WALL_RGBA),
    ]

    inset = e - 0.9  # workbenches/shelving sit just inside the walls
    # Two workbenches along the north wall, spaced apart.
    lines += _table("bench_n1", -e / 2, inset)
    lines += _table("bench_n2", e / 2, inset)
    # One workbench along the east wall (rotated footprint: deep along x).
    lines += _table("bench_e1", inset, -e / 2, length=0.7, depth=1.4)
    # Shelving unit along the west wall.
    lines += _shelf_unit("shelf_w1", -inset, e / 2 - 0.3)
    # A small cluster of storage crates near the south wall, out of the main
    # walking lane down the middle of the room.
    crate_spots = [(-e + 1.2, -e + 1.0), (-e + 1.7, -e + 1.0), (-e + 1.2, -e + 1.5)]
    for i, (cx, cy) in enumerate(crate_spots):
        lines += _crate(f"crate_{i}", cx, cy, rgba=_CRATE_RGBAS[i % len(_CRATE_RGBAS)])

    return "\n".join(lines)


# The four ground-contact wheel discs (2 drive wheels + 2 caster wheels).
# Each one's <visual>/<collision> mesh sits well off its own joint's default
# rotation point (jnt_pos, which URDF always leaves at the child body's own
# origin) -- measured: 6.3-28.7cm -- even though the STL mesh itself is a
# properly centered disc. That offset is *not* an export error to discard:
# it's what places the wheel at its correct ground-contact height (this
# body's local Y axis maps to world Z/up, post the up-axis fix in
# resolve_urdf(), so this reads as a vertical offset). Moving the mesh to
# sit on the default anchor (an earlier version of this function) put the
# wheel visibly up inside the chassis instead. Spinning it for teleop
# without any correction makes it visibly wobble/precess around that
# offset, and the same offset applies to the *collision* geom, so the
# wheel's ground-contact point sweeps up and down as it turns.
#
# The geometrically correct fix moves the *joint's* anchor to the mesh's
# own center instead of moving the mesh: set jnt_pos to the geom's offset
# (unchanged) so the hinge rotates exactly about the point the disc already
# occupies. That keeps the at-rest visual position identical to the raw CAD
# export while eliminating the wobble (rotating a disc about its own
# center, by construction, cannot make it precess).
WHEEL_DISC_BODIES = ["wheels_1", "wheels_2", "caster_wheel_1", "caster_wheel_2"]


def recenter_wheel_geoms(model: "mujoco.MjModel") -> None:
    """Move each wheel disc's joint anchor (jnt_pos) to coincide with its
    own geom's center, in place on the already-compiled model (before
    mj_saveLastXML). Geom positions are left untouched -- see the module
    comment above for why moving the mesh instead was wrong."""
    for body_name in WHEEL_DISC_BODIES:
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if b < 0:
            raise ValueError(f"expected wheel body '{body_name}' not found in model")
        if model.body_jntnum[b] != 1:
            raise ValueError(f"expected exactly one joint on '{body_name}', found {model.body_jntnum[b]}")
        j = model.body_jntadr[b]
        geom_positions = [model.geom_pos[g].copy() for g in range(model.ngeom) if model.geom_bodyid[g] == b]
        if not geom_positions or any(np.linalg.norm(p - geom_positions[0]) > 1e-9 for p in geom_positions):
            raise ValueError(f"expected '{body_name}' geoms to share one center, got {geom_positions}")
        center = geom_positions[0]
        offset = np.linalg.norm(center - model.jnt_pos[j])
        model.jnt_pos[j] = center
        print(f"{body_name}: moved joint anchor {offset * 100:.1f}cm to the disc's own center (disc position unchanged)")


def disable_wheel_floor_collision(mjcf_text: str) -> str:
    """Zero out contype/conaffinity on every wheel disc geom (drive wheels
    and casters alike). Text-level, applied to the already-saved MJCF --
    mj_saveLastXML does not reflect in-memory geom_contype/geom_conaffinity
    mutations made on the model before saving (confirmed: setting them
    directly on the compiled model, then immediately re-saving, produced no
    contype/conaffinity attribute at all in the output -- unlike geom_pos/
    jnt_pos, which *do* round-trip that way; a real quirk/limitation of this
    save path for these two fields specifically), so this has to happen as
    a string edit on the file instead, the same way the rest of this
    script's post-mj_saveLastXML fixes do.

    The virtual planar base (see resolve_urdf()) fixes the robot's height
    directly -- it has never needed the wheels to physically rest on the
    floor to stay up. But the drive wheels *do* still touch the floor with
    real contact/friction, and DRIVE_WHEEL_*'s cosmetic spin rate (set from
    the *commanded* teleop speed, not the chassis's actual speed) is
    essentially never the true rolling speed. Whenever they disagree, the
    resulting slip generates a real kinetic-friction force at the contact
    patch, transmitted straight back into base_link through the wheel's own
    joint -- confirmed live in-browser: forward ctrl held at 0.8 for 20+s
    produced a base speed of ~0.004 m/s, ~1/200th of commanded, because that
    friction was fighting the base's own drive actuator almost to a
    standstill. Since the wheels don't need floor contact for anything, the
    fix is to remove it outright rather than trying to match spin rate to
    actual speed (which would defeat the point of a cosmetic, teleop-speed
    -driven spin in the first place)."""
    wheel_meshes = tuple(WHEEL_DISC_BODIES)  # geom mesh names match their body's name 1:1

    def _strip_wheel_collision(match: "re.Match[str]") -> str:
        tag = match.group(0)
        if not re.search(rf'mesh="({"|".join(wheel_meshes)})"', tag):
            return tag
        tag = re.sub(r'\s*contype="[^"]*"', "", tag)
        tag = re.sub(r'\s*conaffinity="[^"]*"', "", tag)
        return tag[:-2] + ' contype="0" conaffinity="0"/>'

    fixed = re.sub(r"<geom [^>]*/>", _strip_wheel_collision, mjcf_text)
    print(f"disabled floor/world collision on wheel discs: {', '.join(WHEEL_DISC_BODIES)}")
    return fixed


def main() -> None:
    resolved_urdf = sanitize_inertias(resolve_urdf())
    RESOLVED_URDF_PATH.write_text(resolved_urdf)
    print(f"wrote resolved URDF -> {RESOLVED_URDF_PATH.relative_to(ROOT.parent)}")

    model = mujoco.MjModel.from_xml_path(str(RESOLVED_URDF_PATH))
    print(f"loaded MJCF model: {model.nbody} bodies, {model.njnt} joints, {model.nq} qpos, {model.ngeom} geoms")

    recenter_wheel_geoms(model)
    actuator_xml = _build_actuators(model)

    OUTPUT_DIR.mkdir(exist_ok=True)
    OUTPUT_MESHES_DIR.mkdir(exist_ok=True)
    mujoco.mj_saveLastXML(str(OUTPUT_MJCF_PATH), model)

    # mj_saveLastXML carries over mesh <file> paths as given to the compiler,
    # which were relative to the resolved URDF's directory (.../urdf). Copy
    # the referenced meshes alongside the saved MJCF and rewrite the paths to
    # match, so model/mjcf/ is a self-contained, portable unit (no reference
    # back into Humanoid_description_latest_version/).
    mjcf_text = OUTPUT_MJCF_PATH.read_text()
    for stl in MESHES_DIR.glob("*.stl"):
        if f'"../meshes/{stl.name}"' in mjcf_text:
            shutil.copy2(stl, OUTPUT_MESHES_DIR / stl.name)
    mjcf_text = mjcf_text.replace('file="../meshes/', 'file="meshes/')

    # NOTE: an earlier version of this script hid the "*_dummy_*"
    # servo/actuator-housing meshes (the shoulder/arm/hand motor-housing
    # brackets) because each one is rigidly fixed to only *one* side of its
    # joint and visibly separates from the other side once that joint
    # rotates by a real amount. Reverted at the user's request -- the
    # brackets are wanted back as visible landmarks on the hands/shoulders
    # for now; the joint-motion visual gap is a known, separate issue to
    # revisit alongside the wave gesture rework.
    mjcf_text = disable_wheel_floor_collision(mjcf_text)

    # Floor + the lab room (walls + furniture) the mobile base can drive
    # around inside. Walls/furniture share collision bit 1 with the robot's
    # own default conaffinity (see the <default><geom .../></default> added
    # below), so they still stop the robot from being driven through. The
    # floor plane gets its *own* bit (4) instead of sharing bit 1 -- no
    # robot geom's conaffinity includes it, so nothing on the robot collides
    # with the bare floor at all. That's intentional, not an oversight: the
    # virtual planar base (see resolve_urdf()) already fixes the robot's
    # height directly, so floor contact was never load-bearing here, and
    # base_link's own collision box happens to sit flush with the floor at
    # rest -- confirmed live in-browser as the reason a held forward drive
    # command (well within the actuator's own force budget, ~39N of ~150N
    # available) produced a base speed of ~0.004 m/s instead of ~0.8: most
    # of that force was going into kinetic friction against the floor
    # instead of moving the robot. (Recentering the wheels onto their own
    # joints and stripping their own floor collision, above and in
    # recenter_wheel_geoms()/disable_wheel_floor_collision(), fixed the
    # wheels' own contribution to this same problem but not base_link's.)
    floor_extent = ROOM_HALF_EXTENT + 0.2
    mjcf_text = mjcf_text.replace(
        "<worldbody>",
        '<worldbody>\n'
        f'  <geom name="floor" type="plane" size="{floor_extent} {floor_extent} 0.1" rgba="0.25 0.25 0.28 1" contype="4" conaffinity="4"/>\n'
        f"{_build_room()}\n",
        1,
    )

    # Per-joint damping (numerical stability; the position/velocity actuators
    # above -- not this damping -- are what actually holds each joint's
    # commanded pose against gravity).
    damping_by_name = {name: BODY_DAMPING for name in BODY_JOINTS}
    damping_by_name.update({name: FINGER_DAMPING for name in FINGER_JOINTS})
    damping_by_name.update({name: SPIN_DAMPING for name in SPIN_JOINTS})
    damping_by_name.update({name: PASSIVE_DAMPING for name in DRIVE_WHEEL_JOINTS | PASSIVE_SPIN_JOINTS})
    # Virtual base joints are left at zero extra damping -- their velocity
    # actuators (force = kv * (ctrl - qvel)) already regulate qvel directly.
    mjcf_text = _inject_joint_damping(mjcf_text, damping_by_name)

    # Self-collision: several adjacent links (base_link and the
    # wheelbase/wheels/casters mated to it) are snugly fit by design in the
    # source CAD and their collision meshes overlap by several centimeters at
    # rest (confirmed via mj_forward + data.contact on the prior CAD
    # revision; this joint area is unchanged in this revision). None of this
    # model's *other* parts are in contact at rest, so the fix is to disable
    # collisions between the robot's own geoms entirely (contype/conaffinity
    # bit 2) while keeping floor/wall collisions (bit 1).
    # Gravity compensation, applied uniformly to every body: with no real
    # motor specs, a placeholder position-servo gain (BODY_KP) has no
    # principled way to be sized against gravity torque -- e.g. the waist
    # joint alone would need to react the weight of the entire upper body
    # above it, and drooped ~50deg off its commanded pose under BODY_KP
    # before this was added (confirmed by stepping the model to steady
    # state). Real servo/arm controllers almost always include gravity
    # compensation in their own low-level loop, so this is standing in for
    # that -- not a substitute for real gain tuning once real motor specs
    # exist.
    mjcf_text = mjcf_text.replace(
        "<compiler",
        '<option integrator="implicitfast"/>\n'
        "<default>\n"
        '  <geom contype="2" conaffinity="1"/>\n'
        "</default>\n"
        "<compiler",
        1,
    )
    # gravcomp is a per-body attribute, not something <default> can apply
    # (unlike geom/joint) -- inject it onto every <body ...> tag directly.
    mjcf_text = re.sub(r"<body ", '<body gravcomp="1" ', mjcf_text)

    mjcf_text = mjcf_text.replace("</mujoco>", actuator_xml + "\n</mujoco>")
    OUTPUT_MJCF_PATH.write_text(mjcf_text)

    # Verify the saved MJCF is loadable standalone before declaring success.
    mujoco.MjModel.from_xml_path(str(OUTPUT_MJCF_PATH))
    print(f"wrote MJCF -> {OUTPUT_MJCF_PATH.relative_to(ROOT.parent)}")


if __name__ == "__main__":
    main()
