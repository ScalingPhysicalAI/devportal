#!/usr/bin/env python3
"""Convert the Humanoid_description_latest_version ROS2/xacro package into a
native MuJoCo MJCF scene: the robot, plus a bare lab-room environment and a
placeholder actuator set so it can actually be driven around in the sim.

The source URDF is a xacro file exported by a SolidWorks-to-URDF plugin. It
only uses xacro for three top-level <xacro:include> directives (materials,
ros2control, gazebo) and $(find Humanoid_description) path substitution --
no macros, properties, or conditionals -- so it's resolved here with plain
string/XML processing instead of pulling in the `xacro` ROS package.

IMPORTANT -- what this script does and does not know:
    The source CAD/URDF's own <limit effort="100" velocity="100"/> on every
    joint is a uniform placeholder, not real hardware data -- confirmed by
    grepping the xacro itself, every single joint carries that exact same
    round number. Torque *ceilings* (TORQUE_NM below) now come from the
    user directly instead: 15 Nm for the general body servos, 45 Nm for the
    waist, 4 Nm for the wrist, 3 Nm for the fingers (not separately specced,
    treated as an estimate). The lift column's force and every joint's
    *gain* (kp/kv -- how hard it corrects a given error, as opposed to the
    torque ceiling on how hard it's allowed to push at all) are still this
    script's own placeholder tuning, not measured servo response data --
    there is still no tactile-sensor data anywhere. Don't use this (or any
    dataset collected from it) for sim-to-real transfer until that's true.

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
# gains. Everything else that got a real <limit> from CAD (arms, waist tilt,
# neck, both wrist flexions, the torso lift slider) is treated as a "body"
# joint -- split further below into WAIST_JOINTS/WRIST_JOINTS (their own,
# higher/lower torque ceilings) and the rest (shoulders, elbows, upper-arm
# roll, neck) at the general "servo" ceiling.
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
# Waist: Revolute 4 (torso pitch, its own parent link is literally named
# "rotation_waist_joint_1") plus Revolute 3 below (torso yaw, the joint that
# *creates* that same link) -- both get the 45 Nm waist rating.
WAIST_JOINTS = {"Revolute 4"}
# Wrist: the two joints between each forearm and its palm (axis A: Revolute
# 26/27 off the actuator_dummy housings; axis B: Revolute 43/64, the last
# joint before palm_right_1/palm_left_1 themselves) -- 4 Nm each, both arms.
WRIST_JOINTS = {"Revolute 26", "Revolute 27", "Revolute 43", "Revolute 64"}
# Continuous (unlimited) joints that still get a *velocity* actuator (spin-rate
# control avoids angle-wrap issues a position actuator would have here).
SPIN_JOINTS = {"Revolute 3"}  # torso/waist yaw -- also waist-rated, see above
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

# --- torque ceilings: real, user-provided (see the file-level note) --------
BODY_TORQUE_NM = 15.0  # general body servo: shoulders, elbows, upper-arm roll, neck
WAIST_TORQUE_NM = 45.0
WRIST_TORQUE_NM = 4.0
FINGER_TORQUE_NM = 3.0  # not separately specced -- user-provided estimate

# --- gains (kp/kv): still this script's own placeholder tuning -- how hard
# each joint *corrects* a given error, capped by *_TORQUE_NM above on how
# hard it's allowed to push at all. kv (added to every position actuator
# below, not just relying on passive joint damping) is this file's stand-in
# for a real servo's own closed-loop derivative gain -- without it, BODY_KP
# alone against this model's real inertia is badly underdamped: confirmed
# via headless step-response on the lift column specifically (the biggest
# single moving mass on the robot, ~13.7kg by body_subtreemass) -- kv=0
# visibly overshoots its commanded height and oscillates for a full second
# before settling (exactly the "spring"/"moon gravity" bounce reported live);
# kv roughly at or somewhat above the mass-spring critical-damping point
# (2*sqrt(kp*mass) -- ~57 for the lift column's own kp/mass) removes the
# overshoot entirely. The ratios below scale that same kv/kp relationship
# to the other joint categories rather than re-deriving a critical-damping
# mass for each one individually (their moving masses are all much smaller
# and less consequential to get exactly right).
BODY_KP = 60.0
BODY_KV = 45.0
BODY_DAMPING = 2.0
WAIST_KP = 90.0  # stiffer than a general body servo -- 3x the torque budget (45 vs 15 Nm)
WAIST_KV = 70.0
WRIST_KP = 15.0  # softer -- much smaller torque budget (4 Nm) than a general body servo
WRIST_KV = 12.0
FINGER_KP = 3.0
FINGER_KV = 2.0
FINGER_DAMPING = 0.1
LIFT_KP = 60.0
LIFT_KV = 55.0  # tuned directly against this joint's own ~13.7kg lifted mass, see above
SPIN_KV = 15.0
SPIN_DAMPING = 1.0
PASSIVE_DAMPING = 0.3

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

# Pick-and-place demo prop: a small free-floating box, spawned resting on
# bench_n1's tabletop, for the frontend's scripted "Pick & Place" sequence to
# grab (right hand) and carry over to bench_n2. Position is bench_n1's own
# (cx, cy) from _build_room() plus a fixed offset toward the table's front
# (room-facing) edge -- see MujocoViewer.tsx's PICK_PARK_POSE/PLACE_PARK_POSE
# comments for how the frontend derives where the base must park to reach it
# (the two are solved together: this file fixes the object's world position,
# the frontend's arm IK -- solved offline, see its own comment -- fixes the
# base's position/heading *relative to the object*, and moving the object
# here without re-deriving that offset will make the reach miss).
PICKUP_OBJECT_HALF_SIZE = 0.04  # 8cm cube
PICKUP_OBJECT_POS = (-3.0, 4.85, 0.75 + PICKUP_OBJECT_HALF_SIZE)  # on bench_n1
PICKUP_OBJECT_MASS = 0.15  # kg -- light enough for the placeholder finger actuators to hold
PICKUP_OBJECT_RGBA = "0.95 0.45 0.1 1"

# --- Pick & Place grasp geometry (right arm: palm_right_1's chain) ---------
# Solved offline (numeric IK against this same model, minimizing the 4
# main-finger fingertip centroid's distance to a target point -- the thumb
# chain is excluded, see MujocoViewer.tsx's own comment on why) and copied
# here as literals; MujocoViewer.tsx duplicates these same six joint names
# and both qpos arrays (its own PICK_ARM_JOINTS/PICK_GRASP_QPOS/
# PICK_PREGRASP_QPOS) to drive the scripted reach -- if either the object's
# position above or these numbers change, both files need updating together
# or the reach will miss and/or this weld's relpose (computed below, against
# the *unmoved* object) will no longer match where the closed hand actually
# is at the grasp pose.
PICK_ARM_JOINTS = ["Revolute 5", "Revolute 7", "Revolute 9", "Revolute 11", "Revolute 26", "Revolute 43"]
# Solved for a grip centroid 10cm *above* the tabletop (object center sits at
# 4cm, object top face at 8cm) rather than level with the object -- closing
# the fingers right at the object's own height left the lowest of the four
# fingertips only ~1mm above the table (confirmed by headless sim: the arm's
# placeholder position gain couldn't lift out of that, staying jammed against
# the tabletop). The weld below is what actually holds the object regardless
# of this offset -- see _build_grasp_weld()'s own comment.
PICK_GRASP_QPOS = [-0.5666, 0.9047, -0.1661, -0.0047, -0.1486, 0.1785]
# Base (world_x, world_y, yaw) the mobile base must be parked at for the
# above arm pose to actually reach PICKUP_OBJECT_POS -- yaw=pi faces the
# base toward +Y (north, where bench_n1/bench_n2 sit), and (x, y-0.75) is
# this same right arm's reach solved with the base at the world origin
# facing that same way (see the frontend's own PICK_PARK_POSE comment for
# the full derivation of that 0.75m offset from the virtual base's geometry).
PICK_PARK_POSE = (PICKUP_OBJECT_POS[0], PICKUP_OBJECT_POS[1] - 0.75, np.pi)


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
    """One actuator per controllable joint, torque-capped per *_TORQUE_NM
    (see that section's own comment for which numbers are real vs. this
    script's own placeholder tuning)."""
    lines = ["<actuator>"]
    seen = set()
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name is None:
            continue
        seen.add(name)
        jtype = model.jnt_type[j]

        if name in FINGER_JOINTS or name in BODY_JOINTS:
            # Priority matters: WAIST_JOINTS/WRIST_JOINTS/"Slider 2" are all
            # subsets of BODY_JOINTS with their own torque/gain, checked
            # before the generic BODY_JOINTS fallback.
            if name in FINGER_JOINTS:
                kp, kv, torque = FINGER_KP, FINGER_KV, FINGER_TORQUE_NM
            elif name in WAIST_JOINTS:
                kp, kv, torque = WAIST_KP, WAIST_KV, WAIST_TORQUE_NM
            elif name in WRIST_JOINTS:
                kp, kv, torque = WRIST_KP, WRIST_KV, WRIST_TORQUE_NM
            elif name == "Slider 2":
                kp, kv, torque = LIFT_KP, LIFT_KV, None  # not a rotary Nm rating -- see below
            else:
                kp, kv, torque = BODY_KP, BODY_KV, BODY_TORQUE_NM
            lo, hi = model.jnt_range[j]
            # Slider 2 is a linear (prismatic) joint -- its own force budget
            # isn't part of the user's Nm spec (that's all rotary servos).
            # Left at the same placeholder 100N the rest of this script's
            # gains started from; call this out explicitly rather than
            # silently reusing a number that looks like it came from the
            # same real spec as the torque ceilings around it.
            forcerange = 100.0 if torque is None else torque
            lines.append(
                f'  <position name="act_{name}" joint="{name}" kp="{kp}" kv="{kv}" '
                f'ctrlrange="{lo:.6f} {hi:.6f}" '
                f'forcerange="-{forcerange} {forcerange}"/>'
            )
        elif name in SPIN_JOINTS:
            # Torso yaw -- also waist-rated (see WAIST_JOINTS' own comment).
            lines.append(
                f'  <velocity name="act_{name}" joint="{name}" kv="{SPIN_KV}" '
                f'ctrlrange="-2 2" forcerange="-{WAIST_TORQUE_NM} {WAIST_TORQUE_NM}"/>'
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


def _build_pickup_object() -> str:
    """A small free-floating box for the Pick & Place demo. contype=8 (a bit
    of its own) / conaffinity=5 (bits 1+4) makes it collide with the
    tabletop/walls (bit 1) and, since it isn't part of the robot, the
    floor's own isolated bit (4, see the floor geom below) so a
    dropped/missed grasp lands on the floor instead of falling through it
    forever -- but deliberately *not* with the robot itself (contype 2,
    conaffinity 1): the frontend's scripted Pick & Place sequence carries
    this object welded to palm_right_1 (see _build_grasp_weld()) rather than
    through finger contact, and letting it also collide with the robot's own
    body was confirmed live to fail mid-carry -- as the arm swings the held
    object through its reach trajectory, it clips some other robot body
    (shoulder/torso) and the resulting contact impulse overpowers the weld,
    dropping the object. Finger/object contact was never load-bearing here
    anyway (the thumb chain lands ~0.5m from the other four fingertips
    regardless of joint values, so the fingers that do converge on the
    object approach from only one side with nothing to press it against);
    removing robot collision entirely just makes that explicit and stops it
    from actively breaking the carry, at the cost of the fingers not
    visually stopping right at the object's surface."""
    x, y, z = PICKUP_OBJECT_POS
    s = PICKUP_OBJECT_HALF_SIZE
    return (
        f'  <body name="pickup_object" pos="{x} {y} {z}">\n'
        f'    <freejoint name="pickup_object_free"/>\n'
        f'    <inertial pos="0 0 0" mass="{PICKUP_OBJECT_MASS}" diaginertia="0.0001 0.0001 0.0001"/>\n'
        f'    <geom name="pickup_object_geom" type="box" size="{s} {s} {s}" rgba="{PICKUP_OBJECT_RGBA}" '
        f'contype="8" conaffinity="5" friction="1.2 0.01 0.0002"/>\n'
        f"  </body>"
    )


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )



# Which body the grasp weld anchors to. NOT palm_right_1, despite that
# reading as the obvious choice -- palm_right_1's own body origin sits
# ~1.09m from where the closed hand actually is (the same "CAD body origin
# far from its own mesh" quirk documented elsewhere in this file for
# wheels/face_cover/etc, just much worse here), so a weld anchored there
# uses that 1.09m gap as a lever arm: confirmed by headless sim that any
# small residual arm-tracking error (a few thousandths of a radian, well
# within what BODY_KP's placeholder gain leaves under load while driving)
# gets amplified through that lever arm into the object visibly lagging
# behind or flying off during the carry, even though the weld itself never
# breaks (body1-body2 distance stays exactly constant throughout -- it's
# *where* it's anchored that's wrong, not the constraint). finger_tip_1 (one
# of the four fingers that actually converge on the object, see this
# function's own docstring) sits within ~5cm of the object at the grasp
# pose instead -- a >20x smaller lever arm -- confirmed via headless sim to
# make the same residual tracking error imperceptible.
GRASP_WELD_ANCHOR_BODY = "finger_tip_1"


def _build_grasp_weld(model: "mujoco.MjModel") -> str:
    """A weld equality constraint between the pickup object and
    GRASP_WELD_ANCHOR_BODY, created inactive -- the frontend flips
    data.eq_active on only once the scripted Pick & Place sequence has
    actually closed the fingers at the grasp pose, and back off on release.

    This exists because this CAD hand can't reliably hold the object through
    finger/object contact friction alone (confirmed by headless simulation:
    the thumb chain lands ~0.5m from the other four fingertips regardless of
    joint values -- effectively non-opposing -- so the four fingers that do
    converge on the object approach it from only one side, with nothing to
    press it against; closing them sweeps past the object rather than
    caging it). The weld is the load-bearing part of "holding" the object;
    the finger-closing animation happening alongside it is real (the same
    actuators, moved through their full range) but is not itself what keeps
    the object in the hand. See the FINGER_JOINTS-related note in this
    file's own outstanding-hardware list.

    relpose is computed here, once, against the *unmoved* object at
    PICKUP_OBJECT_POS with the arm at PICK_GRASP_QPOS and the base at
    PICK_PARK_POSE -- i.e. baked in as a static offset for this exact
    scripted approach, not a general grasp solver. Moving the object or
    changing either pose array without recomputing this would weld the
    object into empty space next to the hand instead of in it.
    """
    d = mujoco.MjData(model)

    def jaddr(name: str) -> int:
        return model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]

    d.qpos[jaddr("virtual_base_x")] = PICK_PARK_POSE[0]
    d.qpos[jaddr("virtual_base_y")] = PICK_PARK_POSE[1]
    d.qpos[jaddr("virtual_base_yaw")] = PICK_PARK_POSE[2]
    for name, val in zip(PICK_ARM_JOINTS, PICK_GRASP_QPOS):
        d.qpos[jaddr(name)] = val
    # pickup_object's free joint is left at its qpos0 default (on the table)
    # -- that *is* the grasp-moment position/orientation we want to weld.
    mujoco.mj_forward(model, d)

    anchor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, GRASP_WELD_ANCHOR_BODY)
    obj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pickup_object")
    anchor_mat = d.xmat[anchor].reshape(3, 3)
    rel_pos = anchor_mat.T @ (d.xpos[obj] - d.xpos[anchor])

    def quat_conj(q: np.ndarray) -> np.ndarray:
        return np.array([q[0], -q[1], -q[2], -q[3]])

    rel_quat = _quat_mul(quat_conj(d.xquat[anchor]), d.xquat[obj])

    pos_str = " ".join(f"{v:.6f}" for v in rel_pos)
    quat_str = " ".join(f"{v:.6f}" for v in rel_quat)
    return (
        "<equality>\n"
        f'  <weld name="grasp_weld" body1="pickup_object" body2="{GRASP_WELD_ANCHOR_BODY}" '
        f'relpose="{pos_str} {quat_str}" active="false"/>\n'
        "</equality>"
    )


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


# The rest of the actuated skeleton -- torso/waist, both shoulders, both
# elbows, both wrists, neck -- has the *same* root problem as the wheels
# above, just far worse: every one of these joints' child body sits with its
# own origin (jnt_pos defaults to that origin, i.e. 0,0,0 in the body's own
# frame -- confirmed: xanchor for each of these exactly equals its body's
# own xpos) up to 1.3m from where its mesh actually is, discovered chasing a
# live report that gestures made joints look "detached" -- wave's shoulder/
# elbow/wrist rotations (a modest 15-25deg each) were visibly swinging each
# mesh through a wide arc, worse at each joint further down the chain,
# because each one's own mesh sits at the *end* of an effectively 1m+ lever
# arm from its own rotation axis. Confirmed this is the same "origin far
# from its own mesh" export quirk as the wheels/face_cover/palm (documented
# elsewhere in this file) -- not a one-off on a few "special" bodies as
# earlier comments here assumed, but the norm for this entire CAD export.
#
# Unlike the wheels (a symmetric disc, so "the disc's own center" is
# unambiguously the right pivot), an elongated arm segment's mesh has no
# single obvious center to recenter onto -- the actual mechanical hinge is
# at the *seam* where it meets its neighbor, not at its own mesh's middle.
# Measured directly instead: for every (parent, child) pair below, the
# nearest point between the parent's mesh and the child's mesh, in world
# space at rest, comes out to 0.1-7mm apart -- i.e. the meshes already meet
# correctly at their true CAD-assembled seam. That seam point, converted
# into the child body's local frame, is the correct jnt_pos.
ARM_CHAIN_JOINTS: list[tuple[str, str, str]] = [
    ("Revolute 3", "wheelbase_lift_1", "rotation_waist_joint_1"),
    ("Revolute 4", "rotation_waist_joint_1", "chest_enclosure_2_1"),
    ("Revolute 5", "chest_enclosure_2_1", "rotation_shoulder_joint_1"),
    ("Revolute 6", "chest_enclosure_2_1", "rotation_shoulder_joint_2"),
    ("Revolute 7", "rotation_shoulder_joint_1", "rotation_arm_joint_1"),
    ("Revolute 8", "rotation_shoulder_joint_2", "rotation_arm_joint_2"),
    ("Revolute 9", "rotation_arm_joint_1", "arm_joint_1"),
    ("Revolute 10", "rotation_arm_joint_2", "arm_joint_2"),
    ("Revolute 11", "arm_joint_1", "forearm_joint_1"),
    ("Revolute 12", "arm_joint_2", "forearm_joint_2"),
    ("Revolute 26", "forearm_joint_1", "servo_spacer_hand_1"),
    ("Revolute 27", "forearm_joint_2", "servo_spacer_hand_2"),
    ("Revolute 43", "servo_spacer_hand_1", "palm_right_1"),
    ("Revolute 64", "servo_spacer_hand_2", "palm_left_1"),
    ("Revolute 24", "chest_enclosure_2_1", "neck_joint_3dp_1"),
    ("Revolute 25", "neck_joint_3dp_1", "face_cover_3_1"),
]
# A nearest-mesh-point match this far apart means the two bodies' meshes
# don't actually touch at rest -- ARM_CHAIN_JOINTS would be recentering onto
# the wrong point, not a real seam -- so treat it as a hard error rather
# than silently accepting a bad anchor.
_MAX_PLAUSIBLE_SEAM_GAP_M = 0.02


def _mesh_world_verts(model: "mujoco.MjModel", data: "mujoco.MjData", body_id: int) -> np.ndarray:
    """All mesh vertices belonging to body_id's geoms, transformed to world
    space at data's current qpos."""
    chunks = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body_id:
            continue
        mesh_id = model.geom_dataid[g]
        if mesh_id < 0:
            continue
        v0 = model.mesh_vertadr[mesh_id]
        n = model.mesh_vertnum[mesh_id]
        local = model.mesh_vert[v0 : v0 + n].reshape(-1, 3)
        rot = data.geom_xmat[g].reshape(3, 3)
        chunks.append(local @ rot.T + data.geom_xpos[g])
    if not chunks:
        raise ValueError(f"body {body_id} has no mesh geoms to recenter a joint onto")
    return np.concatenate(chunks, axis=0)


def _nearest_point_pair(a: np.ndarray, b: np.ndarray, sample: int = 2500) -> tuple[float, np.ndarray, np.ndarray]:
    """Closest pair of points between two point clouds (brute force, chunked,
    with random subsampling for speed on the larger meshes -- this only
    needs to find *a* point within the seam's own few-mm scale, not the
    mathematically exact nearest point)."""
    rng = np.random.default_rng(0)
    if len(a) > sample:
        a = a[rng.choice(len(a), sample, replace=False)]
    if len(b) > sample:
        b = b[rng.choice(len(b), sample, replace=False)]
    best_dist, best_a, best_b = float("inf"), a[0], b[0]
    for i in range(0, len(a), 200):
        chunk = a[i : i + 200]
        d2 = np.sum((chunk[:, None, :] - b[None, :, :]) ** 2, axis=2)
        ia, ib = np.unravel_index(np.argmin(d2), d2.shape)
        dist = float(np.sqrt(d2[ia, ib]))
        if dist < best_dist:
            best_dist, best_a, best_b = dist, chunk[ia], b[ib]
    return best_dist, best_a, best_b


def recenter_arm_joint_anchors(model: "mujoco.MjModel") -> None:
    """Move each ARM_CHAIN_JOINTS joint's anchor (jnt_pos) from wherever the
    CAD export left it (its child body's own, unrelated-to-geometry origin)
    to the actual seam between that joint's parent and child meshes -- see
    the module comment above. In place on the already-compiled model, same
    timing as recenter_wheel_geoms()."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for joint_name, parent_body, child_body in ARM_CHAIN_JOINTS:
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if j < 0:
            raise ValueError(f"expected joint '{joint_name}' not found in model")
        p_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, parent_body)
        c_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, child_body)
        if p_id < 0 or c_id < 0:
            raise ValueError(f"expected bodies '{parent_body}'/'{child_body}' for joint '{joint_name}'")
        gap, _, seam_world = _nearest_point_pair(
            _mesh_world_verts(model, data, p_id), _mesh_world_verts(model, data, c_id)
        )
        if gap > _MAX_PLAUSIBLE_SEAM_GAP_M:
            raise ValueError(
                f"joint '{joint_name}': nearest point between '{parent_body}' and '{child_body}' "
                f"meshes is {gap * 100:.1f}cm apart -- too far to be their real seam, ARM_CHAIN_JOINTS "
                "is probably wrong for this pair"
            )
        child_mat = data.xmat[c_id].reshape(3, 3)
        seam_local = child_mat.T @ (seam_world - data.xpos[c_id])
        offset = np.linalg.norm(seam_local - model.jnt_pos[j])
        model.jnt_pos[j] = seam_local
        print(f"{joint_name} ({parent_body} -> {child_body}): moved joint anchor {offset * 100:.1f}cm to the meshes' own seam")


# The Slider 2 lift column is a genuinely different problem from
# ARM_CHAIN_JOINTS above -- it's a *slide*, so there's no wrong-pivot/lever-
# arm effect (translating a rigid body by a fixed amount looks the same
# regardless of where its own local origin is; that failure mode is
# rotation-specific). Reported live instead: raising it opens a visible gap
# between wheelbase_lift_1's own moving mesh and base_link's fixed housing
# below it -- confirmed by measuring both meshes' own world bounding boxes:
# base_link's fixed housing tops out well below where wheelbase_lift_1's
# mesh sits even at the *lowest* end of Slider 2's travel, let alone at
# full extension. This isn't a jnt_pos bug -- moving the joint anchor can't
# close a gap between two independently-shaped meshes -- so the CAD simply
# has no housing tall enough to visually contain the column's own travel
# range. Real telescoping columns solve this with an outer tube long enough
# to contain the inner one's full stroke; this adds exactly that as a
# static box geom on base_link (so it doesn't move, only wheelbase_lift_1
# does), sized/positioned from the two meshes' own measured geometry so it
# always spans from the fixed housing's own top up through the moving
# box's lowest reach at full extension, with a margin.
_LIFT_SLEEVE_MARGIN_M = 0.02
_LIFT_SLEEVE_PAD_M = 0.008
_LIFT_SLEEVE_RGBA = "0.08 0.08 0.08 1"


def _build_lift_sleeve(model: "mujoco.MjModel") -> str:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def mesh_world_aabb(body_name: str) -> tuple[np.ndarray, np.ndarray]:
        verts = _mesh_world_verts(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name))
        return verts.min(axis=0), verts.max(axis=0)

    base_lo, base_hi = mesh_world_aabb("base_link")
    lift_lo, lift_hi = mesh_world_aabb("wheelbase_lift_1")

    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Slider 2")
    _, hi_range = model.jnt_range[j]

    world_z_lo = base_hi[2]
    world_z_hi = lift_lo[2] + hi_range + _LIFT_SLEEVE_MARGIN_M
    world_pos = np.array(
        [(lift_lo[0] + lift_hi[0]) / 2, (lift_lo[1] + lift_hi[1]) / 2, (world_z_lo + world_z_hi) / 2]
    )
    world_halfsize = np.array(
        [
            (lift_hi[0] - lift_lo[0]) / 2 + _LIFT_SLEEVE_PAD_M,
            (lift_hi[1] - lift_lo[1]) / 2 + _LIFT_SLEEVE_PAD_M,
            (world_z_hi - world_z_lo) / 2,
        ]
    )

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    base_mat = data.xmat[base_id].reshape(3, 3)
    local_pos = base_mat.T @ (world_pos - data.xpos[base_id])
    # A box's half-extents live along the *geom's own* local axes; since
    # base_link's rotation here is a pure axis permutation/flip (confirmed:
    # its xmat entries are all exactly 0/+-1, the up-axis fix from
    # resolve_urdf()), taking the absolute value of the same rotation
    # applied to the world half-size vector maps each world-axis extent onto
    # the correct local axis exactly, with no shear.
    local_halfsize = np.abs(base_mat.T) @ world_halfsize

    pos_str = " ".join(f"{v:.6f}" for v in local_pos)
    size_str = " ".join(f"{v:.6f}" for v in local_halfsize)
    print(
        f"lift sleeve: covers world z=[{world_z_lo:.3f}, {world_z_hi:.3f}] "
        f"(base housing top -> lift column's own lowest reach at full extension + margin)"
    )
    return f'<geom name="lift_sleeve" type="box" pos="{pos_str}" size="{size_str}" rgba="{_LIFT_SLEEVE_RGBA}" contype="0" conaffinity="0"/>'


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


# The shoulder/arm/hand servo-housing "brackets" (mesh names below). Each one
# is rigidly fixed to only *one* side of the joint it visually sits across
# (a modeling artifact of this CAD export -- the actual motor housing spans
# both the fixed and rotating halves of a real joint, but the export only
# carries it as a single rigid mesh glued to whichever body happened to own
# it), so once that joint rotates by a real amount -- exactly what every
# scripted gesture and Pick & Place now do -- the bracket visibly separates
# from the part it's supposed to sit flush against. Confirmed live in-browser
# (waving, reaching): every one of these joints shows the gap, not just one.
# An earlier revision of this script hid these meshes for exactly this
# reason, then un-hid them at the user's request to keep them visible as
# landmarks; re-hiding now that real, larger-range joint motion (reach/lower/
# wave) makes the separation clearly read as a broken/detached part rather
# than a cosmetic gap.
DUMMY_ACTUATOR_MESHES = [f"actuator_dummy_{i}" for i in range(1, 11)] + ["actuator_stepper_dummy_1"]


def hide_dummy_actuator_meshes(mjcf_text: str) -> str:
    """Push every geom referencing a DUMMY_ACTUATOR_MESHES mesh into render
    group 3 -- MujocoViewer.tsx's own scene builder already skips any geom
    with geom_group >= 3 (`if (!(model.geom_group[g] < 3)) continue;`,
    mirroring MuJoCo's own `simulate` viewer convention), so this only stops
    them from being *drawn*. group is a pure visualization tag in MuJoCo,
    read by nothing else -- collision (contype/conaffinity, unaffected by
    this) and every dynamics computation are untouched, so this cannot
    change how the robot moves or collides, only how it's drawn. Text-level
    for the same reason disable_wheel_floor_collision() is: mj_saveLastXML
    doesn't round-trip in-memory geom_group edits made on the compiled
    model, only geom_pos/jnt_pos-style fields do (confirmed the same way)."""
    mesh_names = tuple(DUMMY_ACTUATOR_MESHES)

    def _hide(match: "re.Match[str]") -> str:
        tag = match.group(0)
        if not re.search(rf'mesh="({"|".join(mesh_names)})"', tag):
            return tag
        tag = re.sub(r'\s*group="[^"]*"', "", tag)
        return tag[:-2] + ' group="3"/>'

    fixed = re.sub(r"<geom [^>]*/>", _hide, mjcf_text)
    print(f"hid {len(mesh_names)} dummy actuator-housing meshes (render group 3, collision unchanged)")
    return fixed


def main() -> None:
    resolved_urdf = sanitize_inertias(resolve_urdf())
    RESOLVED_URDF_PATH.write_text(resolved_urdf)
    print(f"wrote resolved URDF -> {RESOLVED_URDF_PATH.relative_to(ROOT.parent)}")

    model = mujoco.MjModel.from_xml_path(str(RESOLVED_URDF_PATH))
    print(f"loaded MJCF model: {model.nbody} bodies, {model.njnt} joints, {model.nq} qpos, {model.ngeom} geoms")

    recenter_wheel_geoms(model)
    recenter_arm_joint_anchors(model)
    lift_sleeve_xml = _build_lift_sleeve(model)
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

    # See DUMMY_ACTUATOR_MESHES/hide_dummy_actuator_meshes()'s own comment:
    # these were hidden, then un-hidden at the user's request, and are now
    # re-hidden at a later request once real joint motion made the
    # single-sided-bracket gap read as broken rather than cosmetic.
    mjcf_text = hide_dummy_actuator_meshes(mjcf_text)
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

    # Per-joint *passive* damping -- small, uniform, numerical-stability-only
    # (real per-joint response damping is each position actuator's own kv
    # term now, see the gains section above; this is on top of that, not
    # instead of it).
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
    # Integrator: implicitfast. RK4 was tried (it happened to paper over an
    # earlier version of the Pick & Place grasp weld bug, see
    # GRASP_WELD_ANCHOR_BODY's own comment for the actual bug and fix) and
    # reverted -- RK4 broke the base's own velocity-actuated driving
    # outright (confirmed: commanding act_base_vx under RK4 produces an
    # immediate NaN and the base never moves), which is this simulator's
    # most-used, most-tested feature. Not worth revisiting unless a future
    # feature specifically needs it, and even then it'd have to be a
    # per-phase switch (implicitfast while driving, something else while
    # holding still), not a single global setting.
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

    # Added *after* the blanket gravcomp injection above, deliberately: this
    # is a normal passive prop, not a robot link fighting a placeholder
    # position-gain -- it should fall/rest under ordinary gravity, not float.
    mjcf_text = mjcf_text.replace("</worldbody>", f"{_build_pickup_object()}\n</worldbody>", 1)

    # Lift sleeve: a plain <geom>, added directly inside base_link's own
    # <body> block (right after its own two mesh geoms, the last thing
    # there before base_link's nested <body name="wheelbase_lift_1">) so it
    # rides along with the base rigidly, contype/conaffinity=0 since it's
    # purely a visual fill-in, not a real part -- see _build_lift_sleeve()'s
    # own comment.
    _base_link_last_geom = '<geom type="mesh" rgba="0.7 0.7 0.7 1" mesh="base_link"/>'
    if mjcf_text.count(_base_link_last_geom) != 1:
        raise ValueError("expected exactly one base_link collision geom line to anchor the lift sleeve on")
    mjcf_text = mjcf_text.replace(_base_link_last_geom, f"{_base_link_last_geom}\n          {lift_sleeve_xml}", 1)

    mjcf_text = mjcf_text.replace("</mujoco>", actuator_xml + "\n</mujoco>")
    OUTPUT_MJCF_PATH.write_text(mjcf_text)

    # _build_grasp_weld() needs a compiled model that actually has the
    # pickup object in it (added as text above, so the in-memory `model`
    # from earlier in this function doesn't have it) -- reload what was just
    # written, purely to compute the weld's relpose, then append it and
    # write again. Loaded via from_xml_path (not from_xml_string) so the
    # "meshes/..." paths resolve relative to this file's own directory, same
    # as every other load in this script.
    staging_model = mujoco.MjModel.from_xml_path(str(OUTPUT_MJCF_PATH))
    mjcf_text = mjcf_text.replace("</mujoco>", _build_grasp_weld(staging_model) + "\n</mujoco>")
    OUTPUT_MJCF_PATH.write_text(mjcf_text)

    # Verify the saved MJCF is loadable standalone before declaring success.
    mujoco.MjModel.from_xml_path(str(OUTPUT_MJCF_PATH))
    print(f"wrote MJCF -> {OUTPUT_MJCF_PATH.relative_to(ROOT.parent)}")


if __name__ == "__main__":
    main()
