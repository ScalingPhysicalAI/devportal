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

import json
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

# Purchased kitchen environment asset (see model/scripts/convert_kitchen_obj.py
# for how model/kitchen/new+kitchen.obj became these per-part STLs + this
# manifest -- run once, checked in like any other static build input, not
# regenerated here).
KITCHEN_DIR = ROOT / "kitchen"
KITCHEN_MESHES_DIR = KITCHEN_DIR / "meshes"
KITCHEN_MANIFEST_PATH = KITCHEN_DIR / "manifest.json"

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

# Pick-and-place demo prop: a small free-floating box. Position/reach below
# (PICKUP_OBJECT_POS/PICK_GRASP_QPOS/PICK_PARK_POSE) were solved against the
# *previous* procedural room's own bench_n1 counter, which no longer exists
# now that the room is the imported kitchen asset (see _build_room()'s own
# comment) -- left as-is rather than deleted (the user's own call: replace
# the room, not rip out the Pick & Place machinery), but the object now
# spawns at a world position that may not land on any real counter surface
# in the new kitchen. Re-solving this against the new layout is a separate,
# not-yet-requested follow-up.
# (Original comment, still accurate for how these two files' numbers relate
# to each other:) Position is bench_n1's own (cx, cy) plus a fixed offset
# toward the table's front (room-facing) edge -- see MujocoViewer.tsx's
# PICK_PARK_POSE/PLACE_PARK_POSE comments for how the frontend derives where
# the base must park to reach it
# (the two are solved together: this file fixes the object's world position,
# the frontend's arm IK -- solved offline, see its own comment -- fixes the
# base's position/heading *relative to the object*, and moving the object
# here without re-deriving that offset will make the reach miss).
PICKUP_OBJECT_HALF_SIZE = 0.04  # 8cm cube
PICKUP_OBJECT_POS = (2.0, -1.0, 0.02 + PICKUP_OBJECT_HALF_SIZE)  # on the floor
# ^ was (-3.0, 4.85, ...), sitting on "bench_n1" in the previous procedural
# room -- that room no longer exists (see _build_room()'s own comment), and
# that old position now lands 51cm inside the new kitchen's own wall/ceiling
# shell mesh. Confirmed live: the resulting overlap made the object explode
# outward on the very first physics step, hard enough to visibly deform the
# robot's own joints on load (see _build_kitchen_import()'s own comment for
# the actual mechanism). This new position is a plain, empty floor spot,
# verified clear of every kitchen collision volume in
# _build_kitchen_collision_proxy() -- not a counter surface (Pick & Place's
# own reach/park pose below still target the old, no-longer-valid layout
# and remain broken, per the user's own earlier call), just a safe place
# for the object to rest without incident.
PICKUP_OBJECT_MASS = 0.15  # kg -- light enough for the placeholder finger actuators to hold
PICKUP_OBJECT_RGBA = "0.95 0.45 0.1 1"

# The kitchen counter's own coffee cup (FrontColorcup in
# model/kitchen/manifest.json -- see convert_kitchen_obj.py's own comment on
# how it was found/colored), promoted from fixed decoration to a real,
# free-floating, grabbable body -- the user's own ask: "make the cups and
# coffee machine live... let's start with the cup." A plain cylinder here
# rather than the cup's own (much more detailed, but visual-only) mesh --
# same reasoning as PICKUP_OBJECT_HALF_SIZE's box above: a simple primitive
# gives clean, predictable collision geometry for the same placeholder
# finger/weld grasp this file already has, without needing that mesh
# recentered around its own local origin first (its vertices, like every
# kitchen part's, are baked to absolute room-frame positions, not centered
# on the object itself -- fine for a static contype=0 decoration, not
# directly usable as a moving body's own collision geom).
# Originally measured directly off that mesh's own bounds: center (0.532,
# 1.939, 0.947), extents (0.090, 0.114, 0.085) in the kitchen's own room
# frame -- the counter by the sink the user already pointed out this cup
# sits on. Moved from there to here (still the same counter, just its own
# east edge) per the user's own ask, after extensive verified dynamics
# testing (see MujocoViewer.tsx's PICK_GRASP_QPOS comment) established that
# no arm pose -- with or without waist bend -- can close the ~0.58m gap
# from PICK_PARK_PICK to the old spot without an animated assist, but a
# forward waist bend *can* bring the palm to within about 10cm of a spot
# like this one for real, letting the pickup finally look like an actual
# reach instead of the cup sliding into the hand. First landed this at
# (0.95, 1.93, ...), right at the island's own x<=1.0 edge -- reported live
# as looking like the cup was hanging half off the counter -- so it moved in
# another 10cm to here, still well within PICK_GRASP_QPOS's reach (see that
# constant's own comment for the pose this pairs with, solved specifically
# to reach *this* spot, not the old one). The saucer moved the same total
# delta from its own original spot (see convert_kitchen_obj.py's
# _SAUCER_SHIFT); z stays close to the original (island counter top 0.91 +
# this cup's own half-height 0.043).
CUP_OBJECT_POS = (0.85, 1.93, 0.953)
CUP_OBJECT_RADIUS = 0.045
CUP_OBJECT_HALF_HEIGHT = 0.043
CUP_OBJECT_MASS = 0.12  # kg -- lighter than the placeholder box, this is just a cup
CUP_OBJECT_RGBA = "0.36 0.20 0.09 1"  # same brown as the cup's own static color

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


def _build_cup_object() -> tuple[str, str]:
    """The counter's own coffee cup, as a real body -- see CUP_OBJECT_POS's
    own comment for what it replaces. Two geoms, same split as every other
    piece of kitchen furniture (see _build_kitchen_import()'s own comment
    on visual mesh vs. collision proxy): the cup's own real mesh
    (kitchen_cup_object_visual.stl -- a recentered copy of the same
    geometry FrontColorcup renders everywhere else, see
    convert_kitchen_obj.py's own comment on why it needs its own local
    origin) for how it looks, contype=0/conaffinity=0 since a detailed
    non-convex mesh makes a poor collision shape; a plain cylinder,
    invisible (group=3, see the kitchen collision boxes' own convention)
    for actually colliding, contype=8/conaffinity=5 -- same reasoning as
    _build_pickup_object(): collides with the counter/floor/walls (bits 1
    and 4) but not the robot's own body (bit 2), since holding it is a
    kinematic follow, not finger contact (see MujocoViewer.tsx's
    GRABBABLE_OBJECTS -- this is the second body it can hold, alongside
    pickup_object)."""
    x, y, z = CUP_OBJECT_POS
    r, h = CUP_OBJECT_RADIUS, CUP_OBJECT_HALF_HEIGHT
    asset_xml = '  <mesh file="meshes/kitchen_cup_object_visual.stl" name="kitchen_cup_object_visual"/>'
    body_xml = (
        f'  <body name="cup_object" pos="{x} {y} {z}">\n'
        f'    <freejoint name="cup_object_free"/>\n'
        f'    <inertial pos="0 0 0" mass="{CUP_OBJECT_MASS}" diaginertia="0.0001 0.0001 0.0001"/>\n'
        f'    <geom name="cup_object_visual" type="mesh" mesh="kitchen_cup_object_visual" rgba="{CUP_OBJECT_RGBA}" '
        f'contype="0" conaffinity="0"/>\n'
        f'    <geom name="cup_object_geom" type="cylinder" size="{r} {h}" group="3" contype="8" conaffinity="5" '
        f'friction="1.2 0.01 0.0002"/>\n'
        f"  </body>"
    )
    return asset_xml, body_xml


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


# Manual (teleop) grip does NOT use an equality weld the way grasp_weld
# above does, despite that being the obvious design (and the first one
# tried here). Two dead ends, in order:
#
# 1. Created inactive, MujocoViewer.tsx flips data.eq_active on/off around
#    each grab. This engine build's JS bindings throw
#    ("_emval_take_value has unknown type ...memory_viewIbEE") on *any*
#    touch of a bool-typed mjData/mjModel array, which is exactly what
#    eq_active is -- confirmed directly (both data.eq_active and
#    model.eq_active0 throw on a bare *read*, let alone a write). Every
#    numeric array (eq_data included) is fine; it's specifically the
#    boolean ones this build can't marshal, so this would crash the first
#    time any user actually closed a hand near something.
# 2. Leave the weld permanently active="true" instead, and have
#    MujocoViewer.tsx make it a no-op while not gripping by continuously
#    overwriting relpose, every frame, to match whatever the *current*
#    true relative pose already is (zero error -> zero force, in theory).
#    Confirmed live (headless, stepping the compiled model directly) that
#    this doesn't actually work: a weld constrains *relative motion*, not
#    just relative position at the instant it's set, so an "active" weld
#    with a relpose that's merely one physics step stale still resists
#    ordinary gravity/contact motion on both bodies every single step --
#    over hundreds of steps this measurably drags the free object toward
#    the hand (or vice versa) even though the JS was trying to keep it
#    inert. There's no way to make an *active* weld truly inert short of
#    recomputing relpose fully within the same step the solver uses it,
#    which the discrete step loop doesn't allow.
#
# What actually works, and is what MujocoViewer.tsx does: no equality
# constraint at all for manual grip. On grab, record the object's pose
# relative to the anchor fingertip (same relative-pose math as above, just
# computed live); every frame while held, directly overwrite the object's
# own freejoint qpos to (anchor's current pose) composed with that fixed
# offset, and zero its qvel -- a kinematic follow, not a physics
# constraint. This needs nothing from mjModel/mjData beyond the plain
# numeric arrays already confirmed working everywhere else in this file.
# grasp_weld above is left alone (still active="false", toggled via the
# same broken data.eq_active) since Pick & Place is already unreachable
# from the UI and documented elsewhere as broken/stale regardless -- not
# worth the same rework for a dead code path.


def _build_kitchen_import() -> tuple[str, str]:
    """Reads model/kitchen/manifest.json (see convert_kitchen_obj.py) and
    returns (asset_xml, geom_xml) for the purchased kitchen furniture set --
    one <mesh> asset declaration and one flat-colored static <geom> per part,
    already positioned (the conversion script bakes axis conversion and
    recentering directly into each part's own vertex data -- see that
    script's own comment) so these need no per-geom pos/quat here.

    contype/conaffinity are both 0 -- these are *visual only*, deliberately
    not collidable. MuJoCo approximates every <geom type="mesh"> as its own
    convex hull for collision, and each of these 106 parts was merged (in
    convert_kitchen_obj.py) from every instance of one *material* across the
    whole room, not one physical object -- e.g. the wall/ceiling shell mesh
    is hollow in reality, but its convex hull is a single solid block
    filling ~90% of the room's own bounding volume (measured directly:
    239.6 of a possible ~264 m^3). Using that for collision meant the robot
    spawned already overlapping a giant invisible solid block, got
    violently ejected on the very first physics step (this is what the
    user saw as "something hit it and its body got deformed" -- a real
    contact impulse, not a rendering glitch), and once pushed past the
    hull's own boundary was in genuinely empty, uncollidable space outside
    it -- explaining both reported symptoms (can't move inside the kitchen,
    moves freely once outside it) as the same root cause. Real collision
    now comes entirely from _build_kitchen_collision_proxy()'s own simple,
    hand-measured boxes instead."""
    manifest = json.loads(KITCHEN_MANIFEST_PATH.read_text())
    asset_lines = []
    geom_lines = []
    for entry in manifest:
        if entry["material"] in _KITCHEN_DYNAMIC_MATERIALS:
            # This one's a real, pickable object now (see _build_cup_object()
            # and friends) instead of fixed kitchen dressing -- skip its
            # static copy here or the room would show two cups, one of them
            # a ghost the robot can walk straight through.
            continue
        mesh_name = pathlib.Path(entry["file"]).stem
        rgba = " ".join(f"{c:.4f}" for c in entry["rgba"])
        asset_lines.append(f'  <mesh file="meshes/{entry["file"]}" name="{mesh_name}"/>')
        geom_lines.append(
            f'  <geom name="{mesh_name}" type="mesh" mesh="{mesh_name}" rgba="{rgba}" contype="0" conaffinity="0"/>'
        )
    return "\n".join(asset_lines), "\n".join(geom_lines)


# Materials pulled out of the static kitchen import above because they're
# being promoted to real, free-floating, pickable bodies instead (see the
# _build_*_object() functions below) -- kept as a set rather than one-off
# special-casing so the next object (the coffee machine, per the user's own
# "let's start with the cup" ask) is a one-line addition here plus its own
# _build_*_object() function, not a rewrite of this loop.
_KITCHEN_DYNAMIC_MATERIALS = {"FrontColorcup"}


# Simple box colliders standing in for the imported kitchen's own (unusable
# for collision, see _build_kitchen_import()'s own comment) geometry --
# group="3" so loadHumanoidScene.ts's three.js loader hides them (it skips
# any geom_group >= 3, the same convention the robot's own collision-only
# geoms already use), while MuJoCo itself still collides against them
# normally. Dimensions are real measurements, not guessed: the wall
# boundary from Stucco_A02_Color_50cm_White's own bounds; the three counter
# blocks below from running trimesh's connected-components split() on
# Cozinha_Bancada_Marmore_Biancone_120cm (the counter/island marble) and
# Wood_Mahogany_33_46_100cm (the cabinet carcasses), which resolves into
# distinct clusters rather than one bounding box spanning all of them.
#
# A single box covering the *combined* bounding rectangle of every counter
# cluster (the original version of this proxy) was reported live as
# blocking a walkway a user could see was open in the render -- the real
# layout is three separate pieces (west-wall counter run, island, north
# counter/pantry run) with genuine gaps between them, and one bounding
# rectangle around all three swallows those gaps along with the counters
# themselves. These three boxes instead track each cluster's own connected-
# component bounds (with a small pad), leaving the real gap between the
# island (KITCHEN_ISLAND, y up to 3.0) and the north run
# (KITCHEN_NORTH_RUN, y from 3.8) open, matching the actual walkway.
_KITCHEN_WALL_BOUNDS = ((-3.73, 3.73), (-2.75, 7.71), (0.0, 3.70))
_KITCHEN_WALL_THICKNESS = 0.12
# The west counter isn't one uniform-depth run -- its own countertop mesh
# (Cozinha_Bancada_Marmore_Biancone_120cm) is L-shaped: only 0.6m deep from
# the fridge/window past the appliances (x=-3.5..-2.91, measured off that
# mesh's own vertices), then steps out to 1.5m deep near the shelf/jars
# corner (x=-3.5..-1.98) from y=3.87 on. The old single box used the *deep*
# width for the whole run, which swallowed the entire walkway between the
# counter and the island along the shallow stretch -- the user's own report
# ("robot cannot enter from the fridge side"). Two boxes matching the real
# footprint (small pad beyond each measured edge) instead of one oversized
# rectangle. z1=2.5 on both runs originally spanned from the floor clean
# through the base cabinets, the countertop, *and* the wall cabinets above
# it as one solid block -- deliberately, to stop the robot walking through
# any of it. That went uncorrected at the time (unlike the island's own z1,
# see its comment below) because nothing needed to physically touch this
# counter's own *surface* until the coffee machine workflow's own
# place-down did -- confirmed live: a cup placed at the counter's real,
# measured height (0.909, see CUP_OBJECT_POS's own comment) came out ~14cm
# deep inside this box's old solid interior, and the resulting shove sent
# it flying across the room instead of resting on the counter. This splits
# each run into its own counter-height slab (matching the island's own
# 0.91) plus a separate wall-cabinet slab starting at 1.4 -- a typical
# countertop-to-wall-cabinet clearance, and more than enough for this rig's
# own reach -- so there's an open gap right at the counter surface for
# something to actually rest in.
_KITCHEN_WEST_COUNTER_SHALLOW_COUNTER_BOUNDS = ((-3.6, -2.85), (1.6, 3.9), (0.0, 0.91))
_KITCHEN_WEST_COUNTER_SHALLOW_CABINET_BOUNDS = ((-3.6, -2.85), (1.6, 3.9), (1.4, 2.5))
_KITCHEN_WEST_COUNTER_DEEP_COUNTER_BOUNDS = ((-3.6, -1.9), (3.9, 4.65), (0.0, 0.91))
_KITCHEN_WEST_COUNTER_DEEP_CABINET_BOUNDS = ((-3.6, -1.9), (3.9, 4.65), (1.4, 2.5))
# z1=1.0 here used to be a rough guess, well above the island's own real
# countertop mesh (Cozinha_Ilha_Madeira_Bancada_280x50cm / the marble slab
# it shares with the west counter, Cozinha_Bancada_Marmore_Biancone_120cm --
# both measured, top face at z=0.909). Harmless while this box only had to
# stop the robot from walking through the island -- but cup_object
# (urdf_to_mjcf.py's _build_cup_object(), placed at the counter's real
# surface height) rested with its collision cylinder's bottom at z=0.904,
# which put nearly the whole cylinder *inside* this box's old top. MuJoCo's
# contact solver resolved that overlap by firing the cup straight up on the
# very first step -- reported live as "the cup is way up in the air". 0.91
# (a hair above the measured 0.909) leaves only ~5mm of harmless overlap
# instead.
_KITCHEN_ISLAND_BOUNDS = ((-1.95, 1.0), (1.6, 3.0), (0.0, 0.91))
_KITCHEN_NORTH_RUN_BOUNDS = ((-1.85, 0.85), (3.8, 6.0), (0.0, 2.9))
# The center table + its surrounding chairs (all exported under the shared
# "Cadeira_Mesa__010_*" materials -- combined bounding footprint of all
# three, measured off the raw meshes) had no collision proxy at all until
# now -- reported live as "the robot can walk into the chairs". A single
# box over the whole cluster, same simplification as every other furniture
# group here.
_KITCHEN_CENTER_TABLE_BOUNDS = ((-1.75, 0.77), (1.16, 1.85), (0.0, 0.86))


def _box_geom(name: str, bounds: tuple) -> str:
    (x0, x1), (y0, y1), (z0, z1) = bounds
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    hx, hy, hz = (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2
    return f'  <geom name="{name}" type="box" group="3" pos="{cx} {cy} {cz}" size="{hx} {hy} {hz}" contype="1" conaffinity="1"/>'


def _build_kitchen_collision_proxy() -> str:
    (x0, x1), (y0, y1), (z0, z1) = _KITCHEN_WALL_BOUNDS
    t = _KITCHEN_WALL_THICKNESS
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    hx, hy, hz = (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2
    lines = [
        f'  <geom name="kitchen_wall_south" type="box" group="3" pos="{cx} {y0} {cz}" size="{hx + t} {t} {hz}" contype="1" conaffinity="1"/>',
        f'  <geom name="kitchen_wall_north" type="box" group="3" pos="{cx} {y1} {cz}" size="{hx + t} {t} {hz}" contype="1" conaffinity="1"/>',
        f'  <geom name="kitchen_wall_east" type="box" group="3" pos="{x1} {cy} {cz}" size="{t} {hy + t} {hz}" contype="1" conaffinity="1"/>',
        f'  <geom name="kitchen_wall_west" type="box" group="3" pos="{x0} {cy} {cz}" size="{t} {hy + t} {hz}" contype="1" conaffinity="1"/>',
        # A flat slab under the whole room, top face at z=0 (the imported
        # floor mesh's own height, see convert_kitchen_obj.py's own comment
        # on why it's rolled to sit there). The comment that used to sit
        # above _build_pickup_object() already named this floor's own bit
        # (4, distinct from the walls/counters' bit 1) -- that geom just
        # never actually existed until now, confirmed live (and via headless
        # sim: pickup_object and a dropped/released cup_object both
        # free-fall clean through z=0 forever, since the only collision
        # volumes in this proxy were the counters/walls). contype/
        # conaffinity=4, NOT the walls/counters' bit 1: bit 1 is what the
        # robot's own body collides against (contype=2/conaffinity=1, see
        # this file's collision-bitmask comment), and the robot's own base
        # collision mesh -- unlike the drive wheels, which already had floor
        # collision explicitly disabled -- turns out to dip well below z=0
        # (it's centered near its body's own local pos, ~15cm below the
        # nominal ground height, with a large bounding radius covering the
        # whole wheel-well area). Reusing bit 1 here put that mesh into a
        # ~15cm interpenetration with this new floor the instant it existed,
        # which the solver fought by shoving back hard enough to cancel out
        # nearly all of WASD's own drive force -- reported live as "the
        # wheels spin but the robot doesn't move." Bit 4 is exactly the
        # isolated bit dynamic objects already carry in their own
        # conaffinity (see PICKUP_OBJECT_RGBA's/CUP_OBJECT_RGBA's own
        # contype="8" conaffinity="5" = bits 1+4) for precisely this
        # purpose: land on the floor without the robot's own body ever
        # touching it, since the robot's height is fixed by its own virtual
        # joint regardless of floor contact anyway (see resolve_urdf()).
        f'  <geom name="kitchen_floor" type="box" group="3" pos="{cx} {cy} {-0.05}" size="{hx + t} {hy + t} 0.05" contype="4" conaffinity="4"/>',
        _box_geom("kitchen_west_counter_shallow_counter", _KITCHEN_WEST_COUNTER_SHALLOW_COUNTER_BOUNDS),
        _box_geom("kitchen_west_counter_shallow_cabinet", _KITCHEN_WEST_COUNTER_SHALLOW_CABINET_BOUNDS),
        _box_geom("kitchen_west_counter_deep_counter", _KITCHEN_WEST_COUNTER_DEEP_COUNTER_BOUNDS),
        _box_geom("kitchen_west_counter_deep_cabinet", _KITCHEN_WEST_COUNTER_DEEP_CABINET_BOUNDS),
        _box_geom("kitchen_island", _KITCHEN_ISLAND_BOUNDS),
        _box_geom("kitchen_north_run", _KITCHEN_NORTH_RUN_BOUNDS),
        _box_geom("kitchen_center_table", _KITCHEN_CENTER_TABLE_BOUNDS),
    ]
    return "\n".join(lines)


def _build_room() -> str:
    """The room is entirely the purchased kitchen asset now (see
    _build_kitchen_import()/convert_kitchen_obj.py) -- floor, walls,
    ceiling, cabinets, appliances, and decor all come from that one import,
    replacing the previous procedural box room (walls + Pick & Place counter
    benches) outright, per the user's own explicit call. No separate floor
    plane or wall geoms are added here anymore -- the import supplies its
    own (Flooring_Parquet_Parallel_H01_120cm and
    Stucco_A02_Color_50cm_White materials), visually. Real collision comes
    from _build_kitchen_collision_proxy()'s own invisible boxes instead --
    see _build_kitchen_import()'s own comment for why the imported mesh
    geometry itself can't be used for that."""
    _, kitchen_geoms = _build_kitchen_import()
    return kitchen_geoms + "\n" + _build_kitchen_collision_proxy()


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
    # Finger joints (both hands, 15 each) -- left out when this list was
    # first built since manual grip wasn't driving them yet, so the same
    # zeroed-jnt_pos CAD placeholder here went unnoticed (a joint that never
    # actually rotates can't show a wrong-pivot arc). Once grip started
    # commanding real qpos, they showed exactly the same symptom as every
    # other un-recentered joint above: fingers swinging away from the hand
    # in a wide arc instead of curling at their own knuckle. Parent bodies
    # here are the *compiled* MJCF parents, not the URDF ones -- several
    # URDF links in each finger's root joint (e.g. "finger_knuckle_1") are
    # fixed/weld joints that MuJoCo's compiler fuses into their own parent
    # body, so the joint's real mechanical parent ends up being palm_right_1/
    # palm_left_1 directly (confirmed via body_parentid on the compiled
    # model, not assumed from the URDF's own parent tags).
    ("Revolute 48", "palm_right_1", "thumb_knuckle_1"),
    ("Revolute 49", "palm_right_1", "finger_rear_1"),
    ("Revolute 50", "palm_right_1", "finger_rear_2"),
    ("Revolute 51", "palm_right_1", "finger_rear_3"),
    ("Revolute 52", "palm_right_1", "finger_rear_4"),
    ("Revolute 53", "finger_rear_1", "finger_mid_pinky_1"),
    ("Revolute 54", "finger_rear_2", "finger_mid_1"),
    ("Revolute 55", "finger_rear_3", "finger_mid_2"),
    ("Revolute 56", "finger_rear_4", "finger_mid_small_1"),
    ("Revolute 57", "thumb_knuckle_1", "finger_mid_small_2"),
    ("Revolute 58", "finger_mid_pinky_1", "finger_tip_small_1"),
    ("Revolute 59", "finger_mid_1", "finger_tip_1"),
    ("Revolute 60", "finger_mid_2", "finger_tip_2"),
    ("Revolute 61", "finger_mid_small_1", "finger_tip_small_2"),
    ("Revolute 62", "finger_mid_small_2", "finger_tip_small_3"),
    ("Revolute 69", "palm_left_1", "thumb_knuckle_2"),
    ("Revolute 70", "palm_left_1", "finger_rear_5"),
    ("Revolute 71", "palm_left_1", "finger_rear_6"),
    ("Revolute 72", "palm_left_1", "finger_rear_7"),
    ("Revolute 73", "palm_left_1", "finger_rear_8"),
    ("Revolute 74", "finger_rear_5", "finger_mid_pinky_2"),
    ("Revolute 75", "finger_rear_6", "finger_mid_3"),
    ("Revolute 76", "finger_rear_7", "finger_mid_4"),
    ("Revolute 77", "finger_rear_8", "finger_mid_small_3"),
    ("Revolute 78", "thumb_knuckle_2", "finger_mid_small_4"),
    ("Revolute 79", "finger_mid_pinky_2", "finger_tip_small_4"),
    ("Revolute 80", "finger_mid_3", "finger_tip_3"),
    ("Revolute 81", "finger_mid_4", "finger_tip_4"),
    ("Revolute 82", "finger_mid_small_3", "finger_tip_small_5"),
    ("Revolute 83", "finger_mid_small_4", "finger_tip_small_6"),
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
    # Every mesh this script cares about gets re-copied in full below (both
    # the robot's own and the kitchen's) -- clear stale leftovers first, or
    # renaming/re-splitting a material (see convert_kitchen_obj.py) just
    # keeps piling up old copies here under their old names forever, since
    # the copy loops below only ever add files, never remove them.
    for stale in OUTPUT_MESHES_DIR.glob("*.stl"):
        stale.unlink()
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

    # Kitchen environment furniture meshes (see convert_kitchen_obj.py) --
    # copied and declared the same way as the robot's own meshes just above,
    # into the same meshes/ directory (flat, no subfolder, so
    # loadHumanoidScene.ts's generic "fetch every file referenced as
    # meshes/<name>" logic needs no changes to pick these up too). mj_saveLastXML
    # already wrote an <asset> block for the robot's own meshes; this just
    # appends more <mesh> entries to it via text injection, since these
    # parts were never part of the compiled `model` object above.
    for stl in KITCHEN_MESHES_DIR.glob("*.stl"):
        shutil.copy2(stl, OUTPUT_MESHES_DIR / stl.name)
    kitchen_asset_xml, _ = _build_kitchen_import()
    mjcf_text = mjcf_text.replace("<asset>", "<asset>\n" + kitchen_asset_xml, 1)

    # hide_dummy_actuator_meshes() is NOT called here (left defined, in case
    # a future joint's own mesh gap turns out not to be a jnt_pos bug the
    # way the rest of the arm chain's was): the actual cause of these
    # brackets visibly separating during motion was every joint's rotation
    # pivot sitting up to 1.3m from its own mesh (see
    # recenter_arm_joint_anchors() above) -- fixed there, not by hiding
    # anything. Brought back per the user's own request once that was fixed.
    mjcf_text = disable_wheel_floor_collision(mjcf_text)

    # The room -- entirely the imported kitchen asset now, including its own
    # floor mesh (see _build_room()'s own comment) -- shares collision bit 1
    # with the robot's own default conaffinity (see the
    # <default><geom .../></default> added below), so it stops the robot
    # from being driven through it, and the pickup object's own conaffinity
    # (bits 1+4, see _build_pickup_object()) already includes bit 1 too, so
    # it still has a floor to land on if dropped -- no separate procedural
    # floor plane is needed for that anymore. (The virtual planar base, see
    # resolve_urdf(), fixes the robot's own height directly regardless of
    # floor contact, same as before.)
    mjcf_text = mjcf_text.replace("<worldbody>", f"<worldbody>\n{_build_room()}\n", 1)

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
    cup_asset_xml, cup_body_xml = _build_cup_object()
    mjcf_text = mjcf_text.replace("<asset>", "<asset>\n" + cup_asset_xml, 1)
    mjcf_text = mjcf_text.replace(
        "</worldbody>", f"{_build_pickup_object()}\n{cup_body_xml}\n</worldbody>", 1
    )

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
