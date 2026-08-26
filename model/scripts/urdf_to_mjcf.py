#!/usr/bin/env python3
"""Convert the Humanoid_description ROS2/xacro package into a native MuJoCo
MJCF model.

The source URDF is a xacro file exported by a SolidWorks-to-URDF plugin. It
only uses xacro for three top-level <xacro:include> directives (materials,
ros2control, gazebo) and $(find Humanoid_description) path substitution --
no macros, properties, or conditionals -- so it's resolved here with plain
string/XML processing instead of pulling in the `xacro` ROS package.

Usage:
    python3 model/scripts/urdf_to_mjcf.py
"""

import pathlib
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

MIN_EIGENVALUE = 1e-9
MIN_MASS = 1e-6  # kg

ROOT = pathlib.Path(__file__).resolve().parent.parent
DESCRIPTION_DIR = ROOT / "Humanoid_description"
MESHES_DIR = DESCRIPTION_DIR / "meshes"
XACRO_PATH = DESCRIPTION_DIR / "urdf" / "Humanoid.xacro"
MATERIALS_PATH = DESCRIPTION_DIR / "urdf" / "materials.xacro"

RESOLVED_URDF_PATH = DESCRIPTION_DIR / "urdf" / "Humanoid.resolved.urdf"
OUTPUT_DIR = ROOT / "mjcf"
OUTPUT_MESHES_DIR = OUTPUT_DIR / "meshes"
OUTPUT_MJCF_PATH = OUTPUT_DIR / "humanoid.xml"


def resolve_urdf() -> str:
    """Inline the materials include, drop the ros2control/gazebo includes
    (irrelevant to a physics-only MJCF model), and rewrite mesh paths from
    the ROS `$(find pkg)` package syntax to plain paths relative to the
    resolved URDF's own directory."""
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
    # low Y, the head/face_cover sits at high Y -- confirmed by inspecting
    # body positions at rest), not Z. MuJoCo's gravity and floor-plane
    # convention here is Z-up, so without correcting for this the robot
    # spawns lying on its side. Rolling +90deg about X maps local +Y onto
    # world +Z, standing it upright.
    #
    # This robot has a narrow two-wheel-plus-caster footprint under a much
    # heavier upper body/arms, with no balance controller -- given a free
    # 6-DOF joint it topples the instant gravity is applied, no matter how
    # gently it's placed (confirmed: even resting in full contact with the
    # floor from frame one, it tips over within a couple of seconds). So the
    # base is welded to the world at its resting height instead of floating
    # -- it can never fall or tip. The arms/waist/fingers/wheels keep their
    # own joints and still move under real physics; only the base is fixed.
    # Weld height = the rest-pose depth of the lowest mesh vertex (the
    # caster wheel, ~0.112m below base_link's origin in this orientation)
    # plus a hair of clearance so it isn't embedded in the floor.
    resolved = resolved.replace(
        "</robot>",
        '<link name="world"/>\n'
        '<joint name="root_weld" type="fixed">\n'
        '  <origin xyz="0 0 0.115" rpy="1.5707963267948966 0 0"/>\n'
        '  <parent link="world"/>\n'
        '  <child link="base_link"/>\n'
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


def main() -> None:
    resolved_urdf = sanitize_inertias(resolve_urdf())
    RESOLVED_URDF_PATH.write_text(resolved_urdf)
    print(f"wrote resolved URDF -> {RESOLVED_URDF_PATH.relative_to(ROOT.parent)}")

    model = mujoco.MjModel.from_xml_path(str(RESOLVED_URDF_PATH))
    print(f"loaded MJCF model: {model.nbody} bodies, {model.njnt} joints, {model.nq} qpos, {model.ngeom} geoms")

    OUTPUT_DIR.mkdir(exist_ok=True)
    OUTPUT_MESHES_DIR.mkdir(exist_ok=True)
    mujoco.mj_saveLastXML(str(OUTPUT_MJCF_PATH), model)

    # mj_saveLastXML carries over mesh <file> paths as given to the compiler,
    # which were relative to the resolved URDF's directory (Humanoid_description/urdf).
    # Copy the referenced meshes alongside the saved MJCF and rewrite the
    # paths to match, so model/mjcf/ is a self-contained, portable unit
    # (no reference back into Humanoid_description/).
    mjcf_text = OUTPUT_MJCF_PATH.read_text()
    for stl in MESHES_DIR.glob("*.stl"):
        if f'"../meshes/{stl.name}"' in mjcf_text:
            shutil.copy2(stl, OUTPUT_MESHES_DIR / stl.name)
    mjcf_text = mjcf_text.replace('file="../meshes/', 'file="meshes/')

    # Add a ground plane so the free-floating robot has something to land
    # on -- the source model/URDF has no floor of its own. It's given its
    # own contype/conaffinity bit (see the self-collision note below) so it
    # still collides with the robot despite the robot's own geoms no longer
    # colliding with each other.
    mjcf_text = mjcf_text.replace(
        "<worldbody>",
        '<worldbody>\n'
        '  <geom name="floor" type="plane" size="5 5 0.1" rgba="0.25 0.25 0.28 1" contype="1" conaffinity="1"/>\n',
        1,
    )

    # Two fixes for simulation stability, both discovered by stepping the
    # model headlessly and inspecting where it diverges from its rest pose:
    #
    # 1. Self-collision: several adjacent links (base_link and the
    #    wheelbase/wheels/casters mated to it) are snugly fit by design in
    #    the source CAD and their collision meshes overlap by several
    #    centimeters at rest (confirmed via mj_forward + data.contact).
    #    Stepping the physics from there makes the contact solver violently
    #    resolve that interpenetration in the first few frames. None of
    #    this model's *other* parts are in contact at rest, so the fix is
    #    to disable collisions between the robot's own geoms entirely
    #    (contype/conaffinity bit 2) while keeping floor collisions (bit 1)
    #    -- see the floor geom's contype/conaffinity above.
    #
    # 2. No actuation: this URDF's ros2_control hardware interface (which
    #    would normally hold each joint's commanded position) is dropped
    #    during conversion (see resolve_urdf), leaving 53 bare hinge/slide
    #    joints with zero damping. With gravity on and nothing to counter
    #    it, the arms/waist/fingers free-fall from their design pose within
    #    a couple of seconds. A moderate default joint stiffness+damping
    #    (a spring back to the design pose, i.e. qpos0) stands in for the
    #    real servos and holds the model in its standing pose; tuned by
    #    stepping the model for a simulated 30s and confirming it settles
    #    (rather than drifts or oscillates) within a few seconds at <0.09
    #    rad of its rest pose. `implicitfast` integrates joint damping
    #    stably at this stiffness -- the default `Euler` integrator blows
    #    up (NaN qacc) at far lower gains.
    mjcf_text = mjcf_text.replace(
        "<compiler",
        '<option integrator="implicitfast"/>\n'
        "<default>\n"
        '  <joint stiffness="300" damping="25"/>\n'
        '  <geom contype="2" conaffinity="1"/>\n'
        "</default>\n"
        "<compiler",
        1,
    )
    OUTPUT_MJCF_PATH.write_text(mjcf_text)

    # Verify the saved MJCF is loadable standalone before declaring success.
    mujoco.MjModel.from_xml_path(str(OUTPUT_MJCF_PATH))
    print(f"wrote MJCF -> {OUTPUT_MJCF_PATH.relative_to(ROOT.parent)}")


if __name__ == "__main__":
    main()
