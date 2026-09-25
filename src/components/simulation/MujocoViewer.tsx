"use client";

import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import clsx from "clsx";

import { buildSceneFromModel, getBodyAxisXY, getPosition, getQuaternion, loadHumanoidModel } from "@/lib/mujoco/loadHumanoidScene";

// The MuJoCo WASM engine (mujoco.js + mujoco.wasm) is served as a static
// asset from public/mujoco/ rather than bundled -- it's a precompiled
// Emscripten module meant to be fetched at its own URL, and mujoco.js
// resolves its .wasm relative to that URL by default. See
// scripts/sync-mujoco-assets.sh for how these files get there.
const ENGINE_URL = "/mujoco/engine/mujoco.js";

// Drive/teleop tuning -- kept well inside the placeholder actuators' own
// ctrlrange (see model/scripts/urdf_to_mjcf.py) so a held key never asks for
// more than the actuator can deliver. DRIVE_WHEEL_RADIUS is a real
// measurement (from the wheel mesh's own geometry, see the same script);
// the speeds themselves are demo pacing, not hardware specs.
const DRIVE_SPEED = 0.8; // m/s
const TURN_SPEED = 1.0; // rad/s
const DRIVE_WHEEL_RADIUS = 0.1015; // m
// The telescoping lift column between the wheel base and the torso ("Slider
// 2" in the CAD export -- a real prismatic joint, not cosmetic: raising it
// visibly lifts the chest/head/arms as one rigid unit relative to the wheel
// base). U raises, L lowers, holding at whatever height the key was
// released at (a position target, not a velocity) -- ctrlrange itself
// (read from the model, not hardcoded) already caps it to what the column
// can physically extend.
const LIFT_RATE = 0.15; // m/s while U or L is held
// Waist forward bend ("Revolute 4" -- the torso pitch joint, WAIST_JOINTS in
// model/scripts/urdf_to_mjcf.py). Its own ctrlrange goes further than this
// (to -1.92 rad, confirmed via headless kinematics: negative is the forward
// direction, positive doesn't exist on this joint at all -- 0 is fully
// straight), but capped here at -90deg -- a human-plausible forward bend
// limit, not the joint's own mechanical one. F bends forward, R straightens
// back up, same hold-to-move/release-to-hold pattern as U/L.
const BEND_RATE = 0.5; // rad/s while F or R is held
const BEND_MAX = Math.PI / 2; // rad, forward -- clamps *below* the joint's own -1.92 limit

// Right arm (the same one Pick & Place uses). Sign conventions and which
// axis does what confirmed via headless forward kinematics, not guessed --
// swept each joint alone (others held at 0) and read where the hand ended
// up in base_link's own local frame (x=lateral, y=vertical, z=forward/back,
// same convention as getBodyAxisXY/BASE_FORWARD_AXIS elsewhere in this
// file):
//  - "Revolute 5" (shoulder, forward/back): local z barely moves near 0 but
//    swings hugely negative (forward, this body's own forward-axis
//    convention) as qpos goes *negative*, and only *positive* as far as
//    +45deg (the joint's own mechanical limit) before running out of range
//    -- i.e. this one joint's own range is already asymmetric in exactly
//    the shape asked for (a big forward reach, a small backward one), 0 is
//    the neutral/straight-down reference.
//  - "Revolute 7" (shoulder, lateral): local z is untouched throughout;
//    local x sweeps hugely instead. 0 is the same neutral reference.
const SHOULDER_FWD_RATE = 0.6; // rad/s while T or G is held
const SHOULDER_FWD_MAX = Math.PI / 2; // rad, forward raise (T) -- well inside the joint's own -180deg limit
const SHOULDER_BACK_MAX = Math.PI / 6; // rad, backward (G) -- 30deg, inside the joint's own +45deg limit
const SHOULDER_LAT_RATE = 0.6; // rad/s while Y or H is held
const SHOULDER_LAT_MAX = Math.PI / 2; // rad, lateral raise (Y) -- clamps below the joint's own 180deg range
// Upper-arm roll ("Revolute 9") and elbow ("Revolute 11") -- no raise-angle
// spec given for these the way the shoulder had, so each just uses its own
// full mechanical range (read from the model, not hardcoded) as the cap.
const ARM_ROLL_RATE = 0.6; // rad/s while I or K is held
const ELBOW_RATE = 0.8; // rad/s while O or P is held

// Wrist (right arm) -- two axes, both swept offline the same way the
// shoulder axes were (see that comment): "Revolute 26" sweeps the hand
// through a wide forward/back arc, "Revolute 43" sweeps it through a
// lateral/vertical arc at a constant forward distance. Neither has a
// human-plausible-angle spec from the user the way the shoulder did, so (like
// arm roll/elbow) each just uses its own full mechanical range.
const WRIST_A_RATE = 0.8; // rad/s while J or N is held ("Revolute 26")
const WRIST_B_RATE = 0.8; // rad/s while V or B is held ("Revolute 43")

// Left arm mirrors the right one joint-for-joint (see LEFT_ARM_JOINTS'
// own comment below). Originally driven by holding Shift with the same key
// as the matching right-arm control, but that Shift-chord scheme turned out
// unreliable in practice (reported as "left arm mirroring doesn't work at
// all" -- most likely browsers/OSes not reliably reporting a modifier held
// simultaneously with another key the way a plain keydown is reported) --
// switched to its own dedicated keys (number row + punctuation) instead, so
// each left-arm control is a single ordinary key with no modifier involved.
// Every left-arm joint's positive direction was confirmed, not assumed, to
// be the mirror image of its right-arm counterpart's (swept "Revolute
// 6/8/27/64" the same way as "Revolute 5/7/26/43" -- each one's
// hand-position sweep is the same shape, just with z (forward) or x
// (lateral) trending the *opposite* way for the same sign of qpos) -- so the
// left-arm controls below apply the *negated* rate to reach the mirrored
// target range.
const LEFT_MIRROR_SIGN = -1;

// Manual grip (both hands): holding it closed near the object grabs it with
// a kinematic follow (applyGripPose below), not an equality weld -- this
// engine build's JS bindings crash on any access to a boolean-typed
// mjData/mjModel array, which is exactly what the weld-enable mechanism
// (data.eq_active) is, so a weld-based design isn't usable here at all (see
// applyGripPose's own comment). Finger-contact alone doesn't reliably hold
// anything with this hand either (the thumb chain doesn't oppose the other
// four fingers regardless of joint values, see PICK_FINGER_JOINTS' own
// comment), so without the kinematic follow a "grabbed" object would just
// slide free the moment the arm moved.
const GRIP_RATE = 1.2; // 0..1 (open..closed) per second while held
const GRIP_ENGAGE_AT = 0.8; // grip fraction above which closing near the object grabs it
const GRIP_RELEASE_AT = 0.2; // grip fraction below which it lets go
const GRIP_RADIUS = 0.1; // m -- how close the grip anchor must be to the object to grab it. Was 0.2 ("loose: there's no on-screen distance readout") -- reported live as grabbing objects from noticeably far away, which combined with GRIP_SNUG_DIST's own old absence left a large, odd-looking gap between the hand and whatever it grabbed for the rest of the hold.
// The hold itself is a kinematic follow (applyGripPose) that freezes
// whatever the anchor-to-object offset happened to be *at the moment of
// grab* -- so grabbing from anywhere within GRIP_RADIUS (up to 20cm, before
// the tightening above) could freeze in a correspondingly large, permanent
// gap for the whole hold, not just a one-off inaccuracy. Clamping the
// captured relPos's own magnitude (direction preserved, distance capped)
// guarantees a snug-looking hold regardless of exactly how close the grab
// itself was, on top of GRIP_RADIUS's own tightening above.
const GRIP_SNUG_DIST = 0.05; // m -- max allowed anchor-to-object distance once grabbed
// Which of base_link's own local axes points where the robot actually
// faces -- see getBodyAxisXY's comment. Confirmed empirically (not
// guessed): local X is the line straight through both hands (i.e. side to
// side), local Y is straight up, which only leaves local Z as the
// front-back axis.
const BASE_FORWARD_AXIS = 2 as const;

// Fixed "security camera" presets, one per room corner, each looking back
// toward the middle of the kitchen. Coordinates are three.js (Y-up).
// Recomputed from the imported kitchen's own real, measured boundary via
// the same swizzle loadHumanoidScene.ts's getPosition() uses (three.x=mj.x,
// three.y=mj.z, three.z=-mj.y) -- NOT the old 12x12m procedural room's ±5
// these used to be hardcoded to (that room no longer exists; those
// positions sat well outside this kitchen's actual walls, hence needing to
// zoom in just to see anything).
//
// This uses a *tighter* y-range (MuJoCo y: -2.75..6.0) than the kitchen's
// own full collision-wall boundary (_KITCHEN_WALL_BOUNDS in
// urdf_to_mjcf.py, y up to 7.71) on purpose: the imported asset's real
// extent continues past the kitchen proper into a hallway/staircase (see
// the reference renders, model/kitchen/kitchen-scene-*.webp, where it's
// visible through the opening on the right) -- fine to leave in the
// robot's walkable/collision area, but a "corner of the room" camera framed
// against that full extent puts two of the four cameras staring at the
// staircase instead of the kitchen (confirmed live: corner3 landed inside
// the banister). 6.0 is the tall pantry/cabinet run's own far edge
// (Wood_Mahogany_33_46_100cm's own y-max), i.e. roughly where the kitchen
// itself actually ends.
//
// Inset 1.8m (not just enough to clear the walls) and raised to 3.2m (near
// the 3.7m ceiling): a smaller inset/lower height both put the camera
// clipping into the floor-to-ceiling pantry cabinets that run right along
// these walls (confirmed live -- a 1m inset at 2.8m landed the camera
// pressed right up against one). Update these if the kitchen import/its
// offset in convert_kitchen_obj.py ever changes.
const CORNER_VIEWS: { pos: [number, number, number]; lookAt: [number, number, number] }[] = [
  { pos: [1.93, 3.2, 0.95], lookAt: [0, 0.9, -1.6] },
  { pos: [-1.93, 3.2, 0.95], lookAt: [0, 0.9, -1.6] },
  { pos: [-1.93, 3.2, -4.2], lookAt: [0, 0.9, -1.6] },
  { pos: [1.93, 3.2, -4.2], lookAt: [0, 0.9, -1.6] },
];

type ViewMode = "orbit" | "corner1" | "corner2" | "corner3" | "corner4" | "fpp";
const VIEW_OPTIONS: { value: ViewMode; label: string }[] = [
  { value: "orbit", label: "Center Stage" },
  { value: "corner1", label: "Corner 1" },
  { value: "corner2", label: "Corner 2" },
  { value: "corner3", label: "Corner 3" },
  { value: "corner4", label: "Corner 4" },
  { value: "fpp", label: "First Person" },
];

// The on-screen key guide's own content -- grouped so the panel can render
// section headings, not just one long list. Kept as data rather than
// scattered across each control's own JSX so adding a key here can't
// silently drift out of sync with what the panel actually shows.
const KEY_GUIDE: { section: string; rows: { key: string; action: string }[] }[] = [
  {
    section: "Drive",
    rows: [
      { key: "W / S", action: "Forward / back" },
      { key: "A / D", action: "Turn left / right" },
    ],
  },
  {
    section: "Torso",
    rows: [
      { key: "U / L", action: "Raise / lower height" },
      { key: "F / R", action: "Bend forward / straighten" },
    ],
  },
  {
    section: "Right arm",
    rows: [
      { key: "T / G", action: "Shoulder: raise forward / lower back" },
      { key: "Y / H", action: "Shoulder: raise sideways / lower" },
      { key: "I / K", action: "Upper arm: rotate" },
      { key: "O / P", action: "Elbow: bend / straighten" },
      { key: "J / N", action: "Wrist: bend" },
      { key: "V / B", action: "Wrist: roll" },
      { key: "C / X", action: "Hand: grip / release" },
    ],
  },
  {
    section: "Left arm",
    rows: [
      { key: "1 / 2", action: "Shoulder: raise forward / lower back" },
      { key: "3 / 4", action: "Shoulder: raise sideways / lower" },
      { key: "5 / 6", action: "Upper arm: rotate" },
      { key: "7 / 8", action: "Elbow: bend / straighten" },
      { key: "9 / 0", action: "Wrist: bend" },
      { key: "= / -", action: "Wrist: roll" },
      { key: "[ / ]", action: "Hand: grip / release" },
    ],
  },
];

// FPP head look-around range -- a human neck doesn't spin like a turret,
// so this is clamped rather than a free 360 look.
const FPP_MAX_YAW = (110 * Math.PI) / 180;
const FPP_MIN_PITCH = (-70 * Math.PI) / 180; // look down
const FPP_MAX_PITCH = (60 * Math.PI) / 180; // look up
const FPP_LOOK_SENSITIVITY = 0.006; // radians per pixel of drag
const FPP_FORWARD_CLEARANCE = 0.22; // m -- clears the head geom's own ~0.158m bounding radius, see the render loop's own comment
// Center Stage's own default framing -- reused both at startup and to
// restore the view when switching back from a corner/FPP view (see the
// render loop's isOrbitView handling: OrbitControls.update() re-derives its
// internal orbit state from wherever camera.position *currently* is on
// every call, rather than remembering a separately preserved state, so
// leaving the camera at, say, an FPP position and re-enabling orbit
// control produces a broken close-up view centered on that leftover
// position instead of resuming the original framing).
// Also recomputed against the kitchen's own real boundary (see
// CORNER_VIEWS' own comment) -- was [5.5, 4, 5.5]/[0, 0.6, 0], framing the
// old procedural room's center at the world origin; this room's real
// center sits at three.js z=-2.48, not 0.
const ORBIT_POSITION: [number, number, number] = [4.5, 3.5, 0.0];
const ORBIT_TARGET: [number, number, number] = [0, 0.9, -2.48];

// Wave gesture removed at the user's request (the joint-anchor fix -- see
// recenter_arm_joint_anchors() in model/scripts/urdf_to_mjcf.py -- fixed the
// "detached"-looking swing, but the resulting motion still didn't read as a
// wave) -- pending a proper scripted-scene replacement.

function smoothstep(x: number): number {
  const t = Math.min(Math.max(x, 0), 1);
  return t * t * (3 - 2 * t);
}

function lerpArr(a: number[], b: number[], t: number): number[] {
  return a.map((v, i) => v + (b[i] - v) * t);
}

// wxyz quaternion helpers for the kinematic-follow grip's relpose (see
// GRIP_RATE's own comment), computed live since a grab -- manual or the
// Workflow's own scripted one -- can happen from any pose, not one fixed
// one.
function quatConj(q: [number, number, number, number]): [number, number, number, number] {
  return [q[0], -q[1], -q[2], -q[3]];
}
function quatMul(
  a: [number, number, number, number],
  b: [number, number, number, number]
): [number, number, number, number] {
  const [w1, x1, y1, z1] = a;
  const [w2, x2, y2, z2] = b;
  return [
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  ];
}

// Live relpose for a manual-grip weld: body1 (the object)'s current
// position/orientation expressed in body2 (the grip anchor)'s own frame --
// the exact quantity a MJCF <weld relpose="..."> attribute holds, just
// computed at grab time instead of offline (see GRIP_RATE's own comment).
// mujoco/data types are untyped (any) throughout this file, hence the
// explicit params here rather than threading real types through --
// TypeScript infers `unknown[]` for a plain Array.from(any) in some
// overload-resolution cases, which is what this sidesteps.
// Standard rotation-matrix -> quaternion (wxyz, matching MuJoCo's own
// convention) conversion -- needed because MjData exposes geom_xmat (a 3x3
// per geom) but no geom_xquat. See RIGHT_GRIP_ANCHOR_GEOM's own comment for
// why the grip anchor has to be a geom, not a body, in the first place.
function mat3ToQuat(m: Float32Array | Float64Array, base: number): [number, number, number, number] {
  const m00 = m[base], m01 = m[base + 1], m02 = m[base + 2];
  const m10 = m[base + 3], m11 = m[base + 4], m12 = m[base + 5];
  const m20 = m[base + 6], m21 = m[base + 7], m22 = m[base + 8];
  const trace = m00 + m11 + m22;
  if (trace > 0) {
    const s = 0.5 / Math.sqrt(trace + 1.0);
    return [0.25 / s, (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s];
  } else if (m00 > m11 && m00 > m22) {
    const s = 2.0 * Math.sqrt(1.0 + m00 - m11 - m22);
    return [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s];
  } else if (m11 > m22) {
    const s = 2.0 * Math.sqrt(1.0 + m11 - m00 - m22);
    return [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s];
  } else {
    const s = 2.0 * Math.sqrt(1.0 + m22 - m00 - m11);
    return [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s];
  }
}

// anchorGeom is a GEOM id, not a body id -- see RIGHT_GRIP_ANCHOR_GEOM's own
// comment for why (every fingertip/palm body in this CAD export has its own
// origin sitting 0.7-0.9m from where its mesh actually renders; geom_xpos/
// geom_xmat, unlike body xpos/xmat, already account for a geom's own local
// pos/mat offset within its body, giving the mesh's real rendered pose).
// eslint-disable-next-line @typescript-eslint/no-explicit-any
function computeGripRelpose(data: any, anchorGeom: number, objBody: number) {
  const ax = data.geom_xpos[anchorGeom * 3];
  const ay = data.geom_xpos[anchorGeom * 3 + 1];
  const az = data.geom_xpos[anchorGeom * 3 + 2];
  const dx = data.xpos[objBody * 3] - ax;
  const dy = data.xpos[objBody * 3 + 1] - ay;
  const dz = data.xpos[objBody * 3 + 2] - az;
  const m = anchorGeom * 9;
  // R^T * d: R is row-major, so column i is [R[i], R[3+i], R[6+i]], and
  // (R^T*d)'s row i is that column dotted with d.
  const relPos: [number, number, number] = [
    data.geom_xmat[m] * dx + data.geom_xmat[m + 3] * dy + data.geom_xmat[m + 6] * dz,
    data.geom_xmat[m + 1] * dx + data.geom_xmat[m + 4] * dy + data.geom_xmat[m + 7] * dz,
    data.geom_xmat[m + 2] * dx + data.geom_xmat[m + 5] * dy + data.geom_xmat[m + 8] * dz,
  ];
  const anchorQuat = mat3ToQuat(data.geom_xmat, m);
  const oq = objBody * 4;
  const objQuat: [number, number, number, number] = [data.xquat[oq], data.xquat[oq + 1], data.xquat[oq + 2], data.xquat[oq + 3]];
  const relQuat = quatMul(quatConj(anchorQuat), objQuat);
  return { relPos, relQuat };
}

// Caps a freshly-captured grip relPos's own distance at GRIP_SNUG_DIST,
// preserving direction -- see that constant's own comment for why (grabbing
// from anywhere within GRIP_RADIUS would otherwise freeze in whatever gap
// existed at that exact moment, for the whole hold).
function snugRelPos(relPos: [number, number, number]): [number, number, number] {
  const dist = Math.hypot(relPos[0], relPos[1], relPos[2]);
  if (dist <= GRIP_SNUG_DIST || dist === 0) return relPos;
  const scale = GRIP_SNUG_DIST / dist;
  return [relPos[0] * scale, relPos[1] * scale, relPos[2] * scale];
}

// Scripted "Workflow": drives to the counter's own coffee cup
// (urdf_to_mjcf.py's CUP_OBJECT_POS), grabs it with the same right arm/hand
// the wave gesture uses, then carries it over to the coffee machine and
// stops there, still holding it -- see the Workflow dropdown's own comment
// near the JSX for the option this drives.
//
// The six arm joint targets below were solved two different ways before
// landing here, both offline against this same model:
//  1. Pure kinematic IK (finger_tip_1 at the target position) -- looked
//     right in a static mj_forward snapshot, but several joints (arm roll
//     "Revolute 9", both wrist axes) turned out to have real per-joint
//     torque limits (as low as +-15 N*m) that the *kinematic* solve had no
//     way to know about, so the actual pose under gravity settled somewhere
//     else entirely once simulated for real -- confirmed by holding each
//     candidate target under actual dynamics for several seconds and
//     comparing.
//  2. Re-solved with the wrist axes ("Revolute 26"/"43") pinned to 0 --
//     their own safe (empirically swept) holding range sits right around
//     0 anyway -- and arm roll bounded to its own [-0.2, 0.2] safe zone,
//     then verified by simulating the *actual* dynamics (not just
//     kinematics) for 2500+ steps and checking the settled anchor position
//     matches the intended grasp point. GRASP below is that verified pose;
//     PREGRASP is just GRASP scaled by 0.6 (an intermediate "reaching, not
//     yet at the object" pose -- doesn't need its own precise IK solve
//     since it's a waypoint, not a grasp point).
const PICK_ARM_JOINTS = ["Revolute 5", "Revolute 7", "Revolute 9", "Revolute 11", "Revolute 26", "Revolute 43"];
const PICK_REST_QPOS = [0, 0, 0, 0, 0, 0];
// Solved with a different objective than earlier attempts at this same
// problem: instead of minimizing raw palm-to-cup distance (which kept
// landing poses where the *forearm* ends up closer to the cup than the palm
// does -- reported live as "picks up with its forearm/wrist," and verified
// real: this rig's fingers, in most reaching poses, don't point at what the
// palm is merely close to), this searches for where the palm's own local
// +Z axis -- the direction its fingers actually extend along, measured
// directly off finger_tip_1/2/thumb_knuckle_1's geom positions in the
// palm's local frame -- points roughly *at* the cup, offset out from the
// palm by about the fingers' closed reach (0.10m). That "virtual grasp
// point," not the raw palm position, is what's minimized here. The result:
// forearm_joint_1's own geoms settle ~3x farther from the cup than this
// grasp point does (0.31m vs 0.10m, confirmed by simulating actual
// dynamics for 6000+ steps, not a kinematic snapshot) -- the fingers, not
// the forearm, are what's nearest the cup now, and the two contact points
// that actually develop during the reach are both fingertips touching the
// counter (checked directly). This also uses noticeably less waist bend
// than the previous solve (~54 degrees vs ~75) and more shoulder-forward
// rotation instead -- the user's own suggestion ("T to move arm forward")
// -- while still settling the base within ~5cm of PICK_PARK_PICK under the
// bend's reaction load (PICK_BEND_HOLD_KP). Every joint tracks its
// commanded value within a few thousandths of a radian at that settle.
const PICK_GRASP_QPOS = [-2.495845868805196, 0.4561463351982671, 0.0, -1.6365550289818054, 0.0, -0.012969195368815467];
// Waist bend (Revolute 4, same joint/actuator as the F/R teleop -- see
// BEND_RATE's own comment) that pairs with PICK_GRASP_QPOS above. Held at 0
// through "reach" (the arm alone extends out toward the cup here -- the
// visible "stretching forward" motion), then ramped up only during "lower"
// once the arm is already fully extended, and held through
// "close"/"settle"/"pullIn" before ramping back to upright during "lift" --
// never left bent while walking. -0.934 rad (~54 degrees), comfortably
// inside BEND_MIN (-pi/2) so it's within the same range a teleop user could
// reach by hand.
const PICK_GRASP_BEND = -0.934;
// The arm's pose while actually driving to/from the coffee machine, held
// through "lift"/"driveToCoffee1"/"driveToCoffee2". Elbow bent to ~90
// degrees with the upper arm pulled back close to the body -- the user's
// own ask, "like people bend their arm so coffee won't fall," and it
// doubles as the fix for an earlier crash: an earlier, more outward-reaching
// carry pose put a fingertip wide enough that it swept into the north-run
// counter's own corner during a turn -- confirmed via headless sim, reported
// live as "it crashes into the table." This pose keeps the whole arm's
// swept envelope close to the chassis instead (re-verified collision-free
// along the entire drive, not just the corner that broke before). The cup
// rides along rigidly regardless of which pose the arm is in (see
// applyGripPose), so it's carried at chest height here instead of down at
// the hip.
const PICK_CARRY_QPOS = [0.5, 0.2, 0.0, 1.5708, 0.0, 0.0];
// Reach pose for setting the cup down once parked at PICK_COFFEE_PARK --
// solved the same way as PICK_GRASP_QPOS (grasp-point-vs-forearm objective,
// same method used to verify PICK_GRASP_QPOS: real dynamics, not a
// kinematic snapshot), pairing with PICK_PLACE_BEND below. Settles the
// grasp point within ~11.5cm of PICK_PLACE_TARGET (vs the old pose, which
// used no bend at all and left the palm nowhere near the counter), with the
// fingertips -- not the forearm -- the closest part to the target, same as
// the pickup side.
const PICK_PLACE_QPOS = [-1.858531326660775, 0.39707497009106385, 0.0, -0.7577017369007841, 0.0, 0.04444714196240446];
// Waist bend for placement -- deliberately smaller than PICK_GRASP_BEND
// (~35 degrees vs ~54): the user's own ask ("bend a little bit and place it
// exact on table"), and this counter sits at the same height as the pickup
// one, so less horizontal distance to close here means less bend needed.
// Same ramp shape as PICK_GRASP_BEND: 0 through "lowerPlace" (arm extends
// first), ramps up during "settlePlace", held through "placing"/"release",
// back to 0 during "retractPlace".
const PICK_PLACE_BEND = -0.61;
// Where the cup actually ends up when placed -- on the counter surface
// (z=0.909, the same measured marble-slab height CUP_OBJECT_POS in
// urdf_to_mjcf.py uses, plus the cup's own half-height) just east of the
// coffee machine's own footprint (x -3.48..-3.01, y 3.52..3.81 -- see that
// script's own manifest measurements), not overlapping it. "placing" still
// animates the cup here directly rather than trusting the arm's own joint
// targets to land exactly on it -- PICK_PLACE_QPOS gets close (~11.5cm) but
// not exact -- so this is what guarantees an exact, on-counter landing
// spot; the animated distance is now short enough to read as the hand
// setting the cup down, not a separate slide (see "placing"'s own comment).
const PICK_PLACE_TARGET: [number, number, number] = [-2.95, 3.65, 0.952];
// All 15 right-hand finger joints (4 fingers + thumb, 3 joints each). Each
// one closes toward its own lower jnt_range limit and opens toward its
// upper limit -- confirmed empirically, not assumed: sweeping each joint
// end to end and checking which end brings that finger's tip closer to the
// other fingers' tips gave the same answer (low = closer) for all 15,
// including the thumb -- whose own chain, even fully curled, still lands
// about half a metre from the other four fingertips regardless of joint
// values, i.e. it doesn't actually oppose them. That's why the grasp below
// is a weld, not finger-contact friction: the four fingers that do converge
// approach the object from only one side, with nothing to press it against.
const PICK_FINGER_JOINTS = [
  "Revolute 48", "Revolute 49", "Revolute 50", "Revolute 51", "Revolute 52",
  "Revolute 53", "Revolute 54", "Revolute 55", "Revolute 56", "Revolute 57",
  "Revolute 58", "Revolute 59", "Revolute 60", "Revolute 61", "Revolute 62",
];
// Left hand's own 15 finger joints -- same open/closed convention as the
// right hand (lo=closed, hi=open for all 15, confirmed separately for this
// hand rather than assumed mirrored -- see PICK_FINGER_JOINTS' own comment
// for how that was checked). Manual grip only (Pick & Place doesn't use the
// left hand).
const LEFT_FINGER_JOINTS = [
  "Revolute 69", "Revolute 70", "Revolute 71", "Revolute 72", "Revolute 73",
  "Revolute 74", "Revolute 75", "Revolute 76", "Revolute 77", "Revolute 78",
  "Revolute 79", "Revolute 80", "Revolute 81", "Revolute 82", "Revolute 83",
];
// Where the base must be parked (world x, y, and a fixed heading) for
// PICK_GRASP_QPOS to actually reach the cup -- east of the kitchen island,
// facing west toward it (see getBodyAxisXY's own comment for why yaw=-pi/2
// is "faces -X" on this rig, confirmed empirically the same way). Verified
// clear of the island's own collision box with margin (headless sim: zero
// base-vs-island contacts parked here).
const PICK_PARK_PICK: [number, number] = [1.45, 1.939];
const PICK_PARK_YAW = -Math.PI / 2;
// Route from spawn (0,0) to PICK_PARK_PICK, as two waypoints rather than
// one straight line -- a direct line clips the center table's own
// collision box (_KITCHEN_CENTER_TABLE_BOUNDS in urdf_to_mjcf.py) once the
// base's yaw has turned enough toward PICK_PARK_YAW along the way (checked
// at yaw=-pi/2 specifically: ~15cm deep). Whether that turn has happened
// yet by the time the base is actually near the table depends on timing
// that isn't worth relying on -- these two waypoints instead keep x>=1.7
// (clear of the table's own x<=0.77) until y is past the table's y<=1.85,
// verified collision-free at every yaw the base could plausibly be at
// along the way (0, +-pi/2), not just its final one.
const PICK_WAYPOINT_EAST: [number, number] = [1.7, 0.0];
const PICK_WAYPOINT_EAST_NORTH: [number, number] = [1.7, 2.2];
// Route from the cup's park spot to the coffee machine, as two waypoints
// rather than one straight line -- a direct line between them cuts straight
// through the island's own collision box (checked: crosses it for roughly a
// third of the distance). Route instead goes north along the open corridor
// east of the island/north run (x=1.45 clears both the island's x<=1.0 and
// the north run's x<=0.85 the whole way up), then west through the gap
// between the island and the north run (y=3.0..3.8, clear of every counter
// except the west counter's own shallow run, which only starts at
// x<=-2.85 -- well past this waypoint's x=-2.4) to a stop point just east
// of the coffee machine, still facing -X toward it. y=3.5 sits close to the
// gap's own midpoint (3.4) -- centered, not hugging either edge, since the
// north-run corner in particular has very little margin to spare (see
// PICK_CARRY_QPOS's own comment: even a tucked arm, not just the chassis,
// needs real clearance there). Both legs verified collision-free via
// headless sim (stepped the whole route with the arm actually held at
// PICK_CARRY_QPOS, zero robot-vs-kitchen contacts throughout, cup included).
const PICK_WAYPOINT_NORTH: [number, number] = [1.45, 3.5];
const PICK_COFFEE_PARK: [number, number] = [-2.4, 3.5];

// Autopilot driving, used only by this scripted sequence (never WASD
// teleop): plain world-frame position P-control with an acceleration ramp
// on virtual_base_x/y. Unlike the base's *forward* drive (which projects
// onto the body's current heading, see getBodyAxisXY's own comment), these
// two joints are literal world-X/world-Y translations independent of
// heading -- confirmed via headless sim -- so driving to a world point needs
// no heading math at all, only distance. Yaw is held at PICK_PARK_YAW by a
// separate P-loop, active through every phase below (not just while
// driving): left alone, virtual_base_yaw slowly drifts under any small
// disturbance (confirmed via headless sim -- it is not spring-loaded back
// to anything on its own), which would throw off every joint-space arm
// target below, all of which assume the base is exactly facing
// PICK_PARK_YAW.
const PICK_DRIVE_KP = 1.2;
// Faster than WASD's own DRIVE_SPEED -- purely demo pacing (a scripted
// sequence looks better moving briskly than crawling across a 12x12m
// room), not a workaround. See GRASP_WELD_ANCHOR_BODY in
// model/scripts/urdf_to_mjcf.py for why the carry itself is solid at any
// reasonable speed here (an earlier weld anchor choice made this whole
// sequence only marginally stable regardless of speed -- fixed by
// anchoring the weld to the hand instead, not by driving faster).
const PICK_DRIVE_MAX_SPEED = 1.4; // m/s
const PICK_DRIVE_MAX_ACCEL = 1.2; // m/s^2
const PICK_DRIVE_ARRIVE_DIST = 0.05; // m
// Base XY position hold used only while bent forward (see PICK_GRASP_BEND's
// own comment) -- stiffer than PICK_DRIVE_KP above since it's resisting a
// sustained reaction load from the bend, not tracking a moving waypoint.
// Without this the base drifts off PICK_PARK_PICK/PICK_COFFEE_PARK under
// that load, which throws off every *_QPOS-relative reach math -- confirmed
// via headless dynamics that PICK_DRIVE_KP's own 1.2 isn't stiff enough to
// hold this against the bend (drifted ~0.75m over 6000 steps rather than
// settling); 8.0 holds within a few cm.
const PICK_BEND_HOLD_KP = 8.0;
const PICK_DRIVE_TIMEOUT_S = 20; // s -- see the render loop's own comment on why a drive phase gets aborted, not left to spin forever, past this
const PICK_YAW_KP = 3.0;
const PICK_YAW_MAX_RATE = 1.5; // rad/s

// Phase durations, in seconds of simulated time -- ticked once per physics
// step inside the same catch-up loop as the wave gesture, for the same
// reason (see waveElapsedS's own comment: immune to render-frame hitches by
// construction, since nothing here reads wall-clock time). REACH/LOWER/
// SETTLE are all longer than a bare "looks fine in a screenshot" pass would
// use -- these specific joints (see PICK_GRASP_QPOS's own comment on their
// torque limits) take real seconds to actually settle into a commanded pose
// under gravity, confirmed by holding one under actual dynamics and
// watching the position error decay; cutting these too short captured the
// grip relPose against a hand that was still mid-swing, which read live as
// "it picks the cup up from way off from where it actually is." These
// values are the shortest that still verified accurate in that same
// headless check (a first pass roughly double these also worked, but read
// live as "the workflow is stuck," since W/A/S/D and every other idle-gated
// control are correctly locked out for its entire ~18s run -- see
// PICK_DRIVE_TIMEOUT_S's own comment for the separate, real WASD bug this
// was mixed up with).
const PICK_REACH_S = 1.5; // rest -> grasp arm pose, bend still at 0 (the arm-stretch-forward motion)
const PICK_LOWER_S = 1.5; // arm holds at grasp pose, bend ramps 0 -> PICK_GRASP_BEND (the final lean-in to touch)
const PICK_CLOSE_S = 0.5; // fingers open -> closed
const PICK_SETTLE_S = 1.2; // lets the arm actually finish settling before the grip is captured
// Shortened from 0.5s now that PICK_GRASP_QPOS/PICK_GRASP_BEND land the palm
// within a few cm of CUP_OBJECT_POS -- see "pullIn"'s own comment -- instead
// of the ~0.3-0.6m gap the old no-bend pose left, which needed a slower,
// more visible slide to not look like an outright teleport.
const PICK_PULL_IN_S = 0.2;
const PICK_LIFT_S = 1.2; // grasp -> carry pose, bend ramps back to 0, now holding the object
const PICK_LOWER_PLACE_S = 1.5; // carry -> place arm pose, bend still at 0 (arm-stretch-forward, same as pickup)
const PICK_SETTLE_PLACE_S = 1.0; // arm holds at place pose, bend ramps 0 -> PICK_PLACE_BEND
// Shortened from 0.6s -- PICK_PLACE_QPOS/PICK_PLACE_BEND now land the grasp
// point within ~11.5cm of PICK_PLACE_TARGET (see PICK_PLACE_QPOS's own
// comment) instead of the old no-bend pose's much larger gap.
const PICK_PLACING_S = 0.3;
const PICK_RELEASE_S = 0.7; // fingers closed -> open
const PICK_RETRACT_PLACE_S = 1.2; // place -> carry, arm withdrawing after letting go

type PickPhase =
  | "idle"
  | "driveToPick1"
  | "driveToPick2"
  | "driveToPick3"
  | "reach"
  | "lower"
  | "close"
  | "settle"
  | "pullIn"
  | "lift"
  | "driveToCoffee1"
  | "driveToCoffee2"
  | "lowerPlace"
  | "settlePlace"
  | "placing"
  | "release"
  | "retractPlace";

interface MujocoViewerProps {
  /** Enables orbit camera controls, WASD driving, the wave gesture, and the pause/reset/wave overlay. */
  interactive?: boolean;
  className?: string;
}

type Status = "loading" | "ready" | "error";

export function MujocoViewer({ interactive = false, className }: MujocoViewerProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [status, setStatus] = useState<Status>("loading");
  const [paused, setPaused] = useState(false);
  const [view, setView] = useState<ViewMode>("orbit");
  const [showGuide, setShowGuide] = useState(true);
  const [workflowChoice, setWorkflowChoice] = useState("");
  const pausedRef = useRef(paused);
  const viewRef = useRef(view);
  const resetRef = useRef<() => void>(() => {});
  const pickRef = useRef<() => void>(() => {});

  useEffect(() => {
    pausedRef.current = paused;
  }, [paused]);

  useEffect(() => {
    viewRef.current = view;
  }, [view]);

  useEffect(() => {
    let disposed = false;
    let renderer: THREE.WebGLRenderer | null = null;
    let controls: OrbitControls | null = null;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    let model: any = null;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    let data: any = null;
    let resizeObserver: ResizeObserver | null = null;
    let removeKeyListeners: (() => void) | null = null;

    async function init() {
      const container = containerRef.current;
      if (!container) return;

      const { default: loadMujoco } = await import(/* webpackIgnore: true */ ENGINE_URL);
      const mujoco = await loadMujoco();
      if (disposed) return;

      model = await loadHumanoidModel(mujoco);
      data = new mujoco.MjData(model);
      mujoco.mj_forward(model, data);
      if (disposed) return;

      const { root, bodies } = buildSceneFromModel(mujoco, model);

      const scene = new THREE.Scene();
      // Bright kitchen theme (see model/scripts/urdf_to_mjcf.py's _build_room()
      // comment) -- this used to be a near-black 0x08090b matching the old
      // dark lab room; the rest of the dashboard chrome around this canvas
      // stays on its own dark theme (see DashboardChrome.tsx) unchanged, this
      // color is scoped to the 3D scene alone.
      scene.background = new THREE.Color(0xf3efe4);
      scene.add(root);
      // Warm key light + soft shadows, matching the reference renders'
      // (model/kitchen/kitchen-scene-*.webp) warm daylight look -- shadows
      // were previously disabled entirely because a single angled key light
      // in the old 12x12m room cast shadows from the walls that blacked out
      // half of it. That's no longer the constraint now that the room is
      // this kitchen's own real (much smaller, real-scale) footprint, so
      // re-enabling them (with a properly sized shadow-camera frustum, see
      // topLight.shadow.camera below) actually grounds furniture with real
      // contact shadows the way the reference renders have, instead of the
      // flat, shadowless look this had before. Ambient/hemisphere still
      // carry most of the fill so shadows read as soft, not harsh.
      scene.add(new THREE.AmbientLight(0xfff4e6, 0.65));
      scene.add(new THREE.HemisphereLight(0xf7f0df, 0xcfc9ba, 0.6));

      const topLight = new THREE.DirectionalLight(0xfff1d6, 1.5);
      topLight.position.set(1, 8, 3);
      topLight.target.position.set(0, 0.6, -2.48);
      topLight.castShadow = true;
      topLight.shadow.mapSize.set(2048, 2048);
      // Orthographic shadow-camera frustum sized to the kitchen's own real
      // footprint (see CORNER_VIEWS' own comment for how that boundary was
      // measured) plus a margin -- the default frustum is far too tight for
      // a room this size and would clip most of it out of shadow entirely.
      topLight.shadow.camera.left = -5;
      topLight.shadow.camera.right = 5;
      topLight.shadow.camera.top = 6;
      topLight.shadow.camera.bottom = -6;
      topLight.shadow.camera.near = 0.5;
      topLight.shadow.camera.far = 20;
      topLight.shadow.bias = -0.0015;
      // Softer penumbra + a touch less key/more ambient -- counter
      // overhangs were casting a hard-edged, high-contrast shadow onto the
      // floor whose lit/shadowed boundary reads as a distinct pale
      // triangular shape rather than a shadow (the user's own report:
      // "white space" on the floor between the counters, not a shadow).
      topLight.shadow.radius = 6;
      topLight.shadow.blurSamples = 16;
      scene.add(topLight, topLight.target);

      const fillLight = new THREE.DirectionalLight(0xcfe0ff, 0.4);
      fillLight.position.set(-5, 6, -5);
      fillLight.target.position.set(0, 0.6, -2.48);
      fillLight.castShadow = false;
      scene.add(fillLight, fillLight.target);

      // Framed to show the whole kitchen (see ORBIT_POSITION/TARGET's own
      // comment on how these are derived from the room's real boundary),
      // not a close-up on the robot alone. Orbit controls (when
      // interactive) still zoom/pan freely.
      const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
      camera.position.set(...ORBIT_POSITION);
      camera.lookAt(...ORBIT_TARGET);

      renderer = new THREE.WebGLRenderer({ antialias: true });
      // Soft (PCF-filtered) shadows -- see the lighting comment above.
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      renderer.toneMapping = THREE.ACESFilmicToneMapping;
      renderer.toneMappingExposure = 1.05;
      container.appendChild(renderer.domElement);

      const resize = () => {
        if (!renderer) return;
        const w = container.clientWidth || 1;
        const h = container.clientHeight || 1;
        renderer.setSize(w, h);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
      };
      resize();
      resizeObserver = new ResizeObserver(resize);
      resizeObserver.observe(container);

      if (interactive) {
        controls = new OrbitControls(camera, renderer.domElement);
        controls.target.set(...ORBIT_TARGET);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        controls.update();
      }

      resetRef.current = () => {
        mujoco.mj_resetData(model, data);
        mujoco.mj_forward(model, data);
        // mj_resetData already puts the weld back to inactive and the
        // pickup object back at its qpos0 spawn point on bench_n1 -- this
        // just resets the *sequence's own* bookkeeping to match.
        pickPhase = "idle";
        pickPhaseElapsedS = 0;
        pickDriveVX = 0;
        pickDriveVY = 0;
        liftHeight = LIFT_MIN;
        bendAngle = 0;
        shoulderFwdAngle = 0;
        shoulderLatAngle = 0;
        armRollAngle = 0;
        elbowAngle = 0;
        wristAAngle = 0;
        wristBAngle = 0;
        shoulderFwdAngleL = 0;
        shoulderLatAngleL = 0;
        armRollAngleL = 0;
        elbowAngleL = 0;
        wristAAngleL = 0;
        wristBAngleL = 0;
        rightGripFraction = 0;
        leftGripFraction = 0;
        rightGripEngaged = false;
        leftGripEngaged = false;
        rightHeldBody = -1;
        leftHeldBody = -1;
        openBothHands();
      };

      // --- Teleop actuator lookups -------------------------------------
      // Names match the placeholder actuators emitted by
      // model/scripts/urdf_to_mjcf.py's _build_actuators(); mj_name2id
      // returns -1 (harmlessly skipped below) if the loaded scene is an
      // older build without one of these.
      const OBJ_ACTUATOR = mujoco.mjtObj.mjOBJ_ACTUATOR.value;
      const OBJ_BODY = mujoco.mjtObj.mjOBJ_BODY.value;
      const actId = (name: string): number => mujoco.mj_name2id(model, OBJ_ACTUATOR, name);
      const ACT_VX = actId("act_base_vx");
      const ACT_VY = actId("act_base_vy");
      const ACT_YAW = actId("act_base_yaw");
      const ACT_WHEEL_L = actId("act_Revolute 32");
      const ACT_WHEEL_R = actId("act_Revolute 33");
      const ACT_LIFT = actId("act_Slider 2");
      const ACT_BEND = actId("act_Revolute 4");
      const ACT_SHOULDER_FWD = actId("act_Revolute 5");
      const ACT_SHOULDER_LAT = actId("act_Revolute 7");
      const ACT_ARM_ROLL = actId("act_Revolute 9");
      const ACT_ELBOW = actId("act_Revolute 11");
      const ACT_WRIST_A = actId("act_Revolute 26");
      const ACT_WRIST_B = actId("act_Revolute 43");
      // Left arm -- same six joint roles, mirrored bodies (see
      // LEFT_MIRROR_SIGN's own comment).
      const ACT_SHOULDER_FWD_L = actId("act_Revolute 6");
      const ACT_SHOULDER_LAT_L = actId("act_Revolute 8");
      const ACT_ARM_ROLL_L = actId("act_Revolute 10");
      const ACT_ELBOW_L = actId("act_Revolute 12");
      const ACT_WRIST_A_L = actId("act_Revolute 27");
      const ACT_WRIST_B_L = actId("act_Revolute 64");
      const BASE_BODY = mujoco.mj_name2id(model, OBJ_BODY, "base_link");
      const canDrive = ACT_VX >= 0 && ACT_VY >= 0 && ACT_YAW >= 0 && BASE_BODY >= 0;
      // face_cover_3_1's own body origin sits nearly 1.3m from where its mesh
      // actually renders (see FPP_EYE_HEIGHT's old comment -- a large local-
      // origin-vs-geometry offset baked into this CAD export), which used to
      // be worked around by guessing a fixed height/forward offset above
      // base_link instead. That guess put the FPP camera off to one side
      // (reported live: "FPP is from the right side, not the robot's eye") --
      // geom_xpos/geom_xmat (unlike body xpos/xmat) already account for a
      // geom's own local pos/quat offset within its body, so they give the
      // mesh's *actual* rendered pose directly, sidestepping the bad body
      // origin entirely.
      const HEAD_BODY = mujoco.mj_name2id(model, OBJ_BODY, "face_cover_3_1");
      const HEAD_GEOM = HEAD_BODY >= 0 ? model.body_geomadr[HEAD_BODY] : -1;
      const canFpp = canDrive && HEAD_GEOM >= 0;
      const canLift = ACT_LIFT >= 0;
      const LIFT_MIN = canLift ? model.actuator_ctrlrange[ACT_LIFT * 2] : 0;
      const LIFT_MAX = canLift ? model.actuator_ctrlrange[ACT_LIFT * 2 + 1] : 0;
      const canBend = ACT_BEND >= 0;
      // BEND_MAX/SHOULDER_*_MAX clamp *below* their joints' own (more
      // permissive) ranges -- see each constant's own comment -- so these
      // aren't simply each actuator's own ctrlrange the way LIFT_MIN/MAX is.
      const BEND_MIN = -BEND_MAX;
      const canShoulderFwd = ACT_SHOULDER_FWD >= 0;
      const canShoulderLat = ACT_SHOULDER_LAT >= 0;
      const canArmRoll = ACT_ARM_ROLL >= 0;
      const ARM_ROLL_MIN = canArmRoll ? model.actuator_ctrlrange[ACT_ARM_ROLL * 2] : 0;
      const ARM_ROLL_MAX = canArmRoll ? model.actuator_ctrlrange[ACT_ARM_ROLL * 2 + 1] : 0;
      const canElbow = ACT_ELBOW >= 0;
      const ELBOW_MAX = canElbow ? model.actuator_ctrlrange[ACT_ELBOW * 2 + 1] : 0;
      const canWristA = ACT_WRIST_A >= 0;
      const WRIST_A_MIN = canWristA ? model.actuator_ctrlrange[ACT_WRIST_A * 2] : 0;
      const WRIST_A_MAX = canWristA ? model.actuator_ctrlrange[ACT_WRIST_A * 2 + 1] : 0;
      const canWristB = ACT_WRIST_B >= 0;
      const WRIST_B_MIN = canWristB ? model.actuator_ctrlrange[ACT_WRIST_B * 2] : 0;
      const WRIST_B_MAX = canWristB ? model.actuator_ctrlrange[ACT_WRIST_B * 2 + 1] : 0;

      const canShoulderFwdL = ACT_SHOULDER_FWD_L >= 0;
      const canShoulderLatL = ACT_SHOULDER_LAT_L >= 0;
      const canArmRollL = ACT_ARM_ROLL_L >= 0;
      const ARM_ROLL_MIN_L = canArmRollL ? model.actuator_ctrlrange[ACT_ARM_ROLL_L * 2] : 0;
      const ARM_ROLL_MAX_L = canArmRollL ? model.actuator_ctrlrange[ACT_ARM_ROLL_L * 2 + 1] : 0;
      const canElbowL = ACT_ELBOW_L >= 0;
      const ELBOW_MIN_L = canElbowL ? model.actuator_ctrlrange[ACT_ELBOW_L * 2] : 0;
      const canWristAL = ACT_WRIST_A_L >= 0;
      const WRIST_A_MIN_L = canWristAL ? model.actuator_ctrlrange[ACT_WRIST_A_L * 2] : 0;
      const WRIST_A_MAX_L = canWristAL ? model.actuator_ctrlrange[ACT_WRIST_A_L * 2 + 1] : 0;
      const canWristBL = ACT_WRIST_B_L >= 0;
      const WRIST_B_MIN_L = canWristBL ? model.actuator_ctrlrange[ACT_WRIST_B_L * 2] : 0;
      const WRIST_B_MAX_L = canWristBL ? model.actuator_ctrlrange[ACT_WRIST_B_L * 2 + 1] : 0;

      // --- Pick & Place lookups ---------------------------------------
      const OBJ_JOINT = mujoco.mjtObj.mjOBJ_JOINT.value;
      const jointId = (name: string): number => mujoco.mj_name2id(model, OBJ_JOINT, name);
      // NOTE: these joint-name arrays are also reused for jointId() lookups
      // below (bare "Revolute N"), so actId() itself takes the actuator's
      // own full name -- the "act_" prefix has to be added here, at the
      // call site, rather than baked into the shared array. Its absence was
      // a real bug: mj_name2id("Revolute 48", ...) against actuators never
      // matches anything (confirmed directly against the compiled model),
      // so every one of these resolved to -1, silently disabling Pick &
      // Place's arm/finger control and the entire manual grip feature.
      const ACT_PICK_ARM = PICK_ARM_JOINTS.map((name) => actId(`act_${name}`));
      const ACT_PICK_FINGERS = PICK_FINGER_JOINTS.map((name) => actId(`act_${name}`));
      // Each finger joint's own [lo, hi] range, read from the model rather
      // than hardcoded -- lo is "closed", hi is "open" (see
      // PICK_FINGER_JOINTS' own comment).
      const PICK_FINGER_CLOSED = PICK_FINGER_JOINTS.map((name) => {
        const jid = jointId(name);
        return jid >= 0 ? model.jnt_range[jid * 2] : 0;
      });
      const PICK_FINGER_OPEN = PICK_FINGER_JOINTS.map((name) => {
        const jid = jointId(name);
        return jid >= 0 ? model.jnt_range[jid * 2 + 1] : 0;
      });
      // qpos/dof addresses for the Workflow's per-substep finger pin (see
      // that block's own comment) -- resolved once here by name instead of
      // calling jointId() (a string lookup) 15 times every physics substep,
      // which for a multi-second hold is a lot of repeated name lookups for
      // an answer that never changes.
      const PICK_FINGER_QPOSADR = PICK_FINGER_JOINTS.map((name) => {
        const jid = jointId(name);
        return jid >= 0 ? model.jnt_qposadr[jid] : -1;
      });
      const PICK_FINGER_DOFADR = PICK_FINGER_JOINTS.map((name) => {
        const jid = jointId(name);
        return jid >= 0 ? model.jnt_dofadr[jid] : -1;
      });
      const YAW_JOINT = jointId("virtual_base_yaw");
      const YAW_QPOS_ADR = YAW_JOINT >= 0 ? model.jnt_qposadr[YAW_JOINT] : -1;
      const PICKUP_OBJECT_JOINT = jointId("pickup_object_free");
      const PICKUP_OBJECT_DOF = PICKUP_OBJECT_JOINT >= 0 ? model.jnt_dofadr[PICKUP_OBJECT_JOINT] : -1;

      // --- Manual grip lookups -----------------------------------------
      // Right hand reuses PICK_FINGER_JOINTS/ACT_PICK_FINGERS/
      // PICK_FINGER_CLOSED/OPEN above -- same 15 joints Pick & Place
      // drives, just under direct key control instead of a scripted phase
      // (both gated the same way, by pickPhase === "idle", so they never
      // fight over them).
      const ACT_LEFT_FINGERS = LEFT_FINGER_JOINTS.map((name) => actId(`act_${name}`));
      const LEFT_FINGER_CLOSED = LEFT_FINGER_JOINTS.map((name) => {
        const jid = jointId(name);
        return jid >= 0 ? model.jnt_range[jid * 2] : 0;
      });
      const LEFT_FINGER_OPEN = LEFT_FINGER_JOINTS.map((name) => {
        const jid = jointId(name);
        return jid >= 0 ? model.jnt_range[jid * 2 + 1] : 0;
      });
      const PICKUP_OBJECT_BODY = mujoco.mj_name2id(model, OBJ_BODY, "pickup_object");
      // The counter's own coffee cup (see CUP_OBJECT_POS in
      // model/scripts/urdf_to_mjcf.py) -- a second, separate free body, not
      // a reskin of pickup_object, so both can exist (and in principle be
      // held one per hand) at once.
      const CUP_OBJECT_JOINT = jointId("cup_object_free");
      const CUP_OBJECT_DOF = CUP_OBJECT_JOINT >= 0 ? model.jnt_dofadr[CUP_OBJECT_JOINT] : -1;
      const CUP_OBJECT_QPOS_ADR = CUP_OBJECT_JOINT >= 0 ? model.jnt_qposadr[CUP_OBJECT_JOINT] : -1;
      const CUP_OBJECT_BODY = mujoco.mj_name2id(model, OBJ_BODY, "cup_object");
      // Manual grip was originally pinned to pickup_object specifically;
      // generalized into a list so grabbing checks whichever free body is
      // actually closest. Holding is a kinematic follow (see
      // applyGripPose below), not an equality constraint or finger
      // contact -- see model/scripts/urdf_to_mjcf.py's own comment, right
      // above where the manual-grip welds used to be declared, for why.
      // Adding a third object (the coffee machine, per the user's own
      // "let's start with the cup" ask) is one more entry here.
      const GRABBABLE_OBJECTS = [
        { body: PICKUP_OBJECT_BODY, dof: PICKUP_OBJECT_DOF, qposAdr: PICKUP_OBJECT_JOINT >= 0 ? model.jnt_qposadr[PICKUP_OBJECT_JOINT] : -1 },
        { body: CUP_OBJECT_BODY, dof: CUP_OBJECT_DOF, qposAdr: CUP_OBJECT_QPOS_ADR },
      ].filter((o) => o.body >= 0 && o.dof >= 0 && o.qposAdr >= 0);
      // finger_tip_1/finger_tip_3's own body ORIGIN sits 0.7-0.9m from where
      // their mesh actually renders -- confirmed not just for these two but
      // for every single fingertip body in this hand rig (a systemic CAD-
      // export quirk, the same class of bug FPP's own eye position had).
      // Using body xpos/xmat as the grip anchor (the original design) meant
      // every "snug" distance check was measured from an invisible point
      // 0.7m+ from the hand, not from the hand itself -- reported live as
      // "the gap between hand and cup is too much." geom_xpos/geom_xmat
      // (indexed by GEOM id here, not body id) give the real mesh pose
      // instead, same fix as HEAD_GEOM uses for the camera.
      //
      // The anchor itself is the palm (palm_right_1/palm_left_1), not a
      // fingertip -- a held object snugged to a single fingertip still
      // reads as "balanced on one finger," not "held in the hand," reported
      // live as "the cup seems to be on the fingertips, it must be on the
      // palm." Fingers close around wherever the object ends up regardless
      // of which point on the hand it's snugged to (see fingerHold's own
      // comment), so anchoring to the broader, more central palm mesh
      // instead reads as the fingers actually wrapping around it.
      const RIGHT_GRIP_ANCHOR_GEOM = mujoco.mj_name2id(model, OBJ_BODY, "palm_right_1") >= 0
        ? model.body_geomadr[mujoco.mj_name2id(model, OBJ_BODY, "palm_right_1")]
        : -1;
      const LEFT_GRIP_ANCHOR_GEOM = mujoco.mj_name2id(model, OBJ_BODY, "palm_left_1") >= 0
        ? model.body_geomadr[mujoco.mj_name2id(model, OBJ_BODY, "palm_left_1")]
        : -1;
      const canGripRight =
        ACT_PICK_FINGERS.every((i) => i >= 0) && GRABBABLE_OBJECTS.length > 0 && RIGHT_GRIP_ANCHOR_GEOM >= 0;
      const canGripLeft =
        ACT_LEFT_FINGERS.every((i) => i >= 0) && GRABBABLE_OBJECTS.length > 0 && LEFT_GRIP_ANCHOR_GEOM >= 0;
      // Nearest grabbable object to `anchorBody`, excluding whatever the
      // *other* hand is already holding (so both hands can each hold their
      // own object, but not fight over one) -- null if nothing is within
      // GRIP_RADIUS.
      const findGrabTarget = (anchorGeom: number, excludeBody: number) => {
        let best: { body: number; dof: number; qposAdr: number } | null = null;
        let bestDist = GRIP_RADIUS;
        for (const obj of GRABBABLE_OBJECTS) {
          if (obj.body === excludeBody) continue;
          const dist = Math.hypot(
            data.geom_xpos[anchorGeom * 3] - data.xpos[obj.body * 3],
            data.geom_xpos[anchorGeom * 3 + 1] - data.xpos[obj.body * 3 + 1],
            data.geom_xpos[anchorGeom * 3 + 2] - data.xpos[obj.body * 3 + 2]
          );
          if (dist < bestDist) {
            best = obj;
            bestDist = dist;
          }
        }
        return best;
      };
      // Kinematic follow: pose the held object at (anchor's current world
      // pose) composed with the fixed relPos/relQuat captured at grab time
      // -- see model/scripts/urdf_to_mjcf.py's own comment on why this
      // replaced an equality-weld design. Inverse of computeGripRelpose's
      // own math (relPos/relQuat are body1 expressed in body2/anchor's
      // frame, so worldPos = anchorPos + anchorRot*relPos and worldQuat =
      // anchorQuat*relQuat).
      const applyGripPose = (
        anchorGeom: number,
        relPos: [number, number, number],
        relQuat: [number, number, number, number],
        qposAdr: number,
        dof: number
      ) => {
        const ax = data.geom_xpos[anchorGeom * 3];
        const ay = data.geom_xpos[anchorGeom * 3 + 1];
        const az = data.geom_xpos[anchorGeom * 3 + 2];
        const m = anchorGeom * 9;
        data.qpos[qposAdr] =
          ax + data.geom_xmat[m] * relPos[0] + data.geom_xmat[m + 1] * relPos[1] + data.geom_xmat[m + 2] * relPos[2];
        data.qpos[qposAdr + 1] =
          ay + data.geom_xmat[m + 3] * relPos[0] + data.geom_xmat[m + 4] * relPos[1] + data.geom_xmat[m + 5] * relPos[2];
        data.qpos[qposAdr + 2] =
          az + data.geom_xmat[m + 6] * relPos[0] + data.geom_xmat[m + 7] * relPos[1] + data.geom_xmat[m + 8] * relPos[2];
        const anchorQuat = mat3ToQuat(data.geom_xmat, m);
        const worldQuat = quatMul(anchorQuat, relQuat);
        data.qpos[qposAdr + 3] = worldQuat[0];
        data.qpos[qposAdr + 4] = worldQuat[1];
        data.qpos[qposAdr + 5] = worldQuat[2];
        data.qpos[qposAdr + 6] = worldQuat[3];
        for (let k = 0; k < 6; k++) data.qvel[dof + k] = 0;
      };
      const canPick =
        canDrive &&
        YAW_QPOS_ADR >= 0 &&
        CUP_OBJECT_DOF >= 0 &&
        CUP_OBJECT_QPOS_ADR >= 0 &&
        CUP_OBJECT_BODY >= 0 &&
        RIGHT_GRIP_ANCHOR_GEOM >= 0 &&
        ACT_PICK_ARM.every((i) => i >= 0) &&
        ACT_PICK_FINGERS.every((i) => i >= 0);
      if (!canPick) {
        // The Workflow dropdown always renders (see the wave button's own
        // precedent) and silently no-ops if unsupported -- fine for an old
        // build missing the feature entirely, but indistinguishable from
        // "the workflow does nothing" if the loaded scene *should* support
        // it and one lookup below just failed (e.g. a stale cached
        // /mujoco/scene/humanoid.xml missing the cup a newer build added).
        // Logged once at load time so that's not a silent dead end to debug
        // from the user's report alone.
        console.warn("[MujocoViewer] Workflow unavailable in this scene:", {
          canDrive,
          YAW_QPOS_ADR,
          CUP_OBJECT_DOF,
          CUP_OBJECT_QPOS_ADR,
          CUP_OBJECT_BODY,
          RIGHT_GRIP_ANCHOR_GEOM,
          missingArmActuators: PICK_ARM_JOINTS.filter((_, i) => ACT_PICK_ARM[i] < 0),
          missingFingerActuators: PICK_FINGER_JOINTS.filter((_, i) => ACT_PICK_FINGERS[i] < 0),
        });
      }

      // Both hands' finger actuators default to ctrl=0 on load, same as
      // every other actuator -- but 0 happens to equal *closed* for 14 of
      // each hand's 15 finger joints (their own jnt_range is [0, hi], see
      // PICK_FINGER_JOINTS' own comment: lo=closed, hi=open), while the
      // thumb's own root joint (Revolute 48 / Revolute 69) has an
      // asymmetric range that doesn't start at 0. qpos0 is 0 for all of
      // them too (no <keyframe> sets otherwise), so on every load and
      // reset the four fingers snap straight to fully closed while the
      // thumb sits at a mid-range position instead of joining them --
      // looking exactly like a fist that won't quite finish closing,
      // reported as "it automatically tries to close its hands but
      // something is stopping it." Fixed by explicitly starting both hands
      // fully open -- qpos *and* ctrl together, so there's no first-frame
      // snap in either direction -- right after load and after every
      // reset (see resetRef.current above).
      const openHandJoints = (joints: string[], acts: number[], openTarget: number[]) => {
        for (let i = 0; i < joints.length; i++) {
          const jid = jointId(joints[i]);
          if (jid >= 0) data.qpos[model.jnt_qposadr[jid]] = openTarget[i];
          if (acts[i] >= 0) data.ctrl[acts[i]] = openTarget[i];
        }
      };
      const openBothHands = () => {
        openHandJoints(PICK_FINGER_JOINTS, ACT_PICK_FINGERS, PICK_FINGER_OPEN);
        openHandJoints(LEFT_FINGER_JOINTS, ACT_LEFT_FINGERS, LEFT_FINGER_OPEN);
        mujoco.mj_forward(model, data);
      };
      openBothHands();

      const pressedKeys = new Set<string>();

      // Pick & Place state -- see the PICK_* constants above for the phase
      // list/durations. Ticked once per physics step inside the substep
      // catch-up loop below (never from wall-clock/performance.now()) so a
      // slow/backgrounded render frame can't desync it -- a real hitch would
      // otherwise make a fast-moving target jump discontinuously once the
      // catch-up loop resumes and fast-forwards several steps at once.
      // pickDriveVX/VY hold the autopilot's own current commanded velocity
      // across steps, for its acceleration ramp.
      let pickPhase: PickPhase = "idle";
      let pickPhaseElapsedS = 0;
      let pickDriveVX = 0;
      let pickDriveVY = 0;

      // Lift column height target (U/L teleop, see LIFT_RATE's own
      // comment). Starts at LIFT_MIN -- the height every PICK_*_QPOS arm
      // pose was solved against -- so a fresh load reaches exactly where
      // the offline IK assumed; Pick & Place forces it back to LIFT_MIN
      // for the same reason if the user had raised it before starting.
      let liftHeight = LIFT_MIN;
      // Waist bend target (F/R teleop, see BEND_RATE's own comment). Starts
      // at 0 (straight) for the same reason liftHeight starts at LIFT_MIN --
      // every PICK_*_QPOS arm pose was solved with the waist straight, so
      // Pick & Place forces this back to 0 too.
      let bendAngle = 0;
      // Right-arm joint teleop targets (T/G, Y/H, I/K, O/P -- see each
      // *_RATE constant's own comment). All start at 0 (each joint's own
      // neutral/straight reference). Unlike liftHeight/bendAngle, these
      // aren't force-held to a fixed value throughout a Pick & Place run
      // (that sequence needs to freely move these same four joints through
      // its own reach/grasp poses) -- instead they're simply not written to
      // ctrl at all while pickPhase !== "idle" (same idle-gating below as
      // every other teleop control), and reset to 0 only once retract
      // finishes and the arm has actually physically returned to
      // PICK_REST_QPOS (all zeros) -- see that transition below.
      let shoulderFwdAngle = 0;
      let shoulderLatAngle = 0;
      let armRollAngle = 0;
      let elbowAngle = 0;
      let wristAAngle = 0;
      let wristBAngle = 0;
      // Left arm -- same six, own dedicated keys (see LEFT_MIRROR_SIGN's
      // own comment).
      let shoulderFwdAngleL = 0;
      let shoulderLatAngleL = 0;
      let armRollAngleL = 0;
      let elbowAngleL = 0;
      let wristAAngleL = 0;
      let wristBAngleL = 0;

      // Manual grip (C/X right hand, [/] left) -- 0 (open) to 1
      // (closed), ramped by GRIP_RATE like every other hold-to-move
      // control. *Engaged tracks whether that hand's weld is currently
      // holding the object (see GRIP_ENGAGE_AT/GRIP_RELEASE_AT's own
      // comment) -- separate from the fraction itself since engaging only
      // happens on crossing the threshold near the object, not just from
      // being closed.
      let rightGripFraction = 0;
      let leftGripFraction = 0;
      let rightGripEngaged = false;
      let leftGripEngaged = false;
      let rightHeldBody = -1; // which GRABBABLE_OBJECTS entry (by body id) rightGripEngaged refers to, -1 = none
      let leftHeldBody = -1;
      // The rest of what's needed to kinematically hold that object every
      // step (see applyGripPose) -- captured once at grab time, reused
      // every frame until release. Placeholder values until the first
      // grab; never read while *HeldBody is -1.
      let rightHeldRelPos: [number, number, number] = [0, 0, 0];
      let rightHeldRelQuat: [number, number, number, number] = [1, 0, 0, 0];
      let rightHeldDof = -1;
      let rightHeldQposAdr = -1;
      let leftHeldRelPos: [number, number, number] = [0, 0, 0];
      let leftHeldRelQuat: [number, number, number, number] = [1, 0, 0, 0];
      let leftHeldDof = -1;
      let leftHeldQposAdr = -1;
      // Cup's own pose at the start of "pullIn" (still resting on the
      // counter) or "placing" (wherever it was rigidly following the palm
      // to) -- captured once when each phase begins, so that phase can
      // smoothly blend the cup from there to its own target over its own
      // duration. Reused across both phases since they never overlap.
      let placeAnimStartPos: [number, number, number] = [0, 0, 0];
      let placeAnimStartQuat: [number, number, number, number] = [1, 0, 0, 0];

      // FPP head look-around: drag to turn the head like OrbitControls'
      // drag-to-orbit, just clamped to a human-ish range and re-centered
      // on the base's own forward direction (not a fixed world direction)
      // every frame -- see the render loop below.
      let fppYaw = 0;
      let fppPitch = 0;
      let fppDragging = false;
      let fppLastX = 0;
      let fppLastY = 0;

      if (interactive) {
        const onKeyDown = (e: KeyboardEvent) => {
          pressedKeys.add(e.key.toLowerCase());
        };
        const onKeyUp = (e: KeyboardEvent) => {
          pressedKeys.delete(e.key.toLowerCase());
        };
        // A key released while the tab was unfocused (e.g. alt-tabbing away
        // mid-hold) never fires its own keyup, which would otherwise leave
        // it stuck "held" -- clear everything on blur to catch that.
        const onBlur = () => {
          pressedKeys.clear();
        };
        window.addEventListener("blur", onBlur);
        window.addEventListener("keydown", onKeyDown);
        window.addEventListener("keyup", onKeyUp);

        const onPointerDown = (e: PointerEvent) => {
          if (viewRef.current !== "fpp") return;
          fppDragging = true;
          fppLastX = e.clientX;
          fppLastY = e.clientY;
        };
        const onPointerMove = (e: PointerEvent) => {
          if (!fppDragging) return;
          const dx = e.clientX - fppLastX;
          const dy = e.clientY - fppLastY;
          fppLastX = e.clientX;
          fppLastY = e.clientY;
          fppYaw = Math.max(-FPP_MAX_YAW, Math.min(FPP_MAX_YAW, fppYaw - dx * FPP_LOOK_SENSITIVITY));
          fppPitch = Math.max(FPP_MIN_PITCH, Math.min(FPP_MAX_PITCH, fppPitch - dy * FPP_LOOK_SENSITIVITY));
        };
        const onPointerUp = () => {
          fppDragging = false;
        };
        renderer.domElement.addEventListener("pointerdown", onPointerDown);
        window.addEventListener("pointermove", onPointerMove);
        window.addEventListener("pointerup", onPointerUp);

        removeKeyListeners = () => {
          window.removeEventListener("keydown", onKeyDown);
          window.removeEventListener("keyup", onKeyUp);
          window.removeEventListener("blur", onBlur);
          renderer?.domElement.removeEventListener("pointerdown", onPointerDown);
          window.removeEventListener("pointermove", onPointerMove);
          window.removeEventListener("pointerup", onPointerUp);
        };

        if (canPick) {
          pickRef.current = () => {
            if (pickPhase === "idle") {
              // A manually-grabbed object (see canGripRight/Left above)
              // would otherwise keep getting kinematically pinned to
              // whichever hand grabbed it (see applyGripPose) all through
              // this workflow's own reach for the cup -- clearing the
              // engaged flags first stops that (the workflow re-engages
              // rightGripEngaged itself once it actually grasps the cup).
              if (rightGripEngaged) {
                rightGripEngaged = false;
                rightHeldBody = -1;
              }
              if (leftGripEngaged) {
                leftGripEngaged = false;
                leftHeldBody = -1;
              }
              pickPhase = "driveToPick1";
              pickPhaseElapsedS = 0;
              pickDriveVX = 0;
              pickDriveVY = 0;
            }
          };
        }
      }

      const tmpPos = new THREE.Vector3();
      const tmpQuat = new THREE.Quaternion();
      const fppForward = new THREE.Vector3();
      const fppLookAt = new THREE.Vector3();
      const timestepMs = model.opt.timestep * 1000;
      let lastFrameTime: number | null = null;
      let lastView: ViewMode = "orbit";

      renderer.setAnimationLoop((now: number) => {
        if (disposed || !renderer) return;
        // Orbit controls own the camera only in "Center Stage"; every
        // other view (a fixed corner, or FPP -- both handled below, after
        // this frame's body positions are updated) takes the camera over
        // completely, so disable orbit's own input/update while active
        // rather than fight it. `enabled = false` also stops it from
        // consuming the pointer events FPP's own look-around needs.
        const isOrbitView = viewRef.current === "orbit";
        if (controls) controls.enabled = isOrbitView;
        if (isOrbitView) {
          // Re-entering orbit from a corner/FPP view: OrbitControls.update()
          // re-derives its internal orbit state from wherever camera.position
          // *currently* is, every call -- it doesn't remember a separately
          // preserved state -- so without resetting the camera first, it
          // would resume orbiting around whatever leftover FPP/corner
          // position the camera was last at instead of the original framing.
          if (lastView !== "orbit") {
            camera.position.set(...ORBIT_POSITION);
            controls?.target.set(...ORBIT_TARGET);
          }
          controls?.update();
        }
        lastView = viewRef.current;

        // WASD base teleop: W/S drive forward/back along the base's current
        // heading, A/D turn in place -- a standard unicycle-style scheme.
        // "Forward" is this body's own local +Z axis (BASE_FORWARD_AXIS),
        // read directly off its rotation matrix every frame via
        // getBodyAxisXY -- see that function's own comment for why (a
        // previous version derived a "yaw" angle and reconstructed a
        // heading from cos/sin of it, which was off by a constant 90
        // degrees and drove the robot sideways). Confirmed the *axis*
        // choice itself (not just the extraction method) by checking which
        // way the two hands sit relative to each other -- local +X is the
        // hand-to-hand line, so it's not the forward axis; local +Z is the
        // only one left. Wheel spin (cosmetic -- see DRIVE_WHEEL_RADIUS) is
        // kept in sync with commanded speed, not derived from ground
        // contact.
        if (canDrive && pickPhase === "idle") {
          const forward = (pressedKeys.has("w") ? 1 : 0) - (pressedKeys.has("s") ? 1 : 0);
          const turn = (pressedKeys.has("a") ? 1 : 0) - (pressedKeys.has("d") ? 1 : 0);
          const [fx, fy] = getBodyAxisXY(data.xmat, BASE_BODY, BASE_FORWARD_AXIS);
          data.ctrl[ACT_VX] = forward * DRIVE_SPEED * fx;
          data.ctrl[ACT_VY] = forward * DRIVE_SPEED * fy;
          data.ctrl[ACT_YAW] = turn * TURN_SPEED;
          const wheelSpeed = (forward * DRIVE_SPEED) / DRIVE_WHEEL_RADIUS;
          if (ACT_WHEEL_L >= 0) data.ctrl[ACT_WHEEL_L] = wheelSpeed;
          if (ACT_WHEEL_R >= 0) data.ctrl[ACT_WHEEL_R] = wheelSpeed;
        }

        if (!pausedRef.current) {
          if (lastFrameTime === null) lastFrameTime = now;
          // Cap physics catch-up so a slow/backgrounded tab doesn't try to
          // fast-forward the simulation into instability once it resumes.
          let remaining = Math.min(now - lastFrameTime, 50);
          lastFrameTime = now;
          const dtS = timestepMs / 1000;
          // Right-arm vs left-arm key checks -- separate, non-overlapping
          // key sets (no modifier), see LEFT_MIRROR_SIGN's own comment on
          // why this replaced an earlier Shift-chord scheme.
          const keyR = (k: string) => pressedKeys.has(k);
          const keyL = (k: string) => pressedKeys.has(k);
          // Set whenever this frame's substeps kinematically overrode
          // anything (held-object follow or the Workflow's finger pin) --
          // see the one mj_forward call after the loop, below, for why.
          let neededForwardSync = false;
          while (remaining > 0) {
            // Lift column teleop (U/L) -- disabled mid-sequence (forced back
            // to LIFT_MIN instead, see the Pick & Place block below) so a
            // height change never invalidates its offline-solved arm poses.
            if (canLift && pickPhase === "idle") {
              const liftInput = (pressedKeys.has("u") ? 1 : 0) - (pressedKeys.has("l") ? 1 : 0);
              liftHeight = Math.max(LIFT_MIN, Math.min(LIFT_MAX, liftHeight + liftInput * LIFT_RATE * dtS));
              data.ctrl[ACT_LIFT] = liftHeight;
            }

            // Waist bend teleop (F/R) -- same hold-to-move/disabled-mid-
            // sequence pattern as the lift column above. F bends forward
            // (negative, see BEND_MAX's own comment on the sign convention);
            // R straightens back toward 0.
            if (canBend && pickPhase === "idle") {
              const bendInput = (pressedKeys.has("r") ? 1 : 0) - (pressedKeys.has("f") ? 1 : 0);
              bendAngle = Math.max(BEND_MIN, Math.min(0, bendAngle + bendInput * BEND_RATE * dtS));
              data.ctrl[ACT_BEND] = bendAngle;
            }

            // Right shoulder, forward/back (T/G) -- see SHOULDER_FWD_MAX's
            // own comment on the sign convention (T is negative/forward, G
            // is positive/back, same reasoning as bend's F/R). 1/2 mirror
            // this onto the left shoulder (see LEFT_MIRROR_SIGN).
            if (canShoulderFwd && pickPhase === "idle") {
              const input = (keyR("g") ? 1 : 0) - (keyR("t") ? 1 : 0);
              shoulderFwdAngle = Math.max(
                -SHOULDER_FWD_MAX,
                Math.min(SHOULDER_BACK_MAX, shoulderFwdAngle + input * SHOULDER_FWD_RATE * dtS)
              );
              data.ctrl[ACT_SHOULDER_FWD] = shoulderFwdAngle;
            }
            if (canShoulderFwdL && pickPhase === "idle") {
              const input = (keyL("2") ? 1 : 0) - (keyL("1") ? 1 : 0);
              shoulderFwdAngleL = Math.max(
                -SHOULDER_BACK_MAX,
                Math.min(SHOULDER_FWD_MAX, shoulderFwdAngleL + input * SHOULDER_FWD_RATE * dtS * LEFT_MIRROR_SIGN)
              );
              data.ctrl[ACT_SHOULDER_FWD_L] = shoulderFwdAngleL;
            }

            // Right shoulder, lateral raise (Y/H); 3/4 mirrors to left.
            if (canShoulderLat && pickPhase === "idle") {
              const input = (keyR("y") ? 1 : 0) - (keyR("h") ? 1 : 0);
              shoulderLatAngle = Math.max(0, Math.min(SHOULDER_LAT_MAX, shoulderLatAngle + input * SHOULDER_LAT_RATE * dtS));
              data.ctrl[ACT_SHOULDER_LAT] = shoulderLatAngle;
            }
            if (canShoulderLatL && pickPhase === "idle") {
              const input = (keyL("3") ? 1 : 0) - (keyL("4") ? 1 : 0);
              shoulderLatAngleL = Math.max(
                -SHOULDER_LAT_MAX,
                Math.min(0, shoulderLatAngleL + input * SHOULDER_LAT_RATE * dtS * LEFT_MIRROR_SIGN)
              );
              data.ctrl[ACT_SHOULDER_LAT_L] = shoulderLatAngleL;
            }

            // Right upper-arm roll (I/K) -- full mechanical range, no
            // separate raise-angle spec given for this one (see
            // ARM_ROLL_RATE's own comment). 5/6 mirrors to left.
            if (canArmRoll && pickPhase === "idle") {
              const input = (keyR("i") ? 1 : 0) - (keyR("k") ? 1 : 0);
              armRollAngle = Math.max(ARM_ROLL_MIN, Math.min(ARM_ROLL_MAX, armRollAngle + input * ARM_ROLL_RATE * dtS));
              data.ctrl[ACT_ARM_ROLL] = armRollAngle;
            }
            if (canArmRollL && pickPhase === "idle") {
              const input = (keyL("5") ? 1 : 0) - (keyL("6") ? 1 : 0);
              armRollAngleL = Math.max(
                ARM_ROLL_MIN_L,
                Math.min(ARM_ROLL_MAX_L, armRollAngleL + input * ARM_ROLL_RATE * dtS * LEFT_MIRROR_SIGN)
              );
              data.ctrl[ACT_ARM_ROLL_L] = armRollAngleL;
            }

            // Right elbow (O/P) -- 0 (straight) to the joint's own positive
            // limit (bent); same reasoning as ARM_ROLL above. 7/8 mirrors to
            // left (the left elbow's own positive limit bends the *other*
            // way, per LEFT_MIRROR_SIGN, so 0 is still straight but the bent
            // end is ELBOW_MIN_L, not a max).
            if (canElbow && pickPhase === "idle") {
              const input = (keyR("o") ? 1 : 0) - (keyR("p") ? 1 : 0);
              elbowAngle = Math.max(0, Math.min(ELBOW_MAX, elbowAngle + input * ELBOW_RATE * dtS));
              data.ctrl[ACT_ELBOW] = elbowAngle;
            }
            if (canElbowL && pickPhase === "idle") {
              const input = (keyL("7") ? 1 : 0) - (keyL("8") ? 1 : 0);
              elbowAngleL = Math.max(ELBOW_MIN_L, Math.min(0, elbowAngleL + input * ELBOW_RATE * dtS * LEFT_MIRROR_SIGN));
              data.ctrl[ACT_ELBOW_L] = elbowAngleL;
            }

            // Right wrist, axis A (J/N) and axis B (V/B) -- each just uses
            // its own full mechanical range (see WRIST_A_RATE's own
            // comment). 9/0 and =/- mirror to the left wrist.
            if (canWristA && pickPhase === "idle") {
              const input = (keyR("j") ? 1 : 0) - (keyR("n") ? 1 : 0);
              wristAAngle = Math.max(WRIST_A_MIN, Math.min(WRIST_A_MAX, wristAAngle + input * WRIST_A_RATE * dtS));
              data.ctrl[ACT_WRIST_A] = wristAAngle;
            }
            if (canWristAL && pickPhase === "idle") {
              const input = (keyL("9") ? 1 : 0) - (keyL("0") ? 1 : 0);
              wristAAngleL = Math.max(
                WRIST_A_MIN_L,
                Math.min(WRIST_A_MAX_L, wristAAngleL + input * WRIST_A_RATE * dtS * LEFT_MIRROR_SIGN)
              );
              data.ctrl[ACT_WRIST_A_L] = wristAAngleL;
            }
            if (canWristB && pickPhase === "idle") {
              const input = (keyR("v") ? 1 : 0) - (keyR("b") ? 1 : 0);
              wristBAngle = Math.max(WRIST_B_MIN, Math.min(WRIST_B_MAX, wristBAngle + input * WRIST_B_RATE * dtS));
              data.ctrl[ACT_WRIST_B] = wristBAngle;
            }
            if (canWristBL && pickPhase === "idle") {
              const input = (keyL("=") ? 1 : 0) - (keyL("-") ? 1 : 0);
              wristBAngleL = Math.max(
                WRIST_B_MIN_L,
                Math.min(WRIST_B_MAX_L, wristBAngleL + input * WRIST_B_RATE * dtS * LEFT_MIRROR_SIGN)
              );
              data.ctrl[ACT_WRIST_B_L] = wristBAngleL;
            }

            // Manual grip -- C/X right hand, [/] left. Ramps the
            // fraction like every other hold-to-move control, and separately
            // handles grabbing/letting go on crossing
            // GRIP_ENGAGE_AT/GRIP_RELEASE_AT (see that constant's own
            // comment) -- the finger motion itself always runs regardless of
            // whether anything is actually in reach to grab. Holding itself
            // is a kinematic follow (applyGripPose, called every step while
            // engaged), not an equality constraint -- see that function's
            // own comment for why.
            if (canGripRight && pickPhase === "idle") {
              const input = (keyR("c") ? 1 : 0) - (keyR("x") ? 1 : 0);
              rightGripFraction = Math.max(0, Math.min(1, rightGripFraction + input * GRIP_RATE * dtS));
              const fingerTarget = lerpArr(PICK_FINGER_OPEN, PICK_FINGER_CLOSED, rightGripFraction);
              for (let i = 0; i < ACT_PICK_FINGERS.length; i++) data.ctrl[ACT_PICK_FINGERS[i]] = fingerTarget[i];

              if (!rightGripEngaged && rightGripFraction > GRIP_ENGAGE_AT) {
                const target = findGrabTarget(RIGHT_GRIP_ANCHOR_GEOM, leftHeldBody);
                if (target) {
                  const { relPos, relQuat } = computeGripRelpose(data, RIGHT_GRIP_ANCHOR_GEOM, target.body);
                  rightGripEngaged = true;
                  rightHeldBody = target.body;
                  rightHeldRelPos = snugRelPos(relPos);
                  rightHeldRelQuat = relQuat;
                  rightHeldDof = target.dof;
                  rightHeldQposAdr = target.qposAdr;
                }
              } else if (rightGripEngaged && rightGripFraction < GRIP_RELEASE_AT) {
                rightGripEngaged = false;
                rightHeldBody = -1;
              }
              // Following the held object itself (applyGripPose) happens
              // once, after mj_step, below -- see that block's own comment.
            }
            if (canGripLeft && pickPhase === "idle") {
              const input = (keyL("[") ? 1 : 0) - (keyL("]") ? 1 : 0);
              leftGripFraction = Math.max(0, Math.min(1, leftGripFraction + input * GRIP_RATE * dtS));
              const fingerTarget = lerpArr(LEFT_FINGER_OPEN, LEFT_FINGER_CLOSED, leftGripFraction);
              for (let i = 0; i < ACT_LEFT_FINGERS.length; i++) data.ctrl[ACT_LEFT_FINGERS[i]] = fingerTarget[i];

              if (!leftGripEngaged && leftGripFraction > GRIP_ENGAGE_AT) {
                const target = findGrabTarget(LEFT_GRIP_ANCHOR_GEOM, rightHeldBody);
                if (target) {
                  const { relPos, relQuat } = computeGripRelpose(data, LEFT_GRIP_ANCHOR_GEOM, target.body);
                  leftGripEngaged = true;
                  leftHeldBody = target.body;
                  leftHeldRelPos = snugRelPos(relPos);
                  leftHeldRelQuat = relQuat;
                  leftHeldDof = target.dof;
                  leftHeldQposAdr = target.qposAdr;
                }
              } else if (leftGripEngaged && leftGripFraction < GRIP_RELEASE_AT) {
                leftGripEngaged = false;
                leftHeldBody = -1;
              }
              // Following the held object itself (applyGripPose) happens
              // once, after mj_step, below -- see that block's own comment.
            }

            // Scripted Workflow: see the PICK_* constants' own comments for
            // the phase list, timings, and the offline-solved arm targets.
            // Advances pickPhaseElapsedS the same way waveElapsedS advances
            // above, for the same frame-hitch-immunity reason.
            // Fingers are cosmetic once the grip is engaged (the cup is
            // held by applyGripPose below, not finger contact), and get
            // forced kinematically to fingerTarget every step from "close"
            // through the end of the carry (see fingerHold's own comment) --
            // set here so that override has this frame's actual target to
            // use.
            let fingerTarget = PICK_FINGER_OPEN;
            let fingerHold = false;
            // Set only during "pullIn"/"placing" -- a directly scripted cup
            // position/orientation for this step (see those phases' own
            // comments), applied after mj_step the same way fingerHold's
            // override is.
            let cupAnimPos: [number, number, number] | null = null;
            let cupAnimQuat: [number, number, number, number] | null = null;
            if (
              canPick &&
              pickPhase !== "idle" &&
              (pickPhase === "driveToPick1" ||
                pickPhase === "driveToPick2" ||
                pickPhase === "driveToPick3" ||
                pickPhase === "driveToCoffee1" ||
                pickPhase === "driveToCoffee2") &&
              pickPhaseElapsedS > PICK_DRIVE_TIMEOUT_S
            ) {
              // Safety net: the drive phases assume a specific starting
              // position (spawn for driveToPick) and a specific, verified-
              // clear route -- triggering the workflow from some other,
              // unverified position/orientation (e.g. after driving around
              // manually first) could in principle walk the straight-line
              // P-controller into an obstacle it was never checked against,
              // where it would just push against it forever, permanently
              // blocking every other idle-gated control (WASD included)
              // along with it. Aborting back to idle after a generous
              // timeout means a bad start position degrades to "the
              // workflow didn't finish," not "WASD is broken now."
              rightGripEngaged = false;
              rightHeldBody = -1;
              pickPhase = "idle";
              pickPhaseElapsedS = 0;
              pickDriveVX = 0;
              pickDriveVY = 0;
            }
            if (canPick && pickPhase !== "idle") {
              // Yaw hold, active through every phase -- see PICK_YAW_KP's
              // own comment on why this can't be left uncontrolled.
              const yawErr = Math.atan2(Math.sin(PICK_PARK_YAW - data.qpos[YAW_QPOS_ADR]), Math.cos(PICK_PARK_YAW - data.qpos[YAW_QPOS_ADR]));
              data.ctrl[ACT_YAW] = Math.max(-PICK_YAW_MAX_RATE, Math.min(PICK_YAW_MAX_RATE, PICK_YAW_KP * yawErr));

              const t = pickPhaseElapsedS;
              let armTarget = PICK_REST_QPOS;
              // Waist bend target -- see PICK_GRASP_BEND's own comment for
              // the pickup side's ramp (0 through "reach", up during
              // "lower", held, back to 0 during "lift") and PICK_PLACE_BEND's
              // for the placement side's equivalent, smaller ramp. Defaults
              // to upright; only those phases ever change it, same as
              // PICK_REST_QPOS's default above.
              let bendTarget = 0;
              let driveTarget: [number, number] | null = null;
              let nextOnArrive: PickPhase | null = null;

              if (pickPhase === "driveToPick1") {
                driveTarget = PICK_WAYPOINT_EAST;
                nextOnArrive = "driveToPick2";
              } else if (pickPhase === "driveToPick2") {
                driveTarget = PICK_WAYPOINT_EAST_NORTH;
                nextOnArrive = "driveToPick3";
              } else if (pickPhase === "driveToPick3") {
                driveTarget = PICK_PARK_PICK;
                nextOnArrive = "reach";
              } else if (pickPhase === "reach") {
                // Arm-only: stretches from resting straight out to the full
                // grasp pose while the torso stays upright -- the visible
                // "reaching forward" motion (the user's own ask: "arms
                // stretch with T button... arm moves forward and reaches to
                // cup"). The waist bend that finishes closing the gap to the
                // cup happens next, in "lower", once the arm's already fully
                // extended -- not blended in here -- so the two read as
                // distinct motions instead of one folding blob.
                armTarget = lerpArr(PICK_REST_QPOS, PICK_GRASP_QPOS, smoothstep(t / PICK_REACH_S));
                bendTarget = 0;
                if (t >= PICK_REACH_S) {
                  pickPhase = "lower";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "lower") {
                // Arm holds at full extension; only the waist bends now, to
                // bring the already-outstretched hand the rest of the way
                // down to the cup.
                armTarget = PICK_GRASP_QPOS;
                bendTarget = PICK_GRASP_BEND * smoothstep(t / PICK_LOWER_S);
                if (t >= PICK_LOWER_S) {
                  pickPhase = "close";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "close") {
                armTarget = PICK_GRASP_QPOS;
                bendTarget = PICK_GRASP_BEND;
                fingerTarget = lerpArr(PICK_FINGER_OPEN, PICK_FINGER_CLOSED, smoothstep(t / PICK_CLOSE_S));
                fingerHold = true;
                if (t >= PICK_CLOSE_S) {
                  pickPhase = "settle";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "settle") {
                armTarget = PICK_GRASP_QPOS;
                bendTarget = PICK_GRASP_BEND;
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                if (t >= PICK_SETTLE_S) {
                  // Only now -- once the arm has actually finished settling
                  // into PICK_GRASP_QPOS, not the instant fingers finish
                  // closing -- compute the grip (see PICK_REACH_S's own
                  // comment on why: these joints take real seconds to
                  // settle, and capturing early froze in whatever
                  // still-mid-swing offset existed at that moment). Not
                  // engaged yet, though -- even this bent-forward reach still
                  // leaves the palm a few cm from the cup (see
                  // PICK_GRASP_QPOS's own comment), so snapping straight to
                  // the snugged relPos here would still show a small final
                  // jump. "pullIn" (next) animates that last bit shut over
                  // real time instead -- now short enough (PICK_PULL_IN_S) to
                  // read as the hand settling onto the cup, not a slide.
                  const { relPos, relQuat } = computeGripRelpose(data, RIGHT_GRIP_ANCHOR_GEOM, CUP_OBJECT_BODY);
                  rightHeldRelPos = snugRelPos(relPos);
                  rightHeldRelQuat = relQuat;
                  rightHeldDof = CUP_OBJECT_DOF;
                  rightHeldQposAdr = CUP_OBJECT_QPOS_ADR;
                  placeAnimStartPos = [
                    data.xpos[CUP_OBJECT_BODY * 3],
                    data.xpos[CUP_OBJECT_BODY * 3 + 1],
                    data.xpos[CUP_OBJECT_BODY * 3 + 2],
                  ];
                  placeAnimStartQuat = [
                    data.xquat[CUP_OBJECT_BODY * 4],
                    data.xquat[CUP_OBJECT_BODY * 4 + 1],
                    data.xquat[CUP_OBJECT_BODY * 4 + 2],
                    data.xquat[CUP_OBJECT_BODY * 4 + 3],
                  ];
                  pickPhase = "pullIn";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "pullIn") {
                // Animates the cup from where it actually was resting
                // (placeAnimStartPos/Quat, captured above) to the snugged
                // grip pose -- recomputed live off the anchor every step,
                // not just once, so this still lands correctly even though
                // the arm/fingers are still finishing their own settling
                // during this same window. Once the blend reaches 1, the
                // cup is already exactly where applyGripPose would put it,
                // so engaging rightGripEngaged then is a seamless handoff,
                // not a second jump.
                armTarget = PICK_GRASP_QPOS;
                bendTarget = PICK_GRASP_BEND;
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                {
                  const blend = smoothstep(t / PICK_PULL_IN_S);
                  const am = RIGHT_GRIP_ANCHOR_GEOM * 9;
                  const ax = data.geom_xpos[RIGHT_GRIP_ANCHOR_GEOM * 3];
                  const ay = data.geom_xpos[RIGHT_GRIP_ANCHOR_GEOM * 3 + 1];
                  const az = data.geom_xpos[RIGHT_GRIP_ANCHOR_GEOM * 3 + 2];
                  const liveTargetPos: [number, number, number] = [
                    ax +
                      data.geom_xmat[am] * rightHeldRelPos[0] +
                      data.geom_xmat[am + 1] * rightHeldRelPos[1] +
                      data.geom_xmat[am + 2] * rightHeldRelPos[2],
                    ay +
                      data.geom_xmat[am + 3] * rightHeldRelPos[0] +
                      data.geom_xmat[am + 4] * rightHeldRelPos[1] +
                      data.geom_xmat[am + 5] * rightHeldRelPos[2],
                    az +
                      data.geom_xmat[am + 6] * rightHeldRelPos[0] +
                      data.geom_xmat[am + 7] * rightHeldRelPos[1] +
                      data.geom_xmat[am + 8] * rightHeldRelPos[2],
                  ];
                  const liveTargetQuat = quatMul(mat3ToQuat(data.geom_xmat, am), rightHeldRelQuat);
                  cupAnimPos = lerpArr(placeAnimStartPos, liveTargetPos, blend) as [number, number, number];
                  const rawQuat = lerpArr(placeAnimStartQuat, liveTargetQuat, blend);
                  const qn = Math.hypot(rawQuat[0], rawQuat[1], rawQuat[2], rawQuat[3]) || 1;
                  cupAnimQuat = [rawQuat[0] / qn, rawQuat[1] / qn, rawQuat[2] / qn, rawQuat[3] / qn];
                }
                if (t >= PICK_PULL_IN_S) {
                  rightGripEngaged = true;
                  rightGripFraction = 1;
                  rightHeldBody = CUP_OBJECT_BODY;
                  pickPhase = "lift";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "lift") {
                armTarget = lerpArr(PICK_GRASP_QPOS, PICK_CARRY_QPOS, smoothstep(t / PICK_LIFT_S));
                bendTarget = PICK_GRASP_BEND * (1 - smoothstep(t / PICK_LIFT_S));
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                if (t >= PICK_LIFT_S) {
                  pickPhase = "driveToCoffee1";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "driveToCoffee1") {
                armTarget = PICK_CARRY_QPOS;
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                driveTarget = PICK_WAYPOINT_NORTH;
                nextOnArrive = "driveToCoffee2";
              } else if (pickPhase === "driveToCoffee2") {
                armTarget = PICK_CARRY_QPOS;
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                driveTarget = PICK_COFFEE_PARK;
                if (
                  Math.hypot(
                    PICK_COFFEE_PARK[0] - data.xpos[BASE_BODY * 3],
                    PICK_COFFEE_PARK[1] - data.xpos[BASE_BODY * 3 + 1]
                  ) < PICK_DRIVE_ARRIVE_DIST
                ) {
                  pickPhase = "lowerPlace";
                  pickPhaseElapsedS = 0;
                  pickDriveVX = 0;
                  pickDriveVY = 0;
                }
              } else if (pickPhase === "lowerPlace") {
                // Arm-only stretch toward the coffee-machine counter, same
                // shape as "reach" -- bend stays at 0 while the arm extends.
                armTarget = lerpArr(PICK_CARRY_QPOS, PICK_PLACE_QPOS, smoothstep(t / PICK_LOWER_PLACE_S));
                bendTarget = 0;
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                if (t >= PICK_LOWER_PLACE_S) {
                  pickPhase = "settlePlace";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "settlePlace") {
                // Arm holds at full extension; the small placement bend (see
                // PICK_PLACE_BEND's own comment) ramps in now to bring the
                // hand the rest of the way down to the counter.
                armTarget = PICK_PLACE_QPOS;
                bendTarget = PICK_PLACE_BEND * smoothstep(t / PICK_SETTLE_PLACE_S);
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                if (t >= PICK_SETTLE_PLACE_S) {
                  // Stop following the palm and hand off to "placing" instead
                  // of just letting go here -- PICK_PLACE_QPOS/PICK_PLACE_BEND
                  // get close (~11.5cm) but not exact, so releasing at this
                  // exact point would leave the cup sitting slightly off from
                  // a clean, on-counter spot rather than exactly on it.
                  placeAnimStartPos = [
                    data.xpos[CUP_OBJECT_BODY * 3],
                    data.xpos[CUP_OBJECT_BODY * 3 + 1],
                    data.xpos[CUP_OBJECT_BODY * 3 + 2],
                  ];
                  placeAnimStartQuat = [
                    data.xquat[CUP_OBJECT_BODY * 4],
                    data.xquat[CUP_OBJECT_BODY * 4 + 1],
                    data.xquat[CUP_OBJECT_BODY * 4 + 2],
                    data.xquat[CUP_OBJECT_BODY * 4 + 3],
                  ];
                  rightGripEngaged = false;
                  rightHeldBody = -1;
                  pickPhase = "placing";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "placing") {
                // Animates the cup the short remaining distance from wherever
                // the palm left it to PICK_PLACE_TARGET, on the counter next
                // to the machine -- a fixed world point, not a joint-space arm
                // target, since PICK_PLACE_QPOS/PICK_PLACE_BEND land close but
                // not exactly on it (see that constant's own comment).
                // Orientation is left as it was at handoff (only position
                // blends) -- the cup was already upright throughout the
                // carry, nothing needs to rotate here.
                armTarget = PICK_PLACE_QPOS;
                bendTarget = PICK_PLACE_BEND;
                fingerTarget = PICK_FINGER_CLOSED;
                fingerHold = true;
                {
                  const blend = smoothstep(t / PICK_PLACING_S);
                  cupAnimPos = lerpArr(placeAnimStartPos, PICK_PLACE_TARGET, blend) as [number, number, number];
                  cupAnimQuat = placeAnimStartQuat;
                }
                if (t >= PICK_PLACING_S) {
                  pickPhase = "release";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "release") {
                armTarget = PICK_PLACE_QPOS;
                bendTarget = PICK_PLACE_BEND;
                fingerTarget = lerpArr(PICK_FINGER_CLOSED, PICK_FINGER_OPEN, smoothstep(t / PICK_RELEASE_S));
                fingerHold = true;
                if (t >= PICK_RELEASE_S) {
                  pickPhase = "retractPlace";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "retractPlace") {
                armTarget = lerpArr(PICK_PLACE_QPOS, PICK_CARRY_QPOS, smoothstep(t / PICK_RETRACT_PLACE_S));
                bendTarget = PICK_PLACE_BEND * (1 - smoothstep(t / PICK_RETRACT_PLACE_S));
                fingerTarget = PICK_FINGER_OPEN;
                fingerHold = true;
                if (t >= PICK_RETRACT_PLACE_S) {
                  // Workflow done -- sync the idle teleop targets to
                  // PICK_CARRY_QPOS (where the arm actually is now) so
                  // T/G/Y/H/I/K/O/P/J/N/V/B resume from here instead of
                  // snapping the arm somewhere else on the first
                  // idle-gated key.
                  const [sf, sl, ar, el, wa, wb] = PICK_CARRY_QPOS;
                  shoulderFwdAngle = sf;
                  shoulderLatAngle = sl;
                  armRollAngle = ar;
                  elbowAngle = el;
                  wristAAngle = wa;
                  wristBAngle = wb;
                  rightGripFraction = 0;
                  pickPhase = "idle";
                  pickPhaseElapsedS = 0;
                }
              }

              if (driveTarget && nextOnArrive) {
                const ex = driveTarget[0] - data.xpos[BASE_BODY * 3 + 0];
                const ey = driveTarget[1] - data.xpos[BASE_BODY * 3 + 1];
                const dist = Math.hypot(ex, ey);
                if (dist < PICK_DRIVE_ARRIVE_DIST) {
                  pickDriveVX = 0;
                  pickDriveVY = 0;
                  pickPhase = nextOnArrive;
                  pickPhaseElapsedS = 0;
                } else {
                  const desiredVX = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_DRIVE_KP * ex));
                  const desiredVY = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_DRIVE_KP * ey));
                  const maxDelta = PICK_DRIVE_MAX_ACCEL * dtS;
                  pickDriveVX += Math.max(-maxDelta, Math.min(maxDelta, desiredVX - pickDriveVX));
                  pickDriveVY += Math.max(-maxDelta, Math.min(maxDelta, desiredVY - pickDriveVY));
                }
              } else if (driveTarget) {
                // driveToCoffee2's own arrival is handled inline above (it
                // needs to run the teleop-sync/release-to-idle logic, not
                // just switch phases), but still needs the same P-control
                // drive while en route.
                const ex = driveTarget[0] - data.xpos[BASE_BODY * 3 + 0];
                const ey = driveTarget[1] - data.xpos[BASE_BODY * 3 + 1];
                const dist = Math.hypot(ex, ey);
                if (dist < PICK_DRIVE_ARRIVE_DIST) {
                  pickDriveVX = 0;
                  pickDriveVY = 0;
                } else {
                  const desiredVX = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_DRIVE_KP * ex));
                  const desiredVY = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_DRIVE_KP * ey));
                  const maxDelta = PICK_DRIVE_MAX_ACCEL * dtS;
                  pickDriveVX += Math.max(-maxDelta, Math.min(maxDelta, desiredVX - pickDriveVX));
                  pickDriveVY += Math.max(-maxDelta, Math.min(maxDelta, desiredVY - pickDriveVY));
                }
              } else if (
                pickPhase === "reach" || pickPhase === "lower" || pickPhase === "close" ||
                pickPhase === "settle" || pickPhase === "pullIn" || pickPhase === "lift" ||
                pickPhase === "lowerPlace" || pickPhase === "settlePlace" || pickPhase === "placing" ||
                pickPhase === "release" || pickPhase === "retractPlace"
              ) {
                // See PICK_BEND_HOLD_KP's own comment -- these are the only
                // phases that ever bend the waist, so they're the only ones
                // that need this stiffer hold instead of a plain zero. Two
                // different park spots to hold against depending on which
                // side of the workflow this is.
                const holdPark = pickPhase === "reach" || pickPhase === "lower" || pickPhase === "close" ||
                  pickPhase === "settle" || pickPhase === "pullIn" || pickPhase === "lift"
                  ? PICK_PARK_PICK
                  : PICK_COFFEE_PARK;
                const ex = holdPark[0] - data.xpos[BASE_BODY * 3 + 0];
                const ey = holdPark[1] - data.xpos[BASE_BODY * 3 + 1];
                pickDriveVX = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_BEND_HOLD_KP * ex));
                pickDriveVY = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_BEND_HOLD_KP * ey));
              } else {
                pickDriveVX = 0;
                pickDriveVY = 0;
              }
              data.ctrl[ACT_VX] = pickDriveVX;
              data.ctrl[ACT_VY] = pickDriveVY;

              for (let i = 0; i < ACT_PICK_ARM.length; i++) data.ctrl[ACT_PICK_ARM[i]] = armTarget[i];
              for (let i = 0; i < ACT_PICK_FINGERS.length; i++) data.ctrl[ACT_PICK_FINGERS[i]] = fingerTarget[i];
              // Every PICK_*_QPOS arm target was solved at LIFT_MIN -- force
              // that regardless of whatever the user last set with U/L.
              if (canLift) {
                liftHeight = LIFT_MIN;
                data.ctrl[ACT_LIFT] = LIFT_MIN;
              }
              // Waist bend now follows bendTarget (see PICK_GRASP_BEND's own
              // comment) instead of being forced to 0 -- it's 0 outside
              // "reach".."lift" anyway (bendTarget's own default above), so
              // this still overrides whatever the user last set with F/R
              // during every other phase, same as before.
              if (canBend) {
                bendAngle = bendTarget;
                data.ctrl[ACT_BEND] = bendTarget;
              }

              pickPhaseElapsedS += dtS;
            }

            mujoco.mj_step(model, data);

            // Kinematically pin the right hand's fingers to fingerTarget
            // instead of trusting their own actuators for the rest of this
            // step -- these specific finger joints have been confirmed (via
            // extended headless simulation) to slowly drift open under
            // gravity at the grasp pose regardless of how long they're
            // given to "settle": holding them via their normal position
            // actuators there is not just slow to converge, it never
            // converges. Since the cup itself is held by applyGripPose
            // below, not finger contact, the fingers are purely cosmetic
            // here and can be driven directly without affecting the hold.
            // No mj_forward here (or after applyGripPose below) -- the
            // *next* mj_step already recomputes forward kinematics from
            // whatever qpos it's handed as the very first thing it does,
            // same as if this were any other qpos edit between steps. An
            // explicit mj_forward call per substep (there can be dozens per
            // rendered frame -- see the catch-up loop's own comment) was
            // pure wasted work that scaled with how many objects were held,
            // permanently, from the moment anything was first grabbed --
            // reported live as the whole page turning sluggish/unresponsive
            // (WASD included) as soon as something was picked up. One
            // mj_forward after the whole loop (below) is enough to make the
            // very last correction visible to this frame's render.
            if (fingerHold) {
              for (let i = 0; i < PICK_FINGER_QPOSADR.length; i++) {
                if (PICK_FINGER_QPOSADR[i] < 0) continue;
                data.qpos[PICK_FINGER_QPOSADR[i]] = fingerTarget[i];
                data.qvel[PICK_FINGER_DOFADR[i]] = 0;
              }
              neededForwardSync = true;
            }

            // Kinematic follow for whatever's currently grabbed (manual
            // grip or the Workflow above) -- runs regardless of pickPhase
            // now, since the Workflow can hold the cup through phases other
            // than "idle" (see the "settle" phase above, and manual grip's
            // own engage logic higher up, which no longer applies this
            // itself -- see that block's own updated comment).
            if (rightGripEngaged) {
              applyGripPose(RIGHT_GRIP_ANCHOR_GEOM, rightHeldRelPos, rightHeldRelQuat, rightHeldQposAdr, rightHeldDof);
              neededForwardSync = true;
            }
            if (leftGripEngaged) {
              applyGripPose(LEFT_GRIP_ANCHOR_GEOM, leftHeldRelPos, leftHeldRelQuat, leftHeldQposAdr, leftHeldDof);
              neededForwardSync = true;
            }
            // "pullIn"/"placing"'s own directly-scripted cup pose (see
            // those phases' own comments) -- set instead of, not alongside,
            // the two blocks above (neither is engaged during either
            // phase).
            if (cupAnimPos && cupAnimQuat) {
              data.qpos[CUP_OBJECT_QPOS_ADR] = cupAnimPos[0];
              data.qpos[CUP_OBJECT_QPOS_ADR + 1] = cupAnimPos[1];
              data.qpos[CUP_OBJECT_QPOS_ADR + 2] = cupAnimPos[2];
              data.qpos[CUP_OBJECT_QPOS_ADR + 3] = cupAnimQuat[0];
              data.qpos[CUP_OBJECT_QPOS_ADR + 4] = cupAnimQuat[1];
              data.qpos[CUP_OBJECT_QPOS_ADR + 5] = cupAnimQuat[2];
              data.qpos[CUP_OBJECT_QPOS_ADR + 6] = cupAnimQuat[3];
              for (let k = 0; k < 6; k++) data.qvel[CUP_OBJECT_DOF + k] = 0;
              neededForwardSync = true;
            }

            remaining -= timestepMs;
          }
          // Exactly one forward pass for the whole frame (not one per
          // substep -- see that loop's own comment) so the render below
          // sees this frame's very last kinematic correction, not
          // whatever mj_step's own internal forward pass computed just
          // before it.
          if (neededForwardSync) mujoco.mj_forward(model, data);
        } else {
          lastFrameTime = null;
        }

        for (let b = 0; b < model.nbody; b++) {
          const body = bodies[b];
          if (!body) continue;
          body.position.copy(getPosition(data.xpos, b, tmpPos));
          body.quaternion.copy(getQuaternion(data.xquat, b, tmpQuat));
        }

        // FPP: camera mounted at the head, looking the same direction the
        // base's own drive logic (above) treats as "forward" -- so this is
        // literally "the direction WASD's W currently drives," which is
        // what makes it useful for seeing where the robot is headed into a
        // corner the fixed orbit view can't frame well. Bypasses
        // three.js's Y-up bodies entirely except for the head's position
        // (bodies[] is already swizzled by getPosition above); the look
        // direction is computed straight from mujoco's own xmat and
        // swizzled by hand (mujoco (x,y,0) horizontal -> three.js (x,0,-y)),
        // same convention getPosition/getQuaternion use.
        if (viewRef.current.startsWith("corner")) {
          // Fixed "security camera" shots -- see CORNER_VIEWS. Static, but
          // cheap enough to just re-apply every frame rather than track
          // "did the view just change."
          const idx = Number(viewRef.current.slice(-1)) - 1;
          const preset = CORNER_VIEWS[idx];
          if (preset) {
            camera.position.set(...preset.pos);
            camera.lookAt(...preset.lookAt);
          }
        } else if (viewRef.current === "fpp" && canFpp) {
          // FPP: camera mounted exactly at the head geom's own live world
          // pose (data.geom_xpos/HEAD_GEOM -- see canFpp's own comment for
          // why this, not base_link + a guessed height/forward offset, is
          // what actually lands the camera at the robot's eye instead of
          // off to one side). Look direction is still "the direction WASD's
          // W currently drives" (same getBodyAxisXY this frame's teleop
          // above used), turned further by the head look-around drag
          // (fppYaw/fppPitch, see the pointer handlers above) -- there's no
          // independent neck joint on this rig, so aiming still follows the
          // base's own heading, only the camera's *position* comes from the
          // head now. Built as an explicit (yaw, pitch) -> direction,
          // three.js's usual FPS-camera convention, rather than rotating
          // the forward vector by hand -- much harder to get a sign wrong.
          const [fx, fy] = getBodyAxisXY(data.xmat, BASE_BODY, BASE_FORWARD_AXIS);
          const baseYaw = Math.atan2(fx, -fy); // three.js horizontal forward (fx, -fy) -> angle
          const totalYaw = baseYaw + fppYaw;
          const cosPitch = Math.cos(fppPitch);
          fppForward.set(Math.sin(totalYaw) * cosPitch, Math.sin(fppPitch), Math.cos(totalYaw) * cosPitch);
          getPosition(data.geom_xpos, HEAD_GEOM, camera.position);
          // The head geom's own bounding radius is ~0.158m (measured) --
          // sitting exactly at its center (as above) puts the camera inside
          // its own mesh, which reads as a plain black view (looking at the
          // inside of a shell, whatever isn't backface-culled sitting right
          // up against the near clip plane). Nudged forward, along the
          // level (unpitched) heading rather than the full look direction,
          // so looking sharply up/down doesn't dive the camera into the
          // ceiling/floor mesh instead -- comfortably past that radius.
          camera.position.x += Math.sin(totalYaw) * FPP_FORWARD_CLEARANCE;
          camera.position.z += Math.cos(totalYaw) * FPP_FORWARD_CLEARANCE;
          fppLookAt.copy(camera.position).add(fppForward);
          camera.lookAt(fppLookAt);
        }

        renderer.render(scene, camera);
      });

      setStatus("ready");
    }

    init().catch((err) => {
      console.error("Failed to load MuJoCo simulation:", err);
      if (!disposed) setStatus("error");
    });

    return () => {
      disposed = true;
      resizeObserver?.disconnect();
      removeKeyListeners?.();
      controls?.dispose();
      renderer?.setAnimationLoop(null);
      if (renderer) {
        const canvas = renderer.domElement;
        renderer.dispose();
        canvas.remove();
      }
      data?.delete?.();
      model?.delete?.();
    };
    // `interactive` only changes camera controls, not worth re-initializing
    // the whole WASM module/scene for -- it's fixed per mount in practice.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const showControls = interactive && status === "ready";
  const buttonClass =
    "rounded-sm border border-border-strong bg-panel/90 px-4 py-2 text-xs text-off-white backdrop-blur transition-colors hover:bg-panel-raised cursor-pointer";

  const controlButtons = (
    <>
      <button onClick={() => setPaused((p) => !p)} className={buttonClass}>
        {paused ? "Resume" : "Pause"}
      </button>
      <button onClick={() => resetRef.current()} className={buttonClass}>
        Reset
      </button>
      <select
        value={view}
        onChange={(e) => setView(e.target.value as ViewMode)}
        className={clsx(buttonClass, "appearance-none pr-8")}
      >
        {VIEW_OPTIONS.map((opt) => (
          <option key={opt.value} value={opt.value} className="bg-panel text-off-white">
            {opt.label}
          </option>
        ))}
      </select>
      {/* A trigger, not a persistent mode -- picking an action starts it
          (pickRef.current(), the same scripted-sequence entry point the old
          "Pick & Place" button used) and the select snaps straight back to
          the placeholder, rather than staying "selected" on an action that
          already ran. */}
      <select
        value={workflowChoice}
        onChange={(e) => {
          const choice = e.target.value;
          // A focused <select> intercepts the W/A/S/D keys that WASD drive
          // listens for as its own type-ahead ("jump to the option starting
          // with this letter") on some browsers, before the page's own
          // keydown handling ever sees them -- reported live as "WASD
          // stopped working" right after picking a workflow. Blurring hands
          // keyboard focus back to the page the instant a choice is made.
          e.target.blur();
          setWorkflowChoice("");
          if (choice === "pickupCoffee") pickRef.current();
        }}
        className={clsx(buttonClass, "appearance-none pr-8")}
      >
        <option value="" className="bg-panel text-off-white">
          Workflow…
        </option>
        <option value="pickupCoffee" className="bg-panel text-off-white">
          Pick up cup → coffee machine
        </option>
      </select>
    </>
  );

  return (
    <div className={clsx("flex h-full w-full", className)}>
      {/* min-w-0 is load-bearing here: without it a flex child won't shrink
          below its content's natural width, and the canvas column (below)
          would refuse to give up space to the guide panel instead of
          resizing -- exactly the "views overlap" failure mode this avoids. */}
      <div className="flex h-full min-w-0 flex-1 flex-col">
        <div className="relative min-h-0 flex-1 overflow-hidden">
          <div ref={containerRef} className="h-full w-full [&>canvas]:block [&>canvas]:h-full [&>canvas]:w-full" />

          {status === "loading" && (
            <div className="absolute inset-0 flex flex-col items-center justify-center gap-2 bg-black/80">
              <span className="h-2 w-2 animate-pulse rounded-full bg-sand" />
              <p className="text-technical text-xs text-sand">LOADING PHYSICS ENGINE</p>
            </div>
          )}

          {status === "error" && (
            <div className="absolute inset-0 flex flex-col items-center justify-center gap-2 bg-black/80 px-4 text-center">
              <p className="text-technical text-xs text-error">SIMULATION FAILED TO LOAD</p>
              <p className="text-xs text-text-muted">Check the browser console for details.</p>
            </div>
          )}

          {showControls && (
            <div className="absolute top-4 left-4 rounded-sm border border-border-strong bg-panel/90 px-3 py-2 text-xs text-off-white/70 backdrop-blur">
              {view === "fpp" ? (
                <>
                  <span className="text-off-white">FIRST PERSON</span> -- what the robot sees ·
                  drag to look around
                </>
              ) : view === "orbit" ? (
                <>
                  <span className="text-off-white">W A S D</span> to drive · orbit-drag to look ·{" "}
                  {showGuide ? "keys at right" : "open the key guide at right"}
                </>
              ) : (
                <>
                  <span className="text-off-white">W A S D</span> to drive · fixed camera view ·{" "}
                  {showGuide ? "keys at right" : "open the key guide at right"}
                </>
              )}
            </div>
          )}
        </div>

        {showControls && (
          <div className="flex shrink-0 items-center justify-center gap-2 border-t border-border-strong bg-panel/60 py-3">
            {controlButtons}
          </div>
        )}
      </div>

      {showControls && (
        <div className="relative shrink-0">
          {/* The arrow sits on the boundary itself (half in the canvas
              column, half in the panel) so it stays reachable whether the
              panel is open or fully collapsed to w-0 below. */}
          <button
            onClick={() => setShowGuide((v) => !v)}
            aria-label={showGuide ? "Hide key guide" : "Show key guide"}
            className="absolute top-1/2 left-0 z-10 flex h-9 w-5 -translate-x-1/2 -translate-y-1/2 items-center justify-center rounded-sm border border-border-strong bg-panel text-off-white/70 hover:bg-panel-raised cursor-pointer"
          >
            {showGuide ? "›" : "‹"}
          </button>
          <div
            className={clsx(
              "h-full overflow-hidden border-l border-border-strong bg-panel/95 backdrop-blur transition-[width] duration-200 ease-out",
              showGuide ? "w-64" : "w-0"
            )}
          >
            {/* h-full is required here, not just on the outer wrapper --
                overflow-y-auto only ever kicks in once this div's own height
                is actually bounded; without it the div just grows to fit
                every KEY_GUIDE row (now well over one screen's worth) and
                the outer `overflow-hidden` silently clips the rest instead
                of making it scrollable. */}
            <div className="h-full w-64 overflow-y-auto px-4 py-4">
              <p className="text-technical text-xs text-sand mb-4">KEY GUIDE</p>
              {KEY_GUIDE.map((group) => (
                <div key={group.section} className="mb-4 last:mb-0">
                  <p className="text-technical text-[11px] text-text-muted mb-1.5">{group.section}</p>
                  <table className="w-full text-xs">
                    <tbody>
                      {group.rows.map((row) => (
                        <tr key={row.key} className="border-b border-border last:border-0">
                          <td className="py-1.5 pr-2 text-off-white font-mono whitespace-nowrap align-top">{row.key}</td>
                          <td className="py-1.5 text-off-white/70">{row.action}</td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
