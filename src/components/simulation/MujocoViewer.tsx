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
// a weld (see MANUAL_GRIP_ANCHOR_BODY_RIGHT/LEFT and
// grasp_weld_manual_right/left's own comment in
// model/scripts/urdf_to_mjcf.py for why this needs its own pair of welds,
// computed live, rather than reusing Pick & Place's own baked one) --
// finger-contact alone doesn't reliably hold anything with this hand (the
// thumb chain doesn't oppose the other four fingers regardless of joint
// values, see PICK_FINGER_JOINTS' own comment), so without a weld a
// "grabbed" object would just slide free the moment the arm moved.
const GRIP_RATE = 1.2; // 0..1 (open..closed) per second while held
const GRIP_ENGAGE_AT = 0.8; // grip fraction above which closing near the object grabs it
const GRIP_RELEASE_AT = 0.2; // grip fraction below which it lets go
const GRIP_RADIUS = 0.2; // m -- how close the grip anchor must be to the object to grab it (loose: there's no on-screen distance readout, so manual driving/positioning is imprecise)
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
// Height of the head's own mesh above base_link, measured directly (not
// guessed): the head *body*'s own xpos is not usable here -- like several
// other links from this CAD export, face_cover_3_1's body origin sits
// nearly 1.3m away from where its mesh actually renders (a large local
// origin-vs-geometry offset baked into the raw export), landing camera
// position calculations that used it up near/above the room's own ceiling
// height. That's exactly why FPP previously showed mostly wall-tops and
// ceiling void. base_link doesn't have this problem (verified: its body
// origin and its geometry agree), so FPP is mounted a fixed, measured
// height above *that* instead.
const FPP_EYE_HEIGHT = 1.5; // m above base_link
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

// wxyz quaternion helpers for the manual-grip weld's relpose, computed live
// -- same math as urdf_to_mjcf.py's own _quat_mul()/quat_conj(), used there
// to bake grasp_weld's relpose offline; needed here at runtime instead
// because a manual grab can happen from any pose, not one fixed one (see
// GRIP_RATE's own comment).
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
// eslint-disable-next-line @typescript-eslint/no-explicit-any
function computeGripRelpose(data: any, anchorBody: number, objBody: number) {
  const ax = data.xpos[anchorBody * 3];
  const ay = data.xpos[anchorBody * 3 + 1];
  const az = data.xpos[anchorBody * 3 + 2];
  const dx = data.xpos[objBody * 3] - ax;
  const dy = data.xpos[objBody * 3 + 1] - ay;
  const dz = data.xpos[objBody * 3 + 2] - az;
  const m = anchorBody * 9;
  // R^T * d: R is row-major, so column i is [R[i], R[3+i], R[6+i]], and
  // (R^T*d)'s row i is that column dotted with d.
  const relPos: [number, number, number] = [
    data.xmat[m] * dx + data.xmat[m + 3] * dy + data.xmat[m + 6] * dz,
    data.xmat[m + 1] * dx + data.xmat[m + 4] * dy + data.xmat[m + 7] * dz,
    data.xmat[m + 2] * dx + data.xmat[m + 5] * dy + data.xmat[m + 8] * dz,
  ];
  const q = anchorBody * 4;
  const anchorQuat: [number, number, number, number] = [data.xquat[q], data.xquat[q + 1], data.xquat[q + 2], data.xquat[q + 3]];
  const oq = objBody * 4;
  const objQuat: [number, number, number, number] = [data.xquat[oq], data.xquat[oq + 1], data.xquat[oq + 2], data.xquat[oq + 3]];
  const relQuat = quatMul(quatConj(anchorQuat), objQuat);
  return { relPos, relQuat };
}

// Scripted "Pick & Place" demo: drives to bench_n1 (where urdf_to_mjcf.py's
// _build_pickup_object() spawns a small box), grabs it with the same right
// arm/hand the wave gesture uses, carries it to bench_n2, and sets it down.
// The six arm joint targets below were solved offline via numeric IK against
// this same model (see model/scripts/urdf_to_mjcf.py's matching PICK_*
// constants and _build_grasp_weld()'s own comment for the full derivation
// and for why holding the object is a weld constraint, not finger contact
// alone) -- this file's PICK_ARM_JOINTS/PICK_GRASP_QPOS/PICK_PREGRASP_QPOS
// must stay byte-for-byte in sync with that script's copies, or the reach
// will miss where the object actually is and/or land somewhere the
// pre-baked weld offset doesn't match.
const PICK_ARM_JOINTS = ["Revolute 5", "Revolute 7", "Revolute 9", "Revolute 11", "Revolute 26", "Revolute 43"];
const PICK_REST_QPOS = [0, 0, 0, 0, 0, 0];
const PICK_PREGRASP_QPOS = [-0.5884, 0.8807, -0.1608, 0.0012, -0.1425, 0.3047];
const PICK_GRASP_QPOS = [-0.5666, 0.9047, -0.1661, -0.0047, -0.1486, 0.1785];
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
// PICK_GRASP_QPOS to actually reach the object -- see
// model/scripts/urdf_to_mjcf.py's PICK_PARK_POSE for the full derivation.
// bench_n2's park spot is the same point mirrored in x (the two benches are
// otherwise identical and identically-facing, see _build_room()).
const PICK_PARK_PICK: [number, number] = [-3.0, 4.1];
const PICK_PARK_PLACE: [number, number] = [3.0, 4.1];
const PICK_PARK_YAW = Math.PI; // faces +Y (north), toward either bench

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
const PICK_YAW_KP = 3.0;
const PICK_YAW_MAX_RATE = 1.5; // rad/s

// Phase durations, in seconds of simulated time -- ticked once per physics
// step inside the same catch-up loop as the wave gesture, for the same
// reason (see waveElapsedS's own comment: immune to render-frame hitches by
// construction, since nothing here reads wall-clock time).
const PICK_REACH_S = 1.2; // rest -> pregrasp
const PICK_LOWER_S = 0.9; // pregrasp -> grasp
const PICK_CLOSE_S = 0.7; // fingers open -> closed
const PICK_SETTLE_S = 0.3; // brief pause right after the weld grabs, before lifting
const PICK_LIFT_S = 0.9; // grasp -> pregrasp, now holding the object
const PICK_RELEASE_S = 0.7; // fingers closed -> open
const PICK_RETRACT_S = 1.2; // grasp -> rest

type PickPhase =
  | "idle"
  | "driveToPick"
  | "reach"
  | "lower"
  | "close"
  | "settle"
  | "lift"
  | "driveToPlace"
  | "lowerPlace"
  | "release"
  | "retract";

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
      const canFpp = canDrive;
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
      const OBJ_EQUALITY = mujoco.mjtObj.mjOBJ_EQUALITY.value;
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
      const YAW_JOINT = jointId("virtual_base_yaw");
      const YAW_QPOS_ADR = YAW_JOINT >= 0 ? model.jnt_qposadr[YAW_JOINT] : -1;
      const EQ_GRASP_WELD = mujoco.mj_name2id(model, OBJ_EQUALITY, "grasp_weld");
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
      const RIGHT_GRIP_ANCHOR_BODY = mujoco.mj_name2id(model, OBJ_BODY, "finger_tip_1");
      const LEFT_GRIP_ANCHOR_BODY = mujoco.mj_name2id(model, OBJ_BODY, "finger_tip_3");
      const EQ_GRIP_RIGHT = mujoco.mj_name2id(model, OBJ_EQUALITY, "grasp_weld_manual_right");
      const EQ_GRIP_LEFT = mujoco.mj_name2id(model, OBJ_EQUALITY, "grasp_weld_manual_left");
      const canGripRight =
        ACT_PICK_FINGERS.every((i) => i >= 0) &&
        PICKUP_OBJECT_BODY >= 0 &&
        RIGHT_GRIP_ANCHOR_BODY >= 0 &&
        EQ_GRIP_RIGHT >= 0 &&
        PICKUP_OBJECT_DOF >= 0;
      const canGripLeft =
        ACT_LEFT_FINGERS.every((i) => i >= 0) &&
        PICKUP_OBJECT_BODY >= 0 &&
        LEFT_GRIP_ANCHOR_BODY >= 0 &&
        EQ_GRIP_LEFT >= 0 &&
        PICKUP_OBJECT_DOF >= 0;
      const canPick =
        canDrive &&
        YAW_QPOS_ADR >= 0 &&
        EQ_GRASP_WELD >= 0 &&
        PICKUP_OBJECT_DOF >= 0 &&
        ACT_PICK_ARM.every((i) => i >= 0) &&
        ACT_PICK_FINGERS.every((i) => i >= 0);
      if (!canPick) {
        // The "Pick & Place" button is always rendered (see the wave
        // button's own precedent) and silently no-ops if unsupported --
        // fine for an old build missing the feature entirely, but
        // indistinguishable from "the button does nothing" if the loaded
        // scene *should* support it and one lookup below just failed (e.g.
        // a stale cached /mujoco/scene/humanoid.xml missing the object/weld
        // a newer build added). Logged once at load time so that's not a
        // silent dead end to debug from the user's report alone.
        console.warn("[MujocoViewer] Pick & Place unavailable in this scene:", {
          canDrive,
          YAW_QPOS_ADR,
          EQ_GRASP_WELD,
          PICKUP_OBJECT_DOF,
          missingArmActuators: PICK_ARM_JOINTS.filter((_, i) => ACT_PICK_ARM[i] < 0),
          missingFingerActuators: PICK_FINGER_JOINTS.filter((_, i) => ACT_PICK_FINGERS[i] < 0),
        });
      }

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
              // would otherwise get dragged along through this whole
              // sequence attached to whichever hand grabbed it, fighting
              // Pick & Place's own separate weld on the same object.
              if (rightGripEngaged && EQ_GRIP_RIGHT >= 0) {
                data.eq_active[EQ_GRIP_RIGHT] = 0;
                rightGripEngaged = false;
              }
              if (leftGripEngaged && EQ_GRIP_LEFT >= 0) {
                data.eq_active[EQ_GRIP_LEFT] = 0;
                leftGripEngaged = false;
              }
              pickPhase = "driveToPick";
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
            // handles engaging/releasing the weld on crossing
            // GRIP_ENGAGE_AT/GRIP_RELEASE_AT (see that constant's own
            // comment) -- the finger motion itself always runs regardless of
            // whether anything is actually in reach to grab.
            if (canGripRight && pickPhase === "idle") {
              const input = (keyR("c") ? 1 : 0) - (keyR("x") ? 1 : 0);
              rightGripFraction = Math.max(0, Math.min(1, rightGripFraction + input * GRIP_RATE * dtS));
              const fingerTarget = lerpArr(PICK_FINGER_OPEN, PICK_FINGER_CLOSED, rightGripFraction);
              for (let i = 0; i < ACT_PICK_FINGERS.length; i++) data.ctrl[ACT_PICK_FINGERS[i]] = fingerTarget[i];

              const dist = Math.hypot(
                data.xpos[RIGHT_GRIP_ANCHOR_BODY * 3] - data.xpos[PICKUP_OBJECT_BODY * 3],
                data.xpos[RIGHT_GRIP_ANCHOR_BODY * 3 + 1] - data.xpos[PICKUP_OBJECT_BODY * 3 + 1],
                data.xpos[RIGHT_GRIP_ANCHOR_BODY * 3 + 2] - data.xpos[PICKUP_OBJECT_BODY * 3 + 2]
              );
              if (!rightGripEngaged && !leftGripEngaged && rightGripFraction > GRIP_ENGAGE_AT && dist < GRIP_RADIUS) {
                const { relPos, relQuat } = computeGripRelpose(data, RIGHT_GRIP_ANCHOR_BODY, PICKUP_OBJECT_BODY);
                model.eq_data[EQ_GRIP_RIGHT * 11 + 3] = relPos[0];
                model.eq_data[EQ_GRIP_RIGHT * 11 + 4] = relPos[1];
                model.eq_data[EQ_GRIP_RIGHT * 11 + 5] = relPos[2];
                model.eq_data[EQ_GRIP_RIGHT * 11 + 6] = relQuat[0];
                model.eq_data[EQ_GRIP_RIGHT * 11 + 7] = relQuat[1];
                model.eq_data[EQ_GRIP_RIGHT * 11 + 8] = relQuat[2];
                model.eq_data[EQ_GRIP_RIGHT * 11 + 9] = relQuat[3];
                data.eq_active[EQ_GRIP_RIGHT] = 1;
                for (let k = 0; k < 6; k++) data.qvel[PICKUP_OBJECT_DOF + k] = 0;
                rightGripEngaged = true;
              } else if (rightGripEngaged && rightGripFraction < GRIP_RELEASE_AT) {
                data.eq_active[EQ_GRIP_RIGHT] = 0;
                rightGripEngaged = false;
              }
            }
            if (canGripLeft && pickPhase === "idle") {
              const input = (keyL("[") ? 1 : 0) - (keyL("]") ? 1 : 0);
              leftGripFraction = Math.max(0, Math.min(1, leftGripFraction + input * GRIP_RATE * dtS));
              const fingerTarget = lerpArr(LEFT_FINGER_OPEN, LEFT_FINGER_CLOSED, leftGripFraction);
              for (let i = 0; i < ACT_LEFT_FINGERS.length; i++) data.ctrl[ACT_LEFT_FINGERS[i]] = fingerTarget[i];

              const dist = Math.hypot(
                data.xpos[LEFT_GRIP_ANCHOR_BODY * 3] - data.xpos[PICKUP_OBJECT_BODY * 3],
                data.xpos[LEFT_GRIP_ANCHOR_BODY * 3 + 1] - data.xpos[PICKUP_OBJECT_BODY * 3 + 1],
                data.xpos[LEFT_GRIP_ANCHOR_BODY * 3 + 2] - data.xpos[PICKUP_OBJECT_BODY * 3 + 2]
              );
              if (!leftGripEngaged && !rightGripEngaged && leftGripFraction > GRIP_ENGAGE_AT && dist < GRIP_RADIUS) {
                const { relPos, relQuat } = computeGripRelpose(data, LEFT_GRIP_ANCHOR_BODY, PICKUP_OBJECT_BODY);
                model.eq_data[EQ_GRIP_LEFT * 11 + 3] = relPos[0];
                model.eq_data[EQ_GRIP_LEFT * 11 + 4] = relPos[1];
                model.eq_data[EQ_GRIP_LEFT * 11 + 5] = relPos[2];
                model.eq_data[EQ_GRIP_LEFT * 11 + 6] = relQuat[0];
                model.eq_data[EQ_GRIP_LEFT * 11 + 7] = relQuat[1];
                model.eq_data[EQ_GRIP_LEFT * 11 + 8] = relQuat[2];
                model.eq_data[EQ_GRIP_LEFT * 11 + 9] = relQuat[3];
                data.eq_active[EQ_GRIP_LEFT] = 1;
                for (let k = 0; k < 6; k++) data.qvel[PICKUP_OBJECT_DOF + k] = 0;
                leftGripEngaged = true;
              } else if (leftGripEngaged && leftGripFraction < GRIP_RELEASE_AT) {
                data.eq_active[EQ_GRIP_LEFT] = 0;
                leftGripEngaged = false;
              }
            }

            // Scripted Pick & Place: see the PICK_* constants' own comments
            // for the phase list, timings, and the offline-solved arm
            // targets. Advances pickPhaseElapsedS the same way waveElapsedS
            // advances above, for the same frame-hitch-immunity reason.
            if (canPick && pickPhase !== "idle") {
              // Yaw hold, active through every phase -- see PICK_YAW_KP's
              // own comment on why this can't be left uncontrolled.
              const yawErr = Math.atan2(Math.sin(PICK_PARK_YAW - data.qpos[YAW_QPOS_ADR]), Math.cos(PICK_PARK_YAW - data.qpos[YAW_QPOS_ADR]));
              data.ctrl[ACT_YAW] = Math.max(-PICK_YAW_MAX_RATE, Math.min(PICK_YAW_MAX_RATE, PICK_YAW_KP * yawErr));

              const t = pickPhaseElapsedS;
              let armTarget = PICK_REST_QPOS;
              let fingerTarget = PICK_FINGER_OPEN;
              let driveTarget: [number, number] | null = null;

              if (pickPhase === "driveToPick") {
                driveTarget = PICK_PARK_PICK;
              } else if (pickPhase === "reach") {
                armTarget = lerpArr(PICK_REST_QPOS, PICK_PREGRASP_QPOS, smoothstep(t / PICK_REACH_S));
                if (t >= PICK_REACH_S) {
                  pickPhase = "lower";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "lower") {
                armTarget = lerpArr(PICK_PREGRASP_QPOS, PICK_GRASP_QPOS, smoothstep(t / PICK_LOWER_S));
                if (t >= PICK_LOWER_S) {
                  pickPhase = "close";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "close") {
                armTarget = PICK_GRASP_QPOS;
                fingerTarget = lerpArr(PICK_FINGER_OPEN, PICK_FINGER_CLOSED, smoothstep(t / PICK_CLOSE_S));
                if (t >= PICK_CLOSE_S) {
                  pickPhase = "settle";
                  pickPhaseElapsedS = 0;
                  // The instant the fingers finish closing: engage the
                  // weld (see _build_grasp_weld()'s comment for why this
                  // is a weld, not finger-contact friction) and clear any
                  // residual velocity the object's own free joint picked
                  // up while just sitting there, so it doesn't carry that
                  // into the constraint at the moment it engages.
                  data.eq_active[EQ_GRASP_WELD] = 1;
                  for (let k = 0; k < 6; k++) data.qvel[PICKUP_OBJECT_DOF + k] = 0;
                }
              } else if (pickPhase === "settle") {
                armTarget = PICK_GRASP_QPOS;
                fingerTarget = PICK_FINGER_CLOSED;
                if (t >= PICK_SETTLE_S) {
                  pickPhase = "lift";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "lift") {
                armTarget = lerpArr(PICK_GRASP_QPOS, PICK_PREGRASP_QPOS, smoothstep(t / PICK_LIFT_S));
                fingerTarget = PICK_FINGER_CLOSED;
                if (t >= PICK_LIFT_S) {
                  pickPhase = "driveToPlace";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "driveToPlace") {
                armTarget = PICK_PREGRASP_QPOS;
                fingerTarget = PICK_FINGER_CLOSED;
                driveTarget = PICK_PARK_PLACE;
              } else if (pickPhase === "lowerPlace") {
                armTarget = lerpArr(PICK_PREGRASP_QPOS, PICK_GRASP_QPOS, smoothstep(t / PICK_LOWER_S));
                fingerTarget = PICK_FINGER_CLOSED;
                if (t >= PICK_LOWER_S) {
                  pickPhase = "release";
                  pickPhaseElapsedS = 0;
                  data.eq_active[EQ_GRASP_WELD] = 0;
                }
              } else if (pickPhase === "release") {
                armTarget = PICK_GRASP_QPOS;
                fingerTarget = lerpArr(PICK_FINGER_CLOSED, PICK_FINGER_OPEN, smoothstep(t / PICK_RELEASE_S));
                if (t >= PICK_RELEASE_S) {
                  pickPhase = "retract";
                  pickPhaseElapsedS = 0;
                }
              } else if (pickPhase === "retract") {
                armTarget = lerpArr(PICK_GRASP_QPOS, PICK_REST_QPOS, smoothstep(t / PICK_RETRACT_S));
                fingerTarget = PICK_FINGER_OPEN;
                if (t >= PICK_RETRACT_S) {
                  pickPhase = "idle";
                  pickPhaseElapsedS = 0;
                  // The arm is now physically at PICK_REST_QPOS (all
                  // zeros) -- sync the teleop targets to match so
                  // T/G/Y/H/I/K/O/P/J/N/V/B resume from where the arm
                  // actually is instead of jumping from a stale
                  // pre-sequence value.
                  shoulderFwdAngle = 0;
                  shoulderLatAngle = 0;
                  armRollAngle = 0;
                  elbowAngle = 0;
                  wristAAngle = 0;
                  wristBAngle = 0;
                }
              }

              if (driveTarget) {
                const ex = driveTarget[0] - data.xpos[BASE_BODY * 3 + 0];
                const ey = driveTarget[1] - data.xpos[BASE_BODY * 3 + 1];
                const dist = Math.hypot(ex, ey);
                if (dist < PICK_DRIVE_ARRIVE_DIST) {
                  pickDriveVX = 0;
                  pickDriveVY = 0;
                  pickPhase = pickPhase === "driveToPick" ? "reach" : "lowerPlace";
                  pickPhaseElapsedS = 0;
                } else {
                  const desiredVX = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_DRIVE_KP * ex));
                  const desiredVY = Math.max(-PICK_DRIVE_MAX_SPEED, Math.min(PICK_DRIVE_MAX_SPEED, PICK_DRIVE_KP * ey));
                  const maxDelta = PICK_DRIVE_MAX_ACCEL * dtS;
                  pickDriveVX += Math.max(-maxDelta, Math.min(maxDelta, desiredVX - pickDriveVX));
                  pickDriveVY += Math.max(-maxDelta, Math.min(maxDelta, desiredVY - pickDriveVY));
                }
              } else {
                pickDriveVX = 0;
                pickDriveVY = 0;
              }
              data.ctrl[ACT_VX] = pickDriveVX;
              data.ctrl[ACT_VY] = pickDriveVY;

              for (let i = 0; i < ACT_PICK_ARM.length; i++) data.ctrl[ACT_PICK_ARM[i]] = armTarget[i];
              for (let i = 0; i < ACT_PICK_FINGERS.length; i++) data.ctrl[ACT_PICK_FINGERS[i]] = fingerTarget[i];
              // Every PICK_*_QPOS arm target was solved at LIFT_MIN with the
              // waist straight -- force both regardless of whatever the user
              // last set with U/L/F/R.
              if (canLift) {
                liftHeight = LIFT_MIN;
                data.ctrl[ACT_LIFT] = LIFT_MIN;
              }
              if (canBend) {
                bendAngle = 0;
                data.ctrl[ACT_BEND] = 0;
              }

              pickPhaseElapsedS += dtS;
            }

            mujoco.mj_step(model, data);
            remaining -= timestepMs;
          }
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
          // FPP: camera mounted a fixed, measured height above base_link
          // (see FPP_EYE_HEIGHT -- not the head body's own origin, which
          // this CAD export doesn't place at the head's actual geometry).
          // Base direction is "the direction WASD's W currently drives"
          // (same getBodyAxisXY this frame's teleop above used), turned
          // further by the head look-around drag (fppYaw/fppPitch, see the
          // pointer handlers above) so it's not just a fixed forward-only
          // view anymore. Built as an explicit (yaw, pitch) -> direction,
          // three.js's usual FPS-camera convention, rather than rotating
          // the forward vector by hand -- much harder to get a sign wrong.
          const base = bodies[BASE_BODY];
          if (base) {
            const [fx, fy] = getBodyAxisXY(data.xmat, BASE_BODY, BASE_FORWARD_AXIS);
            const baseYaw = Math.atan2(fx, -fy); // three.js horizontal forward (fx, -fy) -> angle
            const totalYaw = baseYaw + fppYaw;
            const cosPitch = Math.cos(fppPitch);
            fppForward.set(Math.sin(totalYaw) * cosPitch, Math.sin(fppPitch), Math.cos(totalYaw) * cosPitch);
            camera.position.copy(base.position);
            camera.position.y += FPP_EYE_HEIGHT;
            camera.position.addScaledVector(fppForward, 0.3);
            fppLookAt.copy(camera.position).add(fppForward);
            camera.lookAt(fppLookAt);
          }
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
