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
// Which of base_link's own local axes points where the robot actually
// faces -- see getBodyAxisXY's comment. Confirmed empirically (not
// guessed): local X is the line straight through both hands (i.e. side to
// side), local Y is straight up, which only leaves local Z as the
// front-back axis.
const BASE_FORWARD_AXIS = 2 as const;

// Fixed "security camera" presets, one per room corner, each looking back
// toward the middle of the floor. Coordinates are three.js (Y-up), matching
// the room built in model/scripts/urdf_to_mjcf.py (ROOM_HALF_EXTENT=6) --
// inset 1m from the actual wall corners so the camera isn't clipping into
// them. Update these if that room size ever changes.
const CORNER_VIEWS: { pos: [number, number, number]; lookAt: [number, number, number] }[] = [
  { pos: [5, 2.3, 5], lookAt: [0, 0.8, 0] },
  { pos: [-5, 2.3, 5], lookAt: [0, 0.8, 0] },
  { pos: [-5, 2.3, -5], lookAt: [0, 0.8, 0] },
  { pos: [5, 2.3, -5], lookAt: [0, 0.8, 0] },
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
const ORBIT_POSITION: [number, number, number] = [5.5, 4, 5.5];
const ORBIT_TARGET: [number, number, number] = [0, 0.6, 0];

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
      scene.background = new THREE.Color(0x08090b);
      scene.add(root);
      // Even, shadowless lighting: the room is a big open floor plan now
      // (walls + furniture, see model/scripts/urdf_to_mjcf.py), and a single
      // angled key light with shadows cast by the walls used to black out
      // half of it. A mostly-overhead light plus strong ambient/hemisphere
      // fill reads like the room's own ceiling lighting instead, with no
      // shadow map at all (see renderer.shadowMap below).
      scene.add(new THREE.AmbientLight(0xffffff, 0.6));
      scene.add(new THREE.HemisphereLight(0xe8edf7, 0x3c3f45, 0.55));

      const topLight = new THREE.DirectionalLight(0xffffff, 1.7);
      topLight.position.set(2, 14, 3);
      topLight.target.position.set(0, 0.6, 0);
      topLight.castShadow = false;
      scene.add(topLight, topLight.target);

      const fillLight = new THREE.DirectionalLight(0xcfe0ff, 0.5);
      fillLight.position.set(-5, 6, -5);
      fillLight.target.position.set(0, 0.6, 0);
      fillLight.castShadow = false;
      scene.add(fillLight, fillLight.target);

      // Pulled back from the old close-up framing (2.4, 1.7, 2.4) -- that
      // was tuned for a robot alone in a small room; this one is a 12x12m
      // lab, so start with enough of it in frame to actually read as a
      // room. Orbit controls (when interactive) still zoom/pan freely.
      const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
      camera.position.set(...ORBIT_POSITION);
      camera.lookAt(...ORBIT_TARGET);

      renderer = new THREE.WebGLRenderer({ antialias: true });
      // No shadow map -- see the lighting comment above.
      renderer.shadowMap.enabled = false;
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
      const BASE_BODY = mujoco.mj_name2id(model, OBJ_BODY, "base_link");
      const canDrive = ACT_VX >= 0 && ACT_VY >= 0 && ACT_YAW >= 0 && BASE_BODY >= 0;
      const canFpp = canDrive;
      const canLift = ACT_LIFT >= 0;
      const LIFT_MIN = canLift ? model.actuator_ctrlrange[ACT_LIFT * 2] : 0;
      const LIFT_MAX = canLift ? model.actuator_ctrlrange[ACT_LIFT * 2 + 1] : 0;
      const canBend = ACT_BEND >= 0;
      // BEND_MAX clamps *below* the joint's own (more permissive) range --
      // see BEND_MAX's own comment -- so this isn't simply the actuator's
      // ctrlrange the way LIFT_MIN/MAX are.
      const BEND_MIN = -BEND_MAX;

      // --- Pick & Place lookups ---------------------------------------
      const OBJ_JOINT = mujoco.mjtObj.mjOBJ_JOINT.value;
      const OBJ_EQUALITY = mujoco.mjtObj.mjOBJ_EQUALITY.value;
      const jointId = (name: string): number => mujoco.mj_name2id(model, OBJ_JOINT, name);
      const ACT_PICK_ARM = PICK_ARM_JOINTS.map(actId);
      const ACT_PICK_FINGERS = PICK_FINGER_JOINTS.map(actId);
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
        const onKeyDown = (e: KeyboardEvent) => pressedKeys.add(e.key.toLowerCase());
        const onKeyUp = (e: KeyboardEvent) => pressedKeys.delete(e.key.toLowerCase());
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
          renderer?.domElement.removeEventListener("pointerdown", onPointerDown);
          window.removeEventListener("pointermove", onPointerMove);
          window.removeEventListener("pointerup", onPointerUp);
        };

        if (canPick) {
          pickRef.current = () => {
            if (pickPhase === "idle") {
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
      <button onClick={() => pickRef.current()} className={buttonClass}>
        Pick & Place 📦
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
    <div className={clsx("flex h-full w-full flex-col", className)}>
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
                <span className="text-off-white">W A S D</span> to drive · <span className="text-off-white">U / L</span> height ·{" "}
                <span className="text-off-white">F / R</span> bend · orbit-drag to look
              </>
            ) : (
              <>
                <span className="text-off-white">W A S D</span> to drive · <span className="text-off-white">U / L</span> height ·{" "}
                <span className="text-off-white">F / R</span> bend · fixed camera view
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
  );
}
