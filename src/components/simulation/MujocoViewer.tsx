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

// Scripted "wave hello" -- right-arm joint targets found by probing forward
// kinematics offline (see conversation/PR notes). This arm's own segments
// are long relative to the rest of the model -- an earlier, bigger pose
// (shoulder -0.6, elbow 0.6) put the hand ~1.7m from the body at head
// height, which read as the hand having come loose and drifted off across
// the room rather than as a raised arm. These smaller angles keep the hand
// within about a meter of the body at roughly chest height; oscillating
// the wrist (Revolute 43) still sweeps it back and forth by a clearly
// visible ~0.8m. Purely a canned demo gesture; not driven by teleop.
const WAVE_SHOULDER_TARGET = -0.3;
const WAVE_ELBOW_TARGET = 0.4;
const WAVE_WRIST_CENTER = 0.5;
const WAVE_WRIST_AMPLITUDE = 0.3;
const WAVE_WRIST_HZ = 1.5;
const WAVE_RISE_S = 0.6;
const WAVE_HOLD_S = 2.4; // oscillation duration, after the rise
const WAVE_LOWER_S = 0.6;
const WAVE_DURATION_S = WAVE_RISE_S + WAVE_HOLD_S + WAVE_LOWER_S;

function smoothstep(x: number): number {
  const t = Math.min(Math.max(x, 0), 1);
  return t * t * (3 - 2 * t);
}

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
  const waveRef = useRef<() => void>(() => {});

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
        waveElapsedS = null;
      };

      // --- Teleop + wave actuator lookups -----------------------------
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
      const ACT_WAVE_SHOULDER = actId("act_Revolute 5");
      const ACT_WAVE_ELBOW = actId("act_Revolute 11");
      const ACT_WAVE_WRIST = actId("act_Revolute 43");
      const BASE_BODY = mujoco.mj_name2id(model, OBJ_BODY, "base_link");
      const canDrive = ACT_VX >= 0 && ACT_VY >= 0 && ACT_YAW >= 0 && BASE_BODY >= 0;
      const canWave = ACT_WAVE_SHOULDER >= 0 && ACT_WAVE_ELBOW >= 0 && ACT_WAVE_WRIST >= 0;
      const canFpp = canDrive;

      const pressedKeys = new Set<string>();
      // Seconds of *simulated* time since the wave started, or null when
      // idle -- advanced once per mj_step below, never from wall-clock time.
      // A first version used performance.now() sampled once per rendered
      // frame, then applied that single ctrl value across every physics
      // sub-step the catch-up loop ran that frame. Any real hitch (a slow
      // frame, e.g. while the tab is busy elsewhere) made the fast wrist
      // oscillation's target jump discontinuously once the catch-up loop
      // resumed, and the sudden correction was enough to fling the
      // fingers' own light, weakly-actuated joints -- reproduced by
      // stalling the render loop mid-wave, never reproducible in a
      // fixed-timestep offline check since nothing there can hitch. Ticking
      // this once per physics step instead makes the wave immune to frame
      // timing by construction.
      let waveElapsedS: number | null = null;

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

        if (canWave) {
          waveRef.current = () => {
            waveElapsedS = 0;
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
        if (canDrive) {
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
            // Scripted wave: rise -> oscillate the wrist a few times ->
            // lower. Computed fresh every physics step (see waveElapsedS's
            // comment above) so it can't desync from wall-clock frame
            // timing.
            if (canWave) {
              let shoulder = 0;
              let elbow = 0;
              let wrist = 0;
              if (waveElapsedS !== null) {
                const t = waveElapsedS;
                if (t < WAVE_RISE_S) {
                  const f = smoothstep(t / WAVE_RISE_S);
                  shoulder = WAVE_SHOULDER_TARGET * f;
                  elbow = WAVE_ELBOW_TARGET * f;
                  wrist = WAVE_WRIST_CENTER * f;
                } else if (t < WAVE_RISE_S + WAVE_HOLD_S) {
                  shoulder = WAVE_SHOULDER_TARGET;
                  elbow = WAVE_ELBOW_TARGET;
                  wrist = WAVE_WRIST_CENTER + WAVE_WRIST_AMPLITUDE * Math.sin(2 * Math.PI * WAVE_WRIST_HZ * (t - WAVE_RISE_S));
                } else if (t < WAVE_DURATION_S) {
                  const f = 1 - smoothstep((t - WAVE_RISE_S - WAVE_HOLD_S) / WAVE_LOWER_S);
                  shoulder = WAVE_SHOULDER_TARGET * f;
                  elbow = WAVE_ELBOW_TARGET * f;
                  wrist = WAVE_WRIST_CENTER * f;
                } else {
                  waveElapsedS = null;
                }
              }
              data.ctrl[ACT_WAVE_SHOULDER] = shoulder;
              data.ctrl[ACT_WAVE_ELBOW] = elbow;
              data.ctrl[ACT_WAVE_WRIST] = wrist;
              if (waveElapsedS !== null) waveElapsedS += dtS;
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
      <button onClick={() => waveRef.current()} className={buttonClass}>
        Wave 👋
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
                <span className="text-off-white">W A S D</span> to drive · orbit-drag to look
              </>
            ) : (
              <>
                <span className="text-off-white">W A S D</span> to drive · fixed camera view
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
