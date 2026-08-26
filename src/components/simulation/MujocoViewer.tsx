"use client";

import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import clsx from "clsx";

import { buildSceneFromModel, getPosition, getQuaternion, loadHumanoidModel } from "@/lib/mujoco/loadHumanoidScene";

// The MuJoCo WASM engine (mujoco.js + mujoco.wasm) is served as a static
// asset from public/mujoco/ rather than bundled -- it's a precompiled
// Emscripten module meant to be fetched at its own URL, and mujoco.js
// resolves its .wasm relative to that URL by default. See
// scripts/sync-mujoco-assets.sh for how these files get there.
const ENGINE_URL = "/mujoco/engine/mujoco.js";

interface MujocoViewerProps {
  /** Enables orbit camera controls and the pause/reset overlay. */
  interactive?: boolean;
  className?: string;
}

type Status = "loading" | "ready" | "error";

export function MujocoViewer({ interactive = false, className }: MujocoViewerProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [status, setStatus] = useState<Status>("loading");
  const [paused, setPaused] = useState(false);
  const pausedRef = useRef(paused);
  const resetRef = useRef<() => void>(() => {});

  useEffect(() => {
    pausedRef.current = paused;
  }, [paused]);

  useEffect(() => {
    let disposed = false;
    let renderer: THREE.WebGLRenderer | null = null;
    let controls: OrbitControls | null = null;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    let model: any = null;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    let data: any = null;
    let resizeObserver: ResizeObserver | null = null;

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
      // Kept deliberately low -- this is what was flattening the model's
      // black material to grey. Contrast comes from the key light + shadow
      // below, not from flat ambient fill.
      scene.add(new THREE.AmbientLight(0xffffff, 0.12));
      scene.add(new THREE.HemisphereLight(0x8fa6c2, 0x0a0a0a, 0.15));

      const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
      keyLight.position.set(3, 5, 2.5);
      keyLight.target.position.set(0, 0.6, 0);
      keyLight.castShadow = true;
      keyLight.shadow.mapSize.set(2048, 2048);
      keyLight.shadow.camera.near = 0.5;
      keyLight.shadow.camera.far = 12;
      keyLight.shadow.camera.left = -2.5;
      keyLight.shadow.camera.right = 2.5;
      keyLight.shadow.camera.top = 2.5;
      keyLight.shadow.camera.bottom = -2.5;
      keyLight.shadow.bias = -0.0015;
      keyLight.shadow.radius = 1.5;
      scene.add(keyLight, keyLight.target);

      // Faint fill from the opposite side -- just enough that the shadowed
      // half of the model isn't a pure silhouette, without washing out the
      // key light's contrast or the black material.
      const fillLight = new THREE.DirectionalLight(0xcfe0ff, 0.18);
      fillLight.position.set(-3, 2, -2.5);
      fillLight.target.position.set(0, 0.6, 0);
      scene.add(fillLight, fillLight.target);

      const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
      camera.position.set(2.4, 1.7, 2.4);
      camera.lookAt(0, 0.6, 0);

      renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.shadowMap.enabled = true;
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
        controls.target.set(0, 0.6, 0);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        controls.update();
      }

      resetRef.current = () => {
        mujoco.mj_resetData(model, data);
        mujoco.mj_forward(model, data);
      };

      const tmpPos = new THREE.Vector3();
      const tmpQuat = new THREE.Quaternion();
      const timestepMs = model.opt.timestep * 1000;
      let lastFrameTime: number | null = null;

      renderer.setAnimationLoop((now: number) => {
        if (disposed || !renderer) return;
        controls?.update();

        if (!pausedRef.current) {
          if (lastFrameTime === null) lastFrameTime = now;
          // Cap physics catch-up so a slow/backgrounded tab doesn't try to
          // fast-forward the simulation into instability once it resumes.
          let remaining = Math.min(now - lastFrameTime, 50);
          lastFrameTime = now;
          while (remaining > 0) {
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

  return (
    <div className={clsx("relative overflow-hidden", className)}>
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

      {interactive && status === "ready" && (
        <div className="absolute bottom-4 left-1/2 flex -translate-x-1/2 items-center gap-2">
          <button
            onClick={() => setPaused((p) => !p)}
            className="rounded-sm border border-border-strong bg-panel/90 px-4 py-2 text-xs text-off-white backdrop-blur transition-colors hover:bg-panel-raised cursor-pointer"
          >
            {paused ? "Resume" : "Pause"}
          </button>
          <button
            onClick={() => resetRef.current()}
            className="rounded-sm border border-border-strong bg-panel/90 px-4 py-2 text-xs text-off-white backdrop-blur transition-colors hover:bg-panel-raised cursor-pointer"
          >
            Reset
          </button>
        </div>
      )}
    </div>
  );
}
