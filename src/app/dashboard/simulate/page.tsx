import Link from "next/link";

import { MujocoViewer } from "@/components/simulation/MujocoViewer";

export default function SimulatePage() {
  return (
    <div className="flex h-[calc(100vh-12rem)] min-h-[560px] flex-col">
      <div className="flex items-start justify-between gap-4">
        <div>
          <p className="text-technical text-xs text-sand mb-2">SIMULATION</p>
          <h1 className="text-display text-4xl text-off-white">Buildo Simulator</h1>
          <p className="mt-3 max-w-lg text-sm text-off-white/70">
            Live physics simulation of the Buildo humanoid model. Drag to
            orbit the camera, pause to inspect a pose, or reset to drop it
            again.
          </p>
        </div>
        <Link href="/dashboard" className="shrink-0 text-sm text-off-white/60 hover:text-off-white">
          ← Overview
        </Link>
      </div>

      <div className="mt-6 min-h-0 flex-1">
        <MujocoViewer interactive className="h-full w-full rounded-sm border border-border-strong bg-black" />
      </div>
    </div>
  );
}
