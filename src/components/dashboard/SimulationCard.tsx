"use client";

import Link from "next/link";
import { Button } from "@/components/ui/Button";

export function SimulationCard() {
  return (
    <div className="rounded-sm border border-border-strong bg-panel p-7">
      <p className="text-technical text-xs text-sand mb-3">SIMULATION</p>
      <p className="text-display text-2xl text-off-white">Test out Buildo on MuJoCo</p>
      <p className="mt-2 text-sm text-text-muted max-w-sm">
        Run Buildo in a physics simulation environment before deploying to
        hardware. Iterate on skills and training runs safely in MuJoCo.
      </p>

      {/* Simulation viewport placeholder */}
      <div className="mt-5 relative overflow-hidden rounded-sm border border-border bg-black aspect-video">
        <div className="absolute inset-0 bg-grid opacity-20" />
        <div className="absolute inset-0 flex flex-col items-center justify-center gap-3">
          <div className="flex items-center gap-2">
            <span className="h-2 w-2 rounded-full bg-sand animate-pulse" />
            <span className="text-technical text-xs text-sand">MUJOCO READY</span>
          </div>
          <p className="text-xs text-text-muted">Physics simulation environment</p>
          {/* Minimal robot wireframe indicator */}
          <div className="mt-2 flex items-end gap-1">
            {[12, 20, 28, 20, 12].map((h, i) => (
              <div
                key={i}
                className="w-1.5 rounded-sm bg-sand/30"
                style={{ height: h }}
              />
            ))}
          </div>
        </div>
      </div>

      <Link href="/dashboard/train">
        <Button className="mt-5 w-full">
          Launch simulation →
        </Button>
      </Link>
    </div>
  );
}
