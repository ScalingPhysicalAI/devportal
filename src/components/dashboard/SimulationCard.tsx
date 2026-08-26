"use client";

import Link from "next/link";
import { Button } from "@/components/ui/Button";
import { MujocoViewer } from "@/components/simulation/MujocoViewer";

export function SimulationCard() {
  return (
    <div className="rounded-sm border border-border-strong bg-panel p-7">
      <p className="text-technical text-xs text-sand mb-3">SIMULATION</p>
      <p className="text-display text-2xl text-off-white">Test out Buildo on MuJoCo</p>
      <p className="mt-2 text-sm text-text-muted max-w-sm">
        Run Buildo in a physics simulation environment before deploying to
        hardware. Iterate on skills and training runs safely in MuJoCo.
      </p>

      <MujocoViewer className="mt-5 rounded-sm border border-border bg-black aspect-video" />

      <Link href="/dashboard/simulate">
        <Button className="mt-5 w-full">
          Launch simulation →
        </Button>
      </Link>
    </div>
  );
}
