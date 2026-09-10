import Image from "next/image";
import Link from "next/link";
import { Button } from "@/components/ui/Button";

// Static preview instead of a live MujocoViewer embed: the overview page
// doesn't need a running physics simulation (and three.js canvas + a
// ResizeObserver-driven renderer.setSize() sitting inside a plain
// aspect-ratio box, with no fixed pixel height anywhere in the chain, was
// exactly the setup that produces a classic resize-observer feedback loop
// -- the card kept growing taller for as long as the page sat open). The
// real, interactive simulator is one click away either way.
export function SimulationCard() {
  return (
    <div className="rounded-sm border border-border-strong bg-panel p-7">
      <p className="text-technical text-xs text-sand mb-3">SIMULATION</p>
      <p className="text-display text-2xl text-off-white">Test out Buildo on MuJoCo</p>
      <p className="mt-2 text-sm text-text-muted max-w-sm">
        Run Buildo in a physics simulation environment before deploying to
        hardware. Iterate on skills and training runs safely in MuJoCo.
      </p>

      <div className="relative mt-5 aspect-video overflow-hidden rounded-sm border border-border bg-black">
        <Image
          src="/images/simulator-preview.png"
          alt="Buildo humanoid robot standing in the MuJoCo lab simulation"
          fill
          priority
          className="object-cover"
          sizes="(min-width: 1024px) 40vw, 90vw"
        />
      </div>

      <Link href="/dashboard/simulate">
        <Button className="mt-5 w-full">
          Launch simulation →
        </Button>
      </Link>
    </div>
  );
}
