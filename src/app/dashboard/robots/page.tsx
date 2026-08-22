import { Button } from "@/components/ui/Button";

export default function RobotsPage() {
  return (
    <div className="max-w-5xl">
      <p className="text-technical text-xs text-sand mb-2">FLEET</p>
      <h1 className="text-display text-4xl text-off-white">Your robots</h1>
      <p className="mt-3 max-w-lg text-sm text-off-white/70">
        Pair a Buildo unit to your account to monitor telemetry, push
        trained models, and manage installed skills remotely.
      </p>

      <div className="mt-10 rounded-sm border border-dashed border-border-strong bg-panel px-8 py-16 text-center">
        <p className="text-display text-2xl text-off-white">No robots paired yet</p>
        <p className="mx-auto mt-2 max-w-sm text-sm text-text-muted">
          Robot pairing over the SDK is coming soon. Once paired, live
          telemetry, battery, and task status will appear here.
        </p>
        <Button variant="secondary" className="mt-6" disabled>
          Pair a robot — coming soon
        </Button>
      </div>

      <div className="mt-10 opacity-40">
        <p className="text-technical text-xs text-text-muted mb-4">PREVIEW — TELEMETRY CARD</p>
        <div className="grid gap-5 sm:grid-cols-2">
          {["Buildo Unit #001", "Buildo Unit #002"].map((name) => (
            <div key={name} className="rounded-sm border border-border bg-panel p-6">
              <div className="flex items-center justify-between">
                <p className="text-display text-xl text-off-white">{name}</p>
                <span className="text-technical text-[11px] text-text-muted">OFFLINE</span>
              </div>
              <div className="mt-5 grid grid-cols-3 gap-4 text-sm">
                <div>
                  <p className="text-text-muted text-xs">Battery</p>
                  <p className="text-technical mt-1">—</p>
                </div>
                <div>
                  <p className="text-text-muted text-xs">Uptime</p>
                  <p className="text-technical mt-1">—</p>
                </div>
                <div>
                  <p className="text-text-muted text-xs">Model</p>
                  <p className="text-technical mt-1">—</p>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
