import Link from "next/link";

import { Button } from "@/components/ui/Button";
import { Container } from "@/components/ui/Container";
import { RigGraphic } from "@/components/graphics/RigGraphic";

const stats = [
  { label: "AI stack layers", value: "3" },
  { label: "Reasoning rate", value: "100 Hz" },
  { label: "Signup reward", value: "20 SFT" },
];

const platformFeatures = [
  {
    tag: "01",
    title: "Train",
    body: "Start from a pre-trained foundation model, or fine-tune Buildo on a dataset you collect yourself with the SDK.",
    href: "/dashboard/train",
  },
  {
    tag: "02",
    title: "Compute",
    body: "Rent GPU time from Starforge on demand — from a single RTX 4090 to a full A100/H100 training run.",
    href: "/dashboard/gpu",
  },
  {
    tag: "03",
    title: "Skills",
    body: "Buy pre-built skills — pick-and-place, navigation, manipulation — and load them straight onto your robot.",
    href: "/dashboard/skills",
  },
  {
    tag: "04",
    title: "Control",
    body: "Monitor telemetry and manage every registered unit from a single developer dashboard.",
    href: "/dashboard/robots",
  },
];

const stack = [
  {
    layer: "System 2",
    rate: "~1 Hz",
    desc: "Vision-language-action model for high-level reasoning and task planning.",
  },
  {
    layer: "System 1",
    rate: "~10 Hz",
    desc: "On-board foundation model handling real-time motion planning.",
  },
  {
    layer: "System 0",
    rate: "~100 Hz",
    desc: "Microcontroller-level fine-motor control and tactile feedback.",
  },
];

export default function LandingPage() {
  return (
    <>
      {/* Hero */}
      <section className="relative overflow-hidden bg-grid">
        <div className="pointer-events-none absolute -top-40 left-1/2 h-[500px] w-[900px] -translate-x-1/2 rounded-full bg-sand/10 blur-[120px]" />
        <Container className="relative pt-24 pb-20 sm:pt-32 sm:pb-28">
          <div className="grid gap-16 lg:grid-cols-2 lg:items-center">
            <div className="animate-fade-up">
              <p className="text-technical text-xs text-sand mb-5">
                STARFORGE // DEVELOPER PORTAL
              </p>
              <h1 className="text-display text-5xl sm:text-6xl lg:text-7xl leading-[0.95] text-off-white">
                Program the robot.
                <br />
                <span className="text-sand">Own the upside.</span>
              </h1>
              <p className="mt-6 max-w-md text-base leading-relaxed text-off-white/70">
                The developer platform for Buildo — Starforge&apos;s Physical
                AI robot. Train it on your own data, rent GPU compute, buy
                skills, and get rewarded in STARFORGE tokens for building
                early.
              </p>
              <div className="mt-9 flex flex-wrap items-center gap-4">
                <Link href="/signup">
                  <Button size="lg">Sign up as a developer</Button>
                </Link>
                <a
                  href="https://starforgerobotics.com/buildo/"
                  target="_blank"
                  rel="noreferrer"
                >
                  <Button variant="secondary" size="lg">
                    View Buildo
                  </Button>
                </a>
              </div>

              <div className="mt-14 grid grid-cols-3 gap-6 max-w-md border-t border-border pt-6">
                {stats.map((s) => (
                  <div key={s.label}>
                    <p className="text-technical text-xl text-sand">{s.value}</p>
                    <p className="mt-1 text-xs text-text-muted">{s.label}</p>
                  </div>
                ))}
              </div>
            </div>

            <div className="relative mx-auto aspect-square w-full max-w-md animate-fade-up [animation-delay:120ms]">
              <RigGraphic className="h-full w-full" />
            </div>
          </div>
        </Container>
      </section>

      {/* Buildo / three-layer stack */}
      <section id="buildo" className="border-t border-border py-24">
        <Container>
          <div className="grid gap-14 lg:grid-cols-[1fr_1.2fr] lg:items-start">
            <div>
              <p className="text-technical text-xs text-sand mb-4">BUILDO ROBOT</p>
              <h2 className="text-display text-4xl text-off-white leading-tight">
                One robot.
                <br />
                Three layers of intelligence.
              </h2>
              <p className="mt-5 text-sm leading-relaxed text-off-white/70 max-w-sm">
                Buildo pairs a wearable dexterous hand and open FOC firmware
                with a hierarchical AI architecture — built for developers
                and researchers who want to own the full stack, not just call
                an API.
              </p>
              <a
                href="https://starforgerobotics.com/buildo/"
                target="_blank"
                rel="noreferrer"
                className="mt-6 inline-block text-sm text-sand hover:underline"
              >
                Read the full spec →
              </a>
            </div>

            <div className="space-y-px overflow-hidden rounded-sm border border-border">
              {stack.map((s) => (
                <div
                  key={s.layer}
                  className="flex flex-col gap-1 bg-panel px-6 py-6 sm:flex-row sm:items-baseline sm:justify-between"
                >
                  <div>
                    <p className="text-display text-xl text-off-white">{s.layer}</p>
                    <p className="mt-1 text-sm text-text-muted max-w-sm">{s.desc}</p>
                  </div>
                  <p className="text-technical text-sm text-sand shrink-0">{s.rate}</p>
                </div>
              ))}
            </div>
          </div>
        </Container>
      </section>

      {/* Platform features */}
      <section id="platform" className="border-t border-border bg-panel py-24">
        <Container>
          <p className="text-technical text-xs text-sand mb-4">THE PLATFORM</p>
          <h2 className="text-display text-4xl text-off-white max-w-lg leading-tight">
            Everything to take Buildo from unboxed to deployed.
          </h2>

          <div className="mt-14 grid gap-px overflow-hidden rounded-sm border border-border sm:grid-cols-2 lg:grid-cols-4">
            {platformFeatures.map((f) => (
              <Link
                key={f.title}
                href={f.href}
                className="group flex flex-col justify-between gap-8 bg-black p-7 transition-colors hover:bg-panel-raised"
              >
                <div className="flex items-center justify-between">
                  <span className="text-technical text-xs text-text-muted">{f.tag}</span>
                  <span className="text-sand opacity-0 transition-opacity group-hover:opacity-100">
                    →
                  </span>
                </div>
                <div>
                  <h3 className="text-display text-2xl text-off-white">{f.title}</h3>
                  <p className="mt-2 text-sm leading-relaxed text-text-muted">{f.body}</p>
                </div>
              </Link>
            ))}
          </div>
        </Container>
      </section>

      {/* Rewards / signup CTA */}
      <section id="rewards" className="border-t border-border py-24">
        <Container>
          <div className="relative overflow-hidden rounded-sm border border-border-strong bg-panel px-8 py-16 text-center sm:px-16">
            <div className="pointer-events-none absolute inset-0 bg-grid opacity-40" />
            <div className="relative">
              <p className="text-technical text-xs text-sand mb-4">EARLY DEVELOPER REWARDS</p>
              <h2 className="text-display text-4xl sm:text-5xl text-off-white max-w-2xl mx-auto leading-tight">
                Sign up, connect a wallet, get STARFORGE tokens.
              </h2>
              <p className="mt-5 max-w-lg mx-auto text-sm leading-relaxed text-off-white/70">
                Early developer accounts are credited with STARFORGE tokens
                on wallet connect — spend them on GPU compute and robot
                skills anywhere in the portal.
              </p>
              <div className="mt-9 flex justify-center">
                <Link href="/signup">
                  <Button size="lg">Create your account</Button>
                </Link>
              </div>
            </div>
          </div>
        </Container>
      </section>
    </>
  );
}
