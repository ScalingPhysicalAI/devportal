import Link from "next/link";

import { Container } from "@/components/ui/Container";
import { Logo } from "@/components/ui/Logo";

export function Footer() {
  return (
    <footer className="border-t border-border bg-black">
      <Container className="flex flex-col gap-8 py-14 sm:flex-row sm:items-start sm:justify-between">
        <div className="max-w-xs">
          <Logo />
          <p className="mt-4 text-sm leading-relaxed text-text-muted">
            The developer platform for Buildo — Starforge&apos;s Physical AI
            robot. Train it, control it, and put it to work.
          </p>
        </div>

        <div className="grid grid-cols-2 gap-10 sm:grid-cols-3">
          <div>
            <p className="text-technical text-[11px] text-sand mb-3">Platform</p>
            <ul className="space-y-2 text-sm text-off-white/70">
              <li>
                <Link href="/dashboard/train" className="hover:text-sand">
                  Train
                </Link>
              </li>
              <li>
                <Link href="/dashboard/gpu" className="hover:text-sand">
                  GPU compute
                </Link>
              </li>
              <li>
                <Link href="/dashboard/skills" className="hover:text-sand">
                  Skills
                </Link>
              </li>
            </ul>
          </div>
          <div>
            <p className="text-technical text-[11px] text-sand mb-3">Account</p>
            <ul className="space-y-2 text-sm text-off-white/70">
              <li>
                <Link href="/signup" className="hover:text-sand">
                  Sign up
                </Link>
              </li>
              <li>
                <Link href="/login" className="hover:text-sand">
                  Log in
                </Link>
              </li>
            </ul>
          </div>
          <div>
            <p className="text-technical text-[11px] text-sand mb-3">Company</p>
            <ul className="space-y-2 text-sm text-off-white/70">
              <li>
                <a
                  href="https://starforgerobotics.com"
                  target="_blank"
                  rel="noreferrer"
                  className="hover:text-sand"
                >
                  starforgerobotics.com
                </a>
              </li>
              <li>
                <a
                  href="https://starforgerobotics.com/buildo/"
                  target="_blank"
                  rel="noreferrer"
                  className="hover:text-sand"
                >
                  Buildo robot
                </a>
              </li>
            </ul>
          </div>
        </div>
      </Container>

      <Container className="border-t border-border py-6">
        <p className="text-technical text-[11px] text-text-muted">
          © {new Date().getFullYear()} Starforge Robotics. Developer portal preview.
        </p>
      </Container>
    </footer>
  );
}
