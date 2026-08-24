import Link from "next/link";

import { getCurrentUser } from "@/lib/auth";
import { Button } from "@/components/ui/Button";
import { Container } from "@/components/ui/Container";
import { Logo } from "@/components/ui/Logo";
import { MobileMenu } from "@/components/MobileMenu";

const links = [
  { href: "/#platform", label: "Platform" },
  { href: "/#rewards", label: "Rewards" },
];

export async function Nav() {
  const user = await getCurrentUser();

  return (
    <header className="relative z-50 border-b border-border bg-black/80 backdrop-blur">
      <Container className="flex h-16 items-center justify-between">
        <Link href="/">
          <Logo />
        </Link>

        <nav className="hidden sm:flex items-center gap-8">
          {links.map((l) => (
            <Link
              key={l.href}
              href={l.href}
              className="text-sm text-off-white/70 transition-colors hover:text-sand"
            >
              {l.label}
            </Link>
          ))}
        </nav>

        <div className="hidden sm:flex items-center gap-3">
          {user ? (
            <Link href="/dashboard">
              <Button size="sm">Dashboard</Button>
            </Link>
          ) : (
            <>
              <Link href="/login">
                <Button variant="ghost" size="sm">
                  Log in
                </Button>
              </Link>
              <Link href="/signup">
                <Button size="sm">Sign up</Button>
              </Link>
            </>
          )}
        </div>

        <MobileMenu
          links={links}
          authArea={
            user ? (
              <Link href="/dashboard">
                <Button size="sm" className="w-full">
                  Dashboard
                </Button>
              </Link>
            ) : (
              <>
                <Link href="/login">
                  <Button variant="secondary" size="sm" className="w-full">
                    Log in
                  </Button>
                </Link>
                <Link href="/signup">
                  <Button size="sm" className="w-full">
                    Sign up
                  </Button>
                </Link>
              </>
            )
          }
        />
      </Container>
    </header>
  );
}
