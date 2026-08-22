"use client";

import Link from "next/link";
import { useRouter } from "next/navigation";
import { useDisconnect } from "wagmi";

import { Logo } from "@/components/ui/Logo";
import { Button } from "@/components/ui/Button";
import { TokenBadge } from "@/components/dashboard/TokenBadge";
import { logout } from "@/lib/api-client";
import type { SafeUser } from "@/lib/auth";

export function TopBar({ user }: { user: SafeUser }) {
  const router = useRouter();
  const { disconnect } = useDisconnect();

  async function handleLogout() {
    disconnect();
    await logout();
    router.push("/");
    router.refresh();
  }

  return (
    <header className="sticky top-0 z-30 flex h-16 items-center justify-between border-b border-border bg-black/90 px-6 backdrop-blur">
      <Link href="/dashboard">
        <Logo />
      </Link>

      <div className="flex items-center gap-3">
        <TokenBadge initialUser={user} />
        <span className="hidden text-sm text-off-white/70 sm:inline">{user.name}</span>
        <Button variant="ghost" size="sm" onClick={handleLogout}>
          Log out
        </Button>
      </div>
    </header>
  );
}
