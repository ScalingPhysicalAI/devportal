"use client";

import Link from "next/link";
import { useRouter } from "next/navigation";
import { useDisconnect } from "wagmi";
import { Menu, X } from "lucide-react";

import { Logo } from "@/components/ui/Logo";
import { Button } from "@/components/ui/Button";
import { TokenBadge } from "@/components/dashboard/TokenBadge";
import { logout } from "@/lib/api-client";
import type { SafeUser } from "@/lib/auth";

interface TopBarProps {
  user: SafeUser;
  onToggleSidebar: () => void;
  sidebarOpen: boolean;
}

export function TopBar({ user, onToggleSidebar, sidebarOpen }: TopBarProps) {
  const router = useRouter();
  const { disconnect } = useDisconnect();

  async function handleLogout() {
    disconnect();
    await logout();
    router.push("/");
    router.refresh();
  }

  return (
    <header className="sticky top-0 z-30 flex h-16 w-full items-center justify-between border-b border-border bg-black/90 px-6 backdrop-blur">
      <div className="flex items-center gap-4">
        <button
          onClick={onToggleSidebar}
          aria-label={sidebarOpen ? "Close menu" : "Open menu"}
          aria-expanded={sidebarOpen}
          className="flex h-8 w-8 shrink-0 items-center justify-center rounded-sm border border-border-strong text-off-white/70 transition-colors hover:text-off-white hover:bg-white/[0.03] cursor-pointer"
        >
          {sidebarOpen ? <X size={16} /> : <Menu size={16} />}
        </button>
        <Link href="/dashboard">
          <Logo />
        </Link>
      </div>

      <div className="flex items-center gap-3">
        <a href="https://starforgerobotics.com" target="_blank" rel="noopener noreferrer">
          <Button variant="ghost" size="sm">
            Back to Website
          </Button>
        </a>
        <TokenBadge initialUser={user} />
        <span className="hidden text-sm text-off-white/70 sm:inline">{user.name}</span>
        <Button variant="ghost" size="sm" onClick={handleLogout}>
          Log out
        </Button>
      </div>
    </header>
  );
}
