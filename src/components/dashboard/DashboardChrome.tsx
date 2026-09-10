"use client";

import { useState } from "react";
import { usePathname } from "next/navigation";
import clsx from "clsx";

import { TopBar } from "@/components/dashboard/TopBar";
import { Sidebar } from "@/components/dashboard/Sidebar";
import { MobileTabs } from "@/components/dashboard/MobileTabs";
import type { SafeUser } from "@/lib/auth";

// Routes that want the full viewport below the navbar (no side padding, no
// permanently reserved sidebar column, no mobile tab strip) -- currently
// just the simulator (see simulate/page.tsx), which just fills whatever
// height/width `main` gives it.
const FULL_BLEED_PREFIXES = ["/dashboard/simulate"];

export function DashboardChrome({ user, children }: { user: SafeUser; children: React.ReactNode }) {
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const pathname = usePathname();
  const fullBleed = FULL_BLEED_PREFIXES.some((p) => pathname.startsWith(p));

  return (
    <div className={clsx("flex w-full flex-col", fullBleed ? "h-screen overflow-hidden" : "min-h-screen")}>
      <TopBar user={user} onToggleSidebar={() => setSidebarOpen((o) => !o)} sidebarOpen={sidebarOpen} />
      {!fullBleed && <MobileTabs />}
      <div className="flex flex-1 min-h-0">
        <Sidebar open={sidebarOpen} onClose={() => setSidebarOpen(false)} />
        <main className={clsx("flex-1 min-h-0", fullBleed ? "" : "px-6 py-10 lg:px-10")}>{children}</main>
      </div>
    </div>
  );
}
