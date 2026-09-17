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
        {/* min-w-0 alongside the existing min-h-0 -- a flex item's default
            min-width is `auto`, not 0, so without it `main` was free to
            inflate past the viewport to fit a wide child's *preferred*
            width instead of constraining that child to the space actually
            available (confirmed live: the simulator's new side guide panel
            was being pushed off-screen this way, not a bug in that
            component's own layout at all). */}
        <main className={clsx("flex-1 min-h-0 min-w-0", fullBleed ? "" : "px-6 py-10 lg:px-10")}>{children}</main>
      </div>
    </div>
  );
}
