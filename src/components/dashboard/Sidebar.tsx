"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import clsx from "clsx";

import { useOnboarding } from "@/components/onboarding/OnboardingProvider";

const items = [
  { href: "/dashboard", label: "Overview", exact: true, tour: "nav-overview" },
  { href: "/dashboard/robots", label: "Robots", tour: "nav-robots" },
  { href: "/dashboard/train", label: "Train", tour: "nav-train" },
  { href: "/dashboard/gpu", label: "GPU compute", tour: "nav-gpu" },
  { href: "/dashboard/skills", label: "Skills", tour: "nav-skills" },
];

export function Sidebar() {
  const pathname = usePathname();
  const { start } = useOnboarding();

  return (
    <aside className="hidden w-56 shrink-0 border-r border-border lg:block">
      <nav className="sticky top-16 flex h-[calc(100vh-4rem)] flex-col gap-1 px-4 py-8">
        {items.map((item) => {
          const active = item.exact ? pathname === item.href : pathname.startsWith(item.href);
          return (
            <Link
              key={item.href}
              href={item.href}
              data-tour={item.tour}
              className={clsx(
                "rounded-sm px-3 py-2 text-sm transition-colors",
                active
                  ? "bg-panel text-sand border border-border"
                  : "text-off-white/60 hover:text-off-white hover:bg-white/[0.03] border border-transparent"
              )}
            >
              {item.label}
            </Link>
          );
        })}

        <div className="mt-auto border-t border-border pt-4">
          <button
            data-tour="nav-tutorial"
            onClick={start}
            className="w-full rounded-sm border border-transparent px-3 py-2 text-left text-sm text-off-white/60 transition-colors hover:bg-white/[0.03] hover:text-off-white cursor-pointer"
          >
            Tutorial
          </button>
        </div>
      </nav>
    </aside>
  );
}
