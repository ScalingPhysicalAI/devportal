"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import clsx from "clsx";
import { Bot, GraduationCap, LayoutGrid, Sparkles, Waves } from "lucide-react";

const items = [
  { href: "/dashboard", label: "Overview", exact: true, tour: "nav-overview", icon: LayoutGrid },
  { href: "/dashboard/robots", label: "Robots", tour: "nav-robots", icon: Bot },
  { href: "/dashboard/simulate", label: "Simulation", tour: "nav-simulate", icon: Waves },
  { href: "/dashboard/train", label: "Train", tour: "nav-train", icon: GraduationCap },
  { href: "/dashboard/skills", label: "Skills", tour: "nav-skills", icon: Sparkles },
];

export function MobileTabs() {
  const pathname = usePathname();

  return (
    <nav className="flex gap-1 overflow-x-auto border-b border-border px-4 py-2 lg:hidden">
      {items.map((item) => {
        const active = item.exact ? pathname === item.href : pathname.startsWith(item.href);
        const Icon = item.icon;
        return (
          <Link
            key={item.href}
            href={item.href}
            data-tour={item.tour}
            className={clsx(
              "flex shrink-0 items-center gap-1.5 rounded-sm px-3 py-1.5 text-xs whitespace-nowrap",
              active ? "bg-panel text-sand border border-border" : "text-off-white/60"
            )}
          >
            <Icon size={13} strokeWidth={1.75} />
            {item.label}
          </Link>
        );
      })}
    </nav>
  );
}
