"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import clsx from "clsx";

const items = [
  { href: "/dashboard", label: "Overview", exact: true, tour: "nav-overview" },
  { href: "/dashboard/robots", label: "Robots", tour: "nav-robots" },
  { href: "/dashboard/train", label: "Train", tour: "nav-train" },
  { href: "/dashboard/gpu", label: "GPU", tour: "nav-gpu" },
  { href: "/dashboard/skills", label: "Skills", tour: "nav-skills" },
];

export function MobileTabs() {
  const pathname = usePathname();

  return (
    <nav className="flex gap-1 overflow-x-auto border-b border-border px-4 py-2 lg:hidden">
      {items.map((item) => {
        const active = item.exact ? pathname === item.href : pathname.startsWith(item.href);
        return (
          <Link
            key={item.href}
            href={item.href}
            data-tour={item.tour}
            className={clsx(
              "shrink-0 rounded-sm px-3 py-1.5 text-xs whitespace-nowrap",
              active ? "bg-panel text-sand border border-border" : "text-off-white/60"
            )}
          >
            {item.label}
          </Link>
        );
      })}
    </nav>
  );
}
