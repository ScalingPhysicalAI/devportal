"use client";

import { useEffect } from "react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import clsx from "clsx";

import { useOnboarding } from "@/components/onboarding/OnboardingProvider";

const items = [
  { href: "/dashboard", label: "Overview", exact: true, tour: "nav-overview" },
  { href: "/dashboard/robots", label: "Robots", tour: "nav-robots" },
  { href: "/dashboard/simulate", label: "Simulation", tour: "nav-simulate" },
  { href: "/dashboard/train", label: "Train", tour: "nav-train" },
  { href: "/dashboard/gpu", label: "GPU compute", tour: "nav-gpu" },
  { href: "/dashboard/skills", label: "Skills", tour: "nav-skills" },
];

interface SidebarProps {
  /** Drawer visibility -- the sidebar is an overlay, not a permanently
   * reserved layout column, toggled from TopBar's menu button. */
  open: boolean;
  onClose: () => void;
}

export function Sidebar({ open, onClose }: SidebarProps) {
  const pathname = usePathname();
  const { start } = useOnboarding();

  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") onClose();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [open, onClose]);

  return (
    <>
      {open && (
        <div
          className="fixed inset-0 z-30 bg-black/60 backdrop-blur-[1px]"
          onClick={onClose}
          aria-hidden="true"
        />
      )}
      <aside
        className={clsx(
          "fixed inset-y-0 left-0 z-40 w-64 border-r border-border bg-black transition-transform duration-200 ease-out",
          open ? "translate-x-0" : "-translate-x-full"
        )}
      >
        <nav className="flex h-full flex-col gap-1 px-4 py-8 overflow-y-auto">
          {items.map((item) => {
            const active = item.exact ? pathname === item.href : pathname.startsWith(item.href);
            return (
              <Link
                key={item.href}
                href={item.href}
                data-tour={item.tour}
                onClick={onClose}
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
              onClick={() => {
                onClose();
                start();
              }}
              className="w-full rounded-sm border border-transparent px-3 py-2 text-left text-sm text-off-white/60 transition-colors hover:bg-white/[0.03] hover:text-off-white cursor-pointer"
            >
              Tutorial
            </button>
          </div>
        </nav>
      </aside>
    </>
  );
}
