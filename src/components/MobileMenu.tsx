"use client";

import { useState } from "react";
import Link from "next/link";
import type { ReactNode } from "react";

export function MobileMenu({
  links,
  authArea,
}: {
  links: { href: string; label: string }[];
  authArea: ReactNode;
}) {
  const [open, setOpen] = useState(false);

  return (
    <div className="sm:hidden">
      <button
        aria-label="Toggle menu"
        onClick={() => setOpen((v) => !v)}
        className="flex h-9 w-9 items-center justify-center rounded-sm border border-border text-off-white"
      >
        <span className="text-technical text-xs">{open ? "×" : "≡"}</span>
      </button>

      {open && (
        <div className="absolute inset-x-0 top-16 z-40 border-b border-border bg-black px-6 py-6">
          <nav className="flex flex-col gap-4">
            {links.map((l) => (
              <Link
                key={l.href}
                href={l.href}
                onClick={() => setOpen(false)}
                className="text-sm text-off-white/80 hover:text-sand"
              >
                {l.label}
              </Link>
            ))}
            <div className="mt-2 flex flex-col gap-3">{authArea}</div>
          </nav>
        </div>
      )}
    </div>
  );
}
