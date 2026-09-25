"use client";

import { useState } from "react";
import clsx from "clsx";

import { SkillsCatalog } from "@/components/dashboard/SkillsCatalog";
import { MySkillsTab } from "@/components/dashboard/MySkillsTab";

export function SkillsTabs({ ownedSkillIds }: { ownedSkillIds: string[] }) {
  const [tab, setTab] = useState<"marketplace" | "mine">("marketplace");

  return (
    <div>
      <div className="flex gap-1 border-b border-border">
        {(
          [
            { id: "marketplace", label: "Marketplace" },
            { id: "mine", label: "My skills" },
          ] as const
        ).map((t) => (
          <button
            key={t.id}
            onClick={() => setTab(t.id)}
            className={clsx(
              "px-4 py-3 text-sm border-b-2 -mb-px transition-colors",
              tab === t.id
                ? "border-sand text-sand"
                : "border-transparent text-off-white/60 hover:text-off-white"
            )}
          >
            {t.label}
          </button>
        ))}
      </div>

      <div className="mt-8">
        {tab === "marketplace" ? <SkillsCatalog ownedSkillIds={ownedSkillIds} /> : <MySkillsTab />}
      </div>
    </div>
  );
}
