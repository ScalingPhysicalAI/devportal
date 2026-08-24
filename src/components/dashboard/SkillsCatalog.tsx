"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useMutation, useQueryClient } from "@tanstack/react-query";

import { Button } from "@/components/ui/Button";
import { buySkill } from "@/lib/api-client";
import { SKILLS_CATALOG } from "@/lib/constants";

export function SkillsCatalog({ ownedSkillIds }: { ownedSkillIds: string[] }) {
  const router = useRouter();
  const queryClient = useQueryClient();
  const [error, setError] = useState<string | null>(null);
  const [buying, setBuying] = useState<string | null>(null);
  const [owned, setOwned] = useState(new Set(ownedSkillIds));

  const mutation = useMutation({
    mutationFn: (skillId: string) => buySkill(skillId),
    onMutate: (skillId) => {
      setError(null);
      setBuying(skillId);
    },
    onSuccess: (data, skillId) => {
      queryClient.setQueryData(["me"], data.user);
      setOwned((prev) => new Set(prev).add(skillId));
      router.refresh();
    },
    onError: (err: Error) => setError(err.message),
    onSettled: () => setBuying(null),
  });

  return (
    <div>
      <div className="grid gap-5 sm:grid-cols-2 lg:grid-cols-3">
        {SKILLS_CATALOG.map((skill) => {
          const isOwned = owned.has(skill.id);
          return (
            <div key={skill.id} className="flex flex-col rounded-sm border border-border bg-panel p-6">
              <div className="flex items-center justify-between">
                <p className="text-technical text-[11px] text-sand">{skill.category}</p>
                {isOwned && (
                  <span className="text-technical text-[11px] text-success">OWNED</span>
                )}
              </div>
              <p className="mt-2 text-display text-xl text-off-white">{skill.name}</p>
              <p className="mt-2 text-sm text-text-muted flex-1">{skill.desc}</p>
              <p className="mt-4 text-technical text-lg text-off-white">
                ${skill.price} <span className="text-sm text-text-muted">credit</span>
              </p>
              <Button
                className="mt-4"
                variant={isOwned ? "secondary" : "primary"}
                disabled={isOwned || mutation.isPending}
                onClick={() => mutation.mutate(skill.id)}
              >
                {isOwned
                  ? "Owned"
                  : buying === skill.id && mutation.isPending
                    ? "Buying…"
                    : "Buy skill"}
              </Button>
            </div>
          );
        })}
      </div>

      {error && <p className="mt-4 text-sm text-error">{error}</p>}
    </div>
  );
}
