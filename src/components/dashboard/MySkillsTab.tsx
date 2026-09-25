"use client";

import { useEffect, useState } from "react";
import Link from "next/link";
import { CheckCircle2, GraduationCap, Rocket } from "lucide-react";

import { Button } from "@/components/ui/Button";
import { deploySkill, getMySkills, type MyTrainedSkill } from "@/lib/my-skills";
import { formatDate } from "@/lib/format";

export function MySkillsTab() {
  const [skills, setSkills] = useState<MyTrainedSkill[] | null>(null);
  const [deploying, setDeploying] = useState<string | null>(null);

  useEffect(() => {
    setSkills(getMySkills());
  }, []);

  function handleDeploy(id: string) {
    setDeploying(id);
    // No backend yet (see my-skills.ts's own comment) -- a brief delay reads
    // as "publishing" instead of an instant, suspiciously-free state flip.
    window.setTimeout(() => {
      deploySkill(id);
      setSkills(getMySkills());
      setDeploying(null);
    }, 600);
  }

  if (skills === null) return null;

  if (skills.length === 0) {
    return (
      <div className="rounded-sm border border-dashed border-border-strong bg-panel px-8 py-16 text-center">
        <GraduationCap size={28} className="mx-auto text-text-muted" strokeWidth={1.5} />
        <p className="text-display text-2xl text-off-white mt-4">No trained skills yet</p>
        <p className="mx-auto mt-2 max-w-sm text-sm text-text-muted">
          Train a skill on your own dataset and it&apos;ll show up here once
          it&apos;s tested and ready to deploy.
        </p>
        <Link href="/dashboard/train">
          <Button variant="secondary" className="mt-6">
            Train a skill
          </Button>
        </Link>
      </div>
    );
  }

  return (
    <div className="grid gap-5 sm:grid-cols-2 lg:grid-cols-3">
      {skills.map((skill, i) => (
        <div
          key={skill.id}
          className="flex flex-col rounded-sm border border-border bg-panel p-6 animate-fade-up"
          style={{ animationDelay: `${i * 60}ms` }}
        >
          <div className="flex items-center justify-between">
            <p className="text-technical text-[11px] text-sand">{skill.baseModel}</p>
            {skill.status === "deployed" ? (
              <span className="flex items-center gap-1 text-technical text-[11px] text-success">
                <Rocket size={11} /> DEPLOYED
              </span>
            ) : (
              <span className="flex items-center gap-1 text-technical text-[11px] text-off-white/60">
                <CheckCircle2 size={11} /> TESTED
              </span>
            )}
          </div>
          <p className="mt-2 text-display text-xl text-off-white">{skill.name}</p>
          <p className="mt-2 text-sm text-text-muted flex-1">
            Trained {formatDate(new Date(skill.createdAt))}
          </p>
          <Button
            className="mt-4"
            variant={skill.status === "deployed" ? "secondary" : "primary"}
            disabled={skill.status === "deployed" || deploying === skill.id}
            onClick={() => handleDeploy(skill.id)}
          >
            {skill.status === "deployed"
              ? "Live on marketplace"
              : deploying === skill.id
                ? "Deploying…"
                : "Deploy to marketplace"}
          </Button>
        </div>
      ))}
    </div>
  );
}
