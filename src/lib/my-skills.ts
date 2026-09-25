"use client";

// Client-only, localStorage-backed mock data for a developer's own
// trained/tested skills -- there's no training backend yet (see
// TrainTabs.tsx's own comment), so this stands in for what would otherwise
// be a database table, just enough to make "train -> test -> deploy to
// marketplace" read as one coherent flow across the Train and Skills pages
// in a demo.

export type MyTrainedSkill = {
  id: string;
  name: string;
  baseModel: string;
  status: "tested" | "deployed";
  createdAt: number;
};

const STORAGE_KEY = "sf-my-skills";

const SEED_SKILLS: MyTrainedSkill[] = [
  {
    id: "seed-cup-pick-place",
    name: "Kitchen Cup Pick & Place",
    baseModel: "Buildo Household v1",
    status: "deployed",
    createdAt: Date.now() - 1000 * 60 * 60 * 24 * 6,
  },
  {
    id: "seed-pallet-sort",
    name: "Warehouse Pallet Sort",
    baseModel: "Buildo Warehouse v1",
    status: "tested",
    createdAt: Date.now() - 1000 * 60 * 60 * 24 * 2,
  },
];

function readAll(): MyTrainedSkill[] {
  if (typeof window === "undefined") return SEED_SKILLS;
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) return SEED_SKILLS;
    return JSON.parse(raw) as MyTrainedSkill[];
  } catch {
    return SEED_SKILLS;
  }
}

function writeAll(skills: MyTrainedSkill[]) {
  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(skills));
  } catch {
    // Private browsing / storage disabled -- fine, this is demo data only.
  }
}

export function getMySkills(): MyTrainedSkill[] {
  return readAll().sort((a, b) => b.createdAt - a.createdAt);
}

export function addTrainedSkill(name: string, baseModel: string): MyTrainedSkill {
  const skill: MyTrainedSkill = {
    id: `${Date.now()}`,
    name,
    baseModel,
    status: "tested",
    createdAt: Date.now(),
  };
  writeAll([skill, ...readAll()]);
  return skill;
}

export function deploySkill(id: string) {
  writeAll(readAll().map((s) => (s.id === id ? { ...s, status: "deployed" } : s)));
}
