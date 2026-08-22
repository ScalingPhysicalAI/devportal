"use client";

import type { SafeUser } from "@/lib/auth";

async function handle<T>(res: Response): Promise<T> {
  const data = await res.json();
  if (!res.ok) {
    throw new Error(data.error ?? "Request failed");
  }
  return data as T;
}

export async function fetchMe() {
  const res = await fetch("/api/auth/me");
  return handle<{ user: SafeUser | null }>(res);
}

export async function logout() {
  await fetch("/api/auth/logout", { method: "POST" });
}

export async function connectWallet(address: string) {
  const res = await fetch("/api/wallet/connect", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ address }),
  });
  return handle<{ user: SafeUser; rewardGranted: number }>(res);
}

export async function rentGpu(gpuType: string, hours: number) {
  const res = await fetch("/api/gpu/rent", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ gpuType, hours }),
  });
  return handle<{ user: SafeUser; session: unknown }>(res);
}

export async function completeOnboarding() {
  const res = await fetch("/api/auth/onboarding", { method: "POST" });
  return handle<{ user: SafeUser }>(res);
}

export async function buySkill(skillId: string) {
  const res = await fetch("/api/skills/buy", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ skillId }),
  });
  return handle<{ user: SafeUser; order: unknown }>(res);
}
