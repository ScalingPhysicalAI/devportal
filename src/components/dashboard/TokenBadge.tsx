"use client";

import { useCurrentUser } from "@/hooks/useCurrentUser";
import type { SafeUser } from "@/lib/auth";
import { TOKEN_SYMBOL } from "@/lib/constants";

export function TokenBadge({ initialUser }: { initialUser: SafeUser }) {
  const { data: user } = useCurrentUser(initialUser);
  const balance = user?.tokenBalance ?? initialUser.tokenBalance;

  return (
    <div className="flex items-center gap-2 rounded-sm border border-border bg-panel px-3.5 h-9">
      <span className="h-1.5 w-1.5 rounded-full bg-sand" />
      <span className="text-technical text-sm text-off-white">{balance}{TOKEN_SYMBOL}</span>
      <span className="text-technical text-xs text-text-muted">credit</span>
    </div>
  );
}
