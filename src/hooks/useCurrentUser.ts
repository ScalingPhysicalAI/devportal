"use client";

import { useQuery } from "@tanstack/react-query";

import { fetchMe } from "@/lib/api-client";
import type { SafeUser } from "@/lib/auth";

export function useCurrentUser(initialUser?: SafeUser) {
  return useQuery({
    queryKey: ["me"],
    queryFn: async () => (await fetchMe()).user,
    initialData: initialUser ?? undefined,
    staleTime: 10_000,
  });
}
