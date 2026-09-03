import { NextResponse } from "next/server";

import { getCurrentUser, getUserFromRequest } from "@/lib/auth";

export async function GET(request: Request) {
  // Cookie-based session (web) first, then Authorization: Bearer (mobile / API).
  const user = (await getCurrentUser()) ?? (await getUserFromRequest(request));
  if (!user) {
    return NextResponse.json({ user: null }, { status: 200 });
  }
  return NextResponse.json({ user });
}
