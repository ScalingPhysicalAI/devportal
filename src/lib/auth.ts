import "server-only";

import { cache } from "react";
import bcrypt from "bcryptjs";
import { SignJWT, jwtVerify } from "jose";
import { cookies } from "next/headers";
import type { User } from "@prisma/client";

import { prisma } from "@/lib/prisma";

const SESSION_COOKIE = "sf_session";
const SESSION_TTL_SECONDS = 60 * 60 * 24 * 30; // 30 days

function getSecretKey() {
  const secret = process.env.AUTH_SECRET;
  if (!secret) {
    throw new Error("AUTH_SECRET is not set. Copy .env.example to .env and fill it in.");
  }
  return new TextEncoder().encode(secret);
}

export async function hashPassword(password: string) {
  return bcrypt.hash(password, 10);
}

export async function verifyPassword(password: string, hash: string) {
  return bcrypt.compare(password, hash);
}

export async function createSessionToken(userId: string) {
  return new SignJWT({ sub: userId })
    .setProtectedHeader({ alg: "HS256" })
    .setIssuedAt()
    .setExpirationTime(`${SESSION_TTL_SECONDS}s`)
    .sign(getSecretKey());
}

/**
 * Sets the session cookie (used by the web app) and returns the same signed
 * JWT so API routes can also hand it back in the response body for non-browser
 * clients such as the mobile app, which authenticate with `Authorization:
 * Bearer <token>` instead of cookies.
 */
export async function createSessionCookie(userId: string) {
  const token = await createSessionToken(userId);

  const cookieStore = await cookies();
  cookieStore.set(SESSION_COOKIE, token, {
    httpOnly: true,
    secure: process.env.NODE_ENV === "production",
    sameSite: "lax",
    path: "/",
    maxAge: SESSION_TTL_SECONDS,
  });

  return token;
}

export async function clearSessionCookie() {
  const cookieStore = await cookies();
  cookieStore.delete(SESSION_COOKIE);
}

function getBearerToken(request?: Request): string | null {
  const header = request?.headers.get("authorization") ?? request?.headers.get("Authorization");
  if (!header) return null;
  const [scheme, value] = header.split(" ");
  return scheme?.toLowerCase() === "bearer" && value ? value.trim() : null;
}

/**
 * Resolves the current user id from either the `sf_session` cookie (web) or an
 * `Authorization: Bearer <token>` header (mobile / API clients). Pass `request`
 * from a route handler to enable the header path; without it, cookie only.
 */
export async function getSessionUserId(request?: Request): Promise<string | null> {
  let token = getBearerToken(request);

  if (!token) {
    const cookieStore = await cookies();
    token = cookieStore.get(SESSION_COOKIE)?.value ?? null;
  }
  if (!token) return null;

  try {
    const { payload } = await jwtVerify(token, getSecretKey());
    return typeof payload.sub === "string" ? payload.sub : null;
  } catch {
    return null;
  }
}

/**
 * Non-cached variant of {@link getCurrentUser} that also accepts a bearer token
 * from the given request. Use this in API route handlers that must serve both
 * the web app and the mobile app.
 */
export async function getUserFromRequest(request: Request): Promise<SafeUser | null> {
  const userId = await getSessionUserId(request);
  if (!userId) return null;

  const user = await prisma.user.findUnique({ where: { id: userId } });
  return user ? toSafeUser(user) : null;
}

export type SafeUser = Omit<User, "passwordHash">;

export function toSafeUser(user: User): SafeUser {
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  const { passwordHash, ...safe } = user;
  return safe;
}

export const getCurrentUser = cache(async (): Promise<SafeUser | null> => {
  const userId = await getSessionUserId();
  if (!userId) return null;

  const user = await prisma.user.findUnique({ where: { id: userId } });
  if (!user) return null;

  return toSafeUser(user);
});
