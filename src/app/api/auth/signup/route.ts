import { NextResponse } from "next/server";
import { randomBytes } from "crypto";
import { Prisma } from "@prisma/client";

import { prisma } from "@/lib/prisma";
import { createSessionCookie, hashPassword, toSafeUser } from "@/lib/auth";
import { signupSchema } from "@/lib/validations";
import { sendVerificationEmail } from "@/lib/mailer";

export async function POST(request: Request) {
  const body = await request.json().catch(() => null);
  const parsed = signupSchema.safeParse(body);

  if (!parsed.success) {
    return NextResponse.json(
      { error: parsed.error.issues[0]?.message ?? "Invalid input" },
      { status: 400 }
    );
  }

  const { name, email, password, role } = parsed.data;
  const passwordHash = await hashPassword(password);
  const emailVerifyToken = randomBytes(32).toString("hex");
  const emailVerifyExpiry = new Date(Date.now() + 24 * 60 * 60 * 1000); // 24 h

  let user;
  try {
    user = await prisma.user.create({
      data: { name, email, passwordHash, role, emailVerifyToken, emailVerifyExpiry },
    });
  } catch (err) {
    if (err instanceof Prisma.PrismaClientKnownRequestError && err.code === "P2002") {
      return NextResponse.json(
        { error: "An account with that email already exists" },
        { status: 409 }
      );
    }
    throw err;
  }

  await createSessionCookie(user.id);

  // Fire-and-forget — a failed email should never block signup.
  sendVerificationEmail(user.email, user.name, emailVerifyToken).catch((err) => {
    console.error("[signup] Failed to send verification email:", err);
  });

  return NextResponse.json({ user: toSafeUser(user) }, { status: 201 });
}
