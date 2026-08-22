import { NextResponse } from "next/server";
import { Prisma } from "@prisma/client";

import { prisma } from "@/lib/prisma";
import { createSessionCookie, hashPassword, toSafeUser } from "@/lib/auth";
import { signupSchema } from "@/lib/validations";
import { sendWelcomeEmail } from "@/lib/mailer";

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

  let user;
  try {
    user = await prisma.user.create({
      data: { name, email, passwordHash, role },
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

  // Fire-and-forget: a failed welcome email should never block signup.
  sendWelcomeEmail(user.email, user.name).catch((err) => {
    console.error("[signup] Failed to send welcome email:", err);
  });

  return NextResponse.json({ user: toSafeUser(user) }, { status: 201 });
}
