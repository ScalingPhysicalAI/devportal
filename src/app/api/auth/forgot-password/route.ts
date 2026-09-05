import { NextResponse } from "next/server";
import { randomBytes } from "crypto";

import { prisma } from "@/lib/prisma";
import { forgotPasswordSchema } from "@/lib/validations";
import { sendPasswordResetEmail } from "@/lib/mailer";

const GENERIC_MESSAGE = "If an account exists for that email, we've sent a reset link.";

export async function POST(request: Request) {
  const body = await request.json().catch(() => null);
  const parsed = forgotPasswordSchema.safeParse(body);

  if (!parsed.success) {
    return NextResponse.json(
      { error: parsed.error.issues[0]?.message ?? "Invalid input" },
      { status: 400 }
    );
  }

  const { email } = parsed.data;
  const user = await prisma.user.findUnique({ where: { email } });

  // Always respond the same way whether or not the account exists, so this
  // endpoint can't be used to enumerate registered emails.
  if (user) {
    const passwordResetToken = randomBytes(32).toString("hex");
    const passwordResetExpiry = new Date(Date.now() + 60 * 60 * 1000); // 1 h

    await prisma.user.update({
      where: { id: user.id },
      data: { passwordResetToken, passwordResetExpiry },
    });

    sendPasswordResetEmail(user.email, user.name, passwordResetToken).catch((err) => {
      console.error("[forgot-password] Failed to send reset email:", err);
    });
  }

  return NextResponse.json({ message: GENERIC_MESSAGE });
}
