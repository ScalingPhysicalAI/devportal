import { NextResponse } from "next/server";

import { prisma } from "@/lib/prisma";

const APP_URL = process.env.NEXT_PUBLIC_APP_URL ?? "http://localhost:3000";
const EMAIL_VERIFY_CREDIT = 20;

export async function GET(request: Request) {
  const { searchParams } = new URL(request.url);
  const token = searchParams.get("token");

  if (!token) {
    return NextResponse.redirect(`${APP_URL}/dashboard?verified=invalid`);
  }

  const user = await prisma.user.findUnique({ where: { emailVerifyToken: token } });

  if (!user) {
    return NextResponse.redirect(`${APP_URL}/dashboard?verified=invalid`);
  }

  if (user.emailVerified) {
    return NextResponse.redirect(`${APP_URL}/dashboard?verified=already`);
  }

  if (user.emailVerifyExpiry && user.emailVerifyExpiry < new Date()) {
    return NextResponse.redirect(`${APP_URL}/dashboard?verified=expired`);
  }

  // Mark verified, clear token, credit 20 tokens — all in one transaction.
  await prisma.$transaction([
    prisma.user.update({
      where: { id: user.id },
      data: {
        emailVerified: true,
        emailVerifyToken: null,
        emailVerifyExpiry: null,
        tokenBalance: { increment: EMAIL_VERIFY_CREDIT },
      },
    }),
    prisma.tokenTransaction.create({
      data: {
        userId: user.id,
        type: "EMAIL_VERIFY_BONUS",
        amount: EMAIL_VERIFY_CREDIT,
        note: "Email verification reward",
      },
    }),
  ]);

  return NextResponse.redirect(`${APP_URL}/dashboard?verified=1`);
}
