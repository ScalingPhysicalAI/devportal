import { NextResponse } from "next/server";
import { Prisma } from "@prisma/client";

import { prisma } from "@/lib/prisma";
import { getSessionUserId, toSafeUser } from "@/lib/auth";
import { walletConnectSchema } from "@/lib/validations";
import { EARLY_WALLET_REWARD } from "@/lib/constants";

export async function POST(request: Request) {
  const userId = await getSessionUserId();
  if (!userId) {
    return NextResponse.json({ error: "Not authenticated" }, { status: 401 });
  }

  const body = await request.json().catch(() => null);
  const parsed = walletConnectSchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json(
      { error: parsed.error.issues[0]?.message ?? "Invalid input" },
      { status: 400 }
    );
  }

  const address = parsed.data.address.toLowerCase();

  const existingUser = await prisma.user.findUnique({ where: { id: userId } });
  if (!existingUser) {
    return NextResponse.json({ error: "Not authenticated" }, { status: 401 });
  }

  const alreadyRewarded = existingUser.walletAddress !== null;

  try {
    const user = await prisma.$transaction(async (tx) => {
      const updated = await tx.user.update({
        where: { id: userId },
        data: {
          walletAddress: address,
          ...(alreadyRewarded
            ? {}
            : { tokenBalance: { increment: EARLY_WALLET_REWARD } }),
        },
      });

      if (!alreadyRewarded) {
        await tx.tokenTransaction.create({
          data: {
            userId,
            type: "WALLET_CONNECT_BONUS",
            amount: EARLY_WALLET_REWARD,
            note: "Early developer signup reward",
          },
        });
      }

      return updated;
    });

    return NextResponse.json({
      user: toSafeUser(user),
      rewardGranted: !alreadyRewarded ? EARLY_WALLET_REWARD : 0,
    });
  } catch (err) {
    if (err instanceof Prisma.PrismaClientKnownRequestError && err.code === "P2002") {
      return NextResponse.json(
        { error: "That wallet address is already linked to another account" },
        { status: 409 }
      );
    }
    throw err;
  }
}
