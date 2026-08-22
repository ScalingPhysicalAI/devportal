import { NextResponse } from "next/server";

import { prisma } from "@/lib/prisma";
import { getSessionUserId, toSafeUser } from "@/lib/auth";
import { skillBuySchema } from "@/lib/validations";
import { SKILLS_CATALOG } from "@/lib/constants";

export async function POST(request: Request) {
  const userId = await getSessionUserId();
  if (!userId) {
    return NextResponse.json({ error: "Not authenticated" }, { status: 401 });
  }

  const body = await request.json().catch(() => null);
  const parsed = skillBuySchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json(
      { error: parsed.error.issues[0]?.message ?? "Invalid input" },
      { status: 400 }
    );
  }

  const skill = SKILLS_CATALOG.find((s) => s.id === parsed.data.skillId);
  if (!skill) {
    return NextResponse.json({ error: "Unknown skill" }, { status: 400 });
  }

  const alreadyOwned = await prisma.skillOrder.findFirst({
    where: { userId, skillId: skill.id },
  });
  if (alreadyOwned) {
    return NextResponse.json({ error: "You already own this skill" }, { status: 409 });
  }

  try {
    const result = await prisma.$transaction(async (tx) => {
      const user = await tx.user.findUnique({ where: { id: userId } });
      if (!user) throw new Error("NOT_FOUND");
      if (user.tokenBalance < skill.price) throw new Error("INSUFFICIENT_BALANCE");

      const updatedUser = await tx.user.update({
        where: { id: userId },
        data: { tokenBalance: { decrement: skill.price } },
      });

      const order = await tx.skillOrder.create({
        data: {
          userId,
          skillId: skill.id,
          skillName: skill.name,
          price: skill.price,
        },
      });

      await tx.tokenTransaction.create({
        data: {
          userId,
          type: "SKILL_PURCHASE",
          amount: -skill.price,
          note: skill.name,
        },
      });

      return { user: updatedUser, order };
    });

    return NextResponse.json({
      user: toSafeUser(result.user),
      order: result.order,
    });
  } catch (err) {
    if (err instanceof Error && err.message === "INSUFFICIENT_BALANCE") {
      return NextResponse.json({ error: "Not enough STARFORGE tokens" }, { status: 402 });
    }
    if (err instanceof Error && err.message === "NOT_FOUND") {
      return NextResponse.json({ error: "Not authenticated" }, { status: 401 });
    }
    throw err;
  }
}
