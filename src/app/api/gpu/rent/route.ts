import { NextResponse } from "next/server";

import { prisma } from "@/lib/prisma";
import { getSessionUserId, toSafeUser } from "@/lib/auth";
import { gpuRentSchema } from "@/lib/validations";
import { GPU_CATALOG } from "@/lib/constants";

export async function POST(request: Request) {
  const userId = await getSessionUserId();
  if (!userId) {
    return NextResponse.json({ error: "Not authenticated" }, { status: 401 });
  }

  const body = await request.json().catch(() => null);
  const parsed = gpuRentSchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json(
      { error: parsed.error.issues[0]?.message ?? "Invalid input" },
      { status: 400 }
    );
  }

  const { gpuType, hours } = parsed.data;
  const gpu = GPU_CATALOG.find((g) => g.type === gpuType);
  if (!gpu) {
    return NextResponse.json({ error: "Unknown GPU type" }, { status: 400 });
  }

  const totalPrice = gpu.pricePerHour * hours;

  try {
    const result = await prisma.$transaction(async (tx) => {
      const user = await tx.user.findUnique({ where: { id: userId } });
      if (!user) throw new Error("NOT_FOUND");
      if (user.tokenBalance < totalPrice) throw new Error("INSUFFICIENT_BALANCE");

      const updatedUser = await tx.user.update({
        where: { id: userId },
        data: { tokenBalance: { decrement: totalPrice } },
      });

      const session = await tx.gpuSession.create({
        data: {
          userId,
          gpuType,
          hours,
          pricePaid: totalPrice,
          endsAt: new Date(Date.now() + hours * 60 * 60 * 1000),
        },
      });

      await tx.tokenTransaction.create({
        data: {
          userId,
          type: "GPU_RENTAL",
          amount: -totalPrice,
          note: `${hours}h ${gpuType}`,
        },
      });

      return { user: updatedUser, session };
    });

    return NextResponse.json({
      user: toSafeUser(result.user),
      session: result.session,
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
