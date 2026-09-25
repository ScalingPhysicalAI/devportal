import { getCurrentUser } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { SkillsTabs } from "@/components/dashboard/SkillsTabs";

export default async function SkillsPage() {
  const user = await getCurrentUser();
  if (!user) return null;

  const orders = await prisma.skillOrder.findMany({
    where: { userId: user.id },
    select: { skillId: true },
  });

  return (
    <div className="w-full">
      <p className="text-technical text-xs text-sand mb-2">SKILLS</p>
      <h1 className="text-display text-4xl text-off-white">Robot skills</h1>
      <p className="mt-3 max-w-lg text-sm text-off-white/70">
        Buy pre-built skills from the marketplace, or train and deploy your
        own.
      </p>

      <div className="mt-10">
        <SkillsTabs ownedSkillIds={orders.map((o) => o.skillId)} />
      </div>
    </div>
  );
}
