import Link from "next/link";

import { getCurrentUser } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { formatDate, transactionLabel } from "@/lib/format";
import { StatCard } from "@/components/dashboard/StatCard";
import { SimulationCard } from "@/components/dashboard/SimulationCard";
import { Button } from "@/components/ui/Button";

export default async function DashboardOverviewPage({
  searchParams,
}: {
  searchParams: Promise<{ welcome?: string }>;
}) {
  const [user, { welcome }] = await Promise.all([getCurrentUser(), searchParams]);
  if (!user) return null;

  const [transactions, gpuCount, skillCount] = await Promise.all([
    prisma.tokenTransaction.findMany({
      where: { userId: user.id },
      orderBy: { createdAt: "desc" },
      take: 6,
    }),
    prisma.gpuSession.count({ where: { userId: user.id } }),
    prisma.skillOrder.count({ where: { userId: user.id } }),
  ]);

  return (
    <div className="max-w-5xl">
      {welcome === "1" && (
        <div className="mb-8 rounded-sm border border-success/30 bg-success/5 px-5 py-4">
          <p className="text-sm text-off-white">
            Welcome, {user.name} — your account is set up. Check your inbox
            for a confirmation email. Your $20 USDC credit has been added.
          </p>
        </div>
      )}

      <p className="text-technical text-xs text-sand mb-2">OVERVIEW</p>
      <h1 className="text-display text-4xl text-off-white">
        Welcome back, {user.name.split(" ")[0]}
      </h1>

      <div className="mt-8 grid gap-5 sm:grid-cols-3">
        <StatCard label="Credit balance" value={`$${user.tokenBalance}`} />
        <StatCard label="GPU sessions" value={gpuCount} />
        <StatCard label="Skills owned" value={skillCount} />
      </div>

      <div className="mt-8 grid gap-6 lg:grid-cols-[1.3fr_1fr]">
        <SimulationCard />

        <div className="rounded-sm border border-border bg-panel p-7">
          <p className="text-technical text-xs text-sand mb-4">QUICK LINKS</p>
          <div className="flex flex-col gap-3">
            <Link href="/dashboard/train">
              <Button variant="secondary" className="w-full justify-between">
                Train your robot <span aria-hidden>→</span>
              </Button>
            </Link>
            <Link href="/dashboard/gpu">
              <Button variant="secondary" className="w-full justify-between">
                Rent GPU compute <span aria-hidden>→</span>
              </Button>
            </Link>
            <Link href="/dashboard/skills">
              <Button variant="secondary" className="w-full justify-between">
                Browse skills <span aria-hidden>→</span>
              </Button>
            </Link>
          </div>
        </div>
      </div>

      <div className="mt-8">
        <p className="text-technical text-xs text-sand mb-4">RECENT ACTIVITY</p>
        <div className="overflow-hidden rounded-sm border border-border">
          {transactions.length === 0 ? (
            <p className="bg-panel px-5 py-8 text-center text-sm text-text-muted">
              No activity yet — rent GPU compute or buy a skill to get started.
            </p>
          ) : (
            <table className="w-full text-sm">
              <tbody>
                {transactions.map((tx) => (
                  <tr key={tx.id} className="border-b border-border last:border-0 bg-panel">
                    <td className="px-5 py-3.5 text-off-white/80">
                      {transactionLabel(tx.type)}
                      {tx.note && (
                        <span className="ml-2 text-text-muted">— {tx.note}</span>
                      )}
                    </td>
                    <td className="px-5 py-3.5 text-right text-technical whitespace-nowrap">
                      <span className={tx.amount >= 0 ? "text-success" : "text-off-white/70"}>
                        {tx.amount >= 0 ? "+" : ""}${Math.abs(tx.amount)} credit
                      </span>
                    </td>
                    <td className="px-5 py-3.5 text-right text-text-muted whitespace-nowrap">
                      {formatDate(tx.createdAt)}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>
      </div>
    </div>
  );
}
