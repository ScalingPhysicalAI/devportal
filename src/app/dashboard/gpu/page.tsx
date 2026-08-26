import { getCurrentUser } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { TOKEN_SYMBOL } from "@/lib/constants";
import { formatDate } from "@/lib/format";
import { GpuCatalog } from "@/components/dashboard/GpuCatalog";

export default async function GpuPage() {
  const user = await getCurrentUser();
  if (!user) return null;

  const sessions = await prisma.gpuSession.findMany({
    where: { userId: user.id },
    orderBy: { startedAt: "desc" },
    take: 10,
  });

  return (
    <div className="max-w-5xl">
      <p className="text-technical text-xs text-sand mb-2">COMPUTE</p>
      <h1 className="text-display text-4xl text-off-white">Rent GPU compute</h1>
      <p className="mt-3 max-w-lg text-sm text-off-white/70">
        Pay with your credit balance for on-demand GPU time to train and
        evaluate your models.
      </p>

      <div className="mt-10">
        <GpuCatalog />
      </div>

      <div className="mt-10">
        <p className="text-technical text-xs text-sand mb-4">MY SESSIONS</p>
        <div className="overflow-hidden rounded-sm border border-border">
          {sessions.length === 0 ? (
            <p className="bg-panel px-5 py-8 text-center text-sm text-text-muted">
              No GPU sessions yet.
            </p>
          ) : (
            <table className="w-full text-sm">
              <tbody>
                {sessions.map((s) => (
                  <tr key={s.id} className="border-b border-border last:border-0 bg-panel">
                    <td className="px-5 py-3.5 text-off-white/80">{s.gpuType}</td>
                    <td className="px-5 py-3.5 text-text-muted">{s.hours}h</td>
                    <td className="px-5 py-3.5 text-technical text-right">
                      {s.pricePaid}{TOKEN_SYMBOL}
                    </td>
                    <td className="px-5 py-3.5 text-right text-text-muted whitespace-nowrap">
                      {formatDate(s.startedAt)}
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
