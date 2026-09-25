import type { LucideIcon } from "lucide-react";

export function StatCard({
  label,
  value,
  suffix,
  icon: Icon,
}: {
  label: string;
  value: string | number;
  suffix?: string;
  icon?: LucideIcon;
}) {
  return (
    <div className="rounded-sm border border-border bg-panel p-5">
      <div className="flex items-center justify-between">
        <p className="text-technical text-[11px] text-text-muted uppercase">{label}</p>
        {Icon && <Icon size={15} className="text-sand/70" strokeWidth={1.75} />}
      </div>
      <p className="mt-2 text-display text-3xl text-off-white">
        {value}
        {suffix && <span className="ml-1 text-sm text-text-muted">{suffix}</span>}
      </p>
    </div>
  );
}
