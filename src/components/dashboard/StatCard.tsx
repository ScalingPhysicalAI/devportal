export function StatCard({
  label,
  value,
  suffix,
}: {
  label: string;
  value: string | number;
  suffix?: string;
}) {
  return (
    <div className="rounded-sm border border-border bg-panel p-5">
      <p className="text-technical text-[11px] text-text-muted uppercase">{label}</p>
      <p className="mt-2 text-display text-3xl text-off-white">
        {value}
        {suffix && <span className="ml-1 text-sm text-text-muted">{suffix}</span>}
      </p>
    </div>
  );
}
