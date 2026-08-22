import clsx from "clsx";

export function Logo({ className }: { className?: string }) {
  return (
    <span className={clsx("inline-flex items-baseline gap-1.5", className)}>
      <span className="text-display text-lg text-off-white">Starforge</span>
      <span className="text-technical text-[10px] text-sand">/dev</span>
    </span>
  );
}
