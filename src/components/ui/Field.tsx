import type { InputHTMLAttributes, LabelHTMLAttributes, ReactNode, SelectHTMLAttributes } from "react";
import clsx from "clsx";

export function FieldLabel({ className, ...props }: LabelHTMLAttributes<HTMLLabelElement>) {
  return (
    <label
      className={clsx(
        "block text-xs font-medium uppercase tracking-wider text-text-muted mb-2",
        className
      )}
      {...props}
    />
  );
}

export function TextInput({ className, ...props }: InputHTMLAttributes<HTMLInputElement>) {
  return (
    <input
      className={clsx(
        "w-full rounded-sm border border-border bg-panel px-3.5 h-11 text-sm text-off-white placeholder:text-text-muted/70 outline-none transition-colors focus:border-sand",
        className
      )}
      {...props}
    />
  );
}

export function Select({ className, ...props }: SelectHTMLAttributes<HTMLSelectElement>) {
  return (
    <select
      className={clsx(
        "w-full rounded-sm border border-border bg-panel px-3.5 h-11 text-sm text-off-white outline-none transition-colors focus:border-sand appearance-none",
        className
      )}
      {...props}
    />
  );
}

export function FieldError({ children }: { children?: ReactNode }) {
  if (!children) return null;
  return <p className="mt-1.5 text-xs text-error">{children}</p>;
}

export function FieldWrap({ children }: { children: ReactNode }) {
  return <div className="mb-5">{children}</div>;
}
