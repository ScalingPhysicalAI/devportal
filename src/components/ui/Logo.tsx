import Image from "next/image";
import clsx from "clsx";

import wordmark from "@/app/wordmark-removebg-preview.png";

export function Logo({ className }: { className?: string }) {
  return (
    <span className={clsx("inline-flex items-center gap-2", className)}>
      <Image
        src={wordmark}
        alt="Starforge"
        priority
        className="h-11 w-auto"
      />
      <span className="text-technical text-[10px] text-sand">/dev</span>
    </span>
  );
}
