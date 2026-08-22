"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useMutation, useQueryClient } from "@tanstack/react-query";

import { Button } from "@/components/ui/Button";
import { rentGpu } from "@/lib/api-client";
import { GPU_CATALOG, TOKEN_SYMBOL, type GpuType } from "@/lib/constants";

export function GpuCatalog() {
  const router = useRouter();
  const queryClient = useQueryClient();
  const [hours, setHours] = useState<Record<GpuType, number>>({
    "RTX 4090": 4,
    "A100 80GB": 4,
    H100: 4,
  });
  const [error, setError] = useState<string | null>(null);
  const [renting, setRenting] = useState<GpuType | null>(null);

  const mutation = useMutation({
    mutationFn: ({ gpuType, h }: { gpuType: GpuType; h: number }) => rentGpu(gpuType, h),
    onMutate: ({ gpuType }) => {
      setError(null);
      setRenting(gpuType);
    },
    onSuccess: (data) => {
      queryClient.setQueryData(["me"], data.user);
      router.refresh();
    },
    onError: (err: Error) => setError(err.message),
    onSettled: () => setRenting(null),
  });

  return (
    <div>
      <div className="grid gap-5 sm:grid-cols-3">
        {GPU_CATALOG.map((gpu) => (
          <div key={gpu.type} className="flex flex-col rounded-sm border border-border bg-panel p-6">
            <p className="text-display text-xl text-off-white">{gpu.type}</p>
            <p className="text-technical text-xs text-sand mt-1">{gpu.vram} VRAM</p>
            <p className="mt-3 text-sm text-text-muted flex-1">{gpu.desc}</p>
            <p className="mt-4 text-technical text-lg text-off-white">
              {gpu.pricePerHour} {TOKEN_SYMBOL}
              <span className="text-sm text-text-muted"> /hr</span>
            </p>

            <div className="mt-4 flex items-center gap-2">
              <input
                type="number"
                min={1}
                max={336}
                value={hours[gpu.type]}
                onChange={(e) =>
                  setHours((prev) => ({
                    ...prev,
                    [gpu.type]: Math.max(1, Number(e.target.value) || 1),
                  }))
                }
                className="w-20 rounded-sm border border-border bg-black px-2.5 h-9 text-sm text-off-white outline-none focus:border-sand"
              />
              <span className="text-xs text-text-muted">hours</span>
            </div>

            <Button
              className="mt-4"
              disabled={mutation.isPending}
              onClick={() => mutation.mutate({ gpuType: gpu.type, h: hours[gpu.type] })}
            >
              {renting === gpu.type && mutation.isPending
                ? "Renting…"
                : `Rent for ${gpu.pricePerHour * hours[gpu.type]} ${TOKEN_SYMBOL}`}
            </Button>
          </div>
        ))}
      </div>

      {error && <p className="mt-4 text-sm text-error">{error}</p>}
    </div>
  );
}
