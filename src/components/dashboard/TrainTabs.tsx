"use client";

import { useRef, useState } from "react";
import clsx from "clsx";

import { Button } from "@/components/ui/Button";
import { PRETRAINED_MODELS } from "@/lib/constants";

export function TrainTabs() {
  const [tab, setTab] = useState<"pretrained" | "dataset">("pretrained");
  const [files, setFiles] = useState<File[]>([]);
  const [dragging, setDragging] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);

  return (
    <div>
      <div className="flex gap-1 border-b border-border">
        {(
          [
            { id: "pretrained", label: "Pre-trained models" },
            { id: "dataset", label: "Your own dataset" },
          ] as const
        ).map((t) => (
          <button
            key={t.id}
            onClick={() => setTab(t.id)}
            className={clsx(
              "px-4 py-3 text-sm border-b-2 -mb-px transition-colors",
              tab === t.id
                ? "border-sand text-sand"
                : "border-transparent text-off-white/60 hover:text-off-white"
            )}
          >
            {t.label}
          </button>
        ))}
      </div>

      {tab === "pretrained" ? (
        <div className="mt-8 grid gap-5 sm:grid-cols-2 lg:grid-cols-3">
          {PRETRAINED_MODELS.map((m) => (
            <div key={m.id} className="rounded-sm border border-border bg-panel p-6 flex flex-col">
              <p className="text-display text-xl text-off-white">{m.name}</p>
              <p className="mt-1 text-technical text-xs text-sand">{m.params} params</p>
              <p className="mt-3 text-sm text-text-muted flex-1">{m.desc}</p>
              <Button variant="secondary" size="sm" className="mt-5" disabled>
                Deploy to robot — pair a robot first
              </Button>
            </div>
          ))}
        </div>
      ) : (
        <div className="mt-8">
          <div
            onDragOver={(e) => {
              e.preventDefault();
              setDragging(true);
            }}
            onDragLeave={() => setDragging(false)}
            onDrop={(e) => {
              e.preventDefault();
              setDragging(false);
              setFiles((prev) => [...prev, ...Array.from(e.dataTransfer.files)]);
            }}
            onClick={() => inputRef.current?.click()}
            className={clsx(
              "cursor-pointer rounded-sm border border-dashed px-8 py-16 text-center transition-colors",
              dragging ? "border-sand bg-panel-raised" : "border-border-strong bg-panel"
            )}
          >
            <p className="text-display text-2xl text-off-white">
              Drop dataset files, or click to browse
            </p>
            <p className="mx-auto mt-2 max-w-sm text-sm text-text-muted">
              Episode logs (.hdf5, .mcap) or video captures from the data
              collection SDK.
            </p>
            <input
              ref={inputRef}
              type="file"
              multiple
              className="hidden"
              onChange={(e) =>
                setFiles((prev) => [...prev, ...Array.from(e.target.files ?? [])])
              }
            />
          </div>

          {files.length > 0 && (
            <div className="mt-5 overflow-hidden rounded-sm border border-border">
              {files.map((f, i) => (
                <div
                  key={`${f.name}-${i}`}
                  className="flex items-center justify-between border-b border-border bg-panel px-5 py-3 text-sm last:border-0"
                >
                  <span className="text-off-white/80">{f.name}</span>
                  <span className="text-technical text-xs text-text-muted">
                    {(f.size / 1024).toFixed(0)} KB
                  </span>
                </div>
              ))}
            </div>
          )}

          <Button className="mt-6" disabled={files.length === 0}>
            Start training run — coming soon
          </Button>
        </div>
      )}
    </div>
  );
}
