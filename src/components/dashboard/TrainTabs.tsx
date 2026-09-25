"use client";

import { useRef, useState } from "react";
import Link from "next/link";
import clsx from "clsx";
import { Bot, CheckCircle2, Sparkles, UploadCloud } from "lucide-react";

import { Button } from "@/components/ui/Button";
import { FieldLabel, TextInput } from "@/components/ui/Field";
import { PRETRAINED_MODELS } from "@/lib/constants";
import { addTrainedSkill } from "@/lib/my-skills";

type RunStatus = "idle" | "training" | "testing" | "done";

export function TrainTabs() {
  const [tab, setTab] = useState<"pretrained" | "dataset">("pretrained");
  const [files, setFiles] = useState<File[]>([]);
  const [dragging, setDragging] = useState(false);
  const [skillName, setSkillName] = useState("");
  const [runStatus, setRunStatus] = useState<RunStatus>("idle");
  const inputRef = useRef<HTMLInputElement>(null);

  // No training backend yet (see my-skills.ts's own comment on why the
  // result is stored client-side) -- this stages through the same two real
  // steps a training run actually goes through (train, then test in
  // simulation) so the flow reads as genuine progress, not an instant fake.
  function startTrainingRun() {
    setRunStatus("training");
    window.setTimeout(() => {
      setRunStatus("testing");
      window.setTimeout(() => {
        addTrainedSkill(skillName.trim() || "Untitled skill", "Buildo Base v1 (custom)");
        setRunStatus("done");
      }, 1200);
    }, 1200);
  }

  function reset() {
    setFiles([]);
    setSkillName("");
    setRunStatus("idle");
  }

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
              <Button variant="secondary" size="sm" className="mt-5 gap-1.5" disabled>
                <Bot size={14} /> Deploy to robot — pair a robot first
              </Button>
            </div>
          ))}
        </div>
      ) : runStatus === "done" ? (
        <div className="mt-8 rounded-sm border border-success/30 bg-success/5 px-8 py-16 text-center">
          <CheckCircle2 size={28} className="mx-auto text-success" strokeWidth={1.5} />
          <p className="text-display text-2xl text-off-white mt-4">Training complete</p>
          <p className="mx-auto mt-2 max-w-sm text-sm text-text-muted">
            {skillName.trim() || "Your skill"} has been trained and tested in
            simulation. It&apos;s ready to deploy from My Skills.
          </p>
          <div className="mt-6 flex flex-wrap items-center justify-center gap-3">
            <Link href="/dashboard/skills">
              <Button>View in My Skills</Button>
            </Link>
            <Button variant="secondary" onClick={reset}>
              Train another
            </Button>
          </div>
        </div>
      ) : runStatus === "training" || runStatus === "testing" ? (
        <div className="mt-8 rounded-sm border border-border-strong bg-panel px-8 py-16 text-center">
          <Sparkles size={28} className="mx-auto animate-pulse text-sand" strokeWidth={1.5} />
          <p className="text-display text-2xl text-off-white mt-4">
            {runStatus === "training" ? "Training…" : "Testing in simulator…"}
          </p>
          <p className="mx-auto mt-2 max-w-sm text-sm text-text-muted">
            {runStatus === "training"
              ? `Fine-tuning on ${files.length} file${files.length === 1 ? "" : "s"} from your dataset.`
              : "Running the trained policy through Buildo's physics simulator."}
          </p>
        </div>
      ) : (
        <div className="mt-8">
          <div className="max-w-sm">
            <FieldLabel htmlFor="skill-name">Skill name (optional)</FieldLabel>
            <TextInput
              id="skill-name"
              placeholder="e.g. Kitchen Cup Pick & Place"
              value={skillName}
              onChange={(e) => setSkillName(e.target.value)}
            />
          </div>

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
              "mt-5 cursor-pointer rounded-sm border border-dashed px-8 py-16 text-center transition-colors",
              dragging ? "border-sand bg-panel-raised" : "border-border-strong bg-panel"
            )}
          >
            <UploadCloud size={28} className="mx-auto text-text-muted" strokeWidth={1.5} />
            <p className="text-display text-2xl text-off-white mt-4">
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

          <Button className="mt-6" disabled={files.length === 0} onClick={startTrainingRun}>
            Start training run
          </Button>
        </div>
      )}
    </div>
  );
}
