"use client";

import { useEffect, useLayoutEffect, useState } from "react";
import { createPortal } from "react-dom";

import { Button } from "@/components/ui/Button";
import { useOnboarding } from "@/components/onboarding/OnboardingProvider";

const SPOTLIGHT_PADDING = 8;
const CARD_WIDTH = 300;
const CARD_MARGIN = 14;

function findVisibleTarget(id: string): HTMLElement | null {
  const candidates = document.querySelectorAll<HTMLElement>(`[data-tour="${id}"]`);
  for (const el of candidates) {
    const rect = el.getBoundingClientRect();
    if (rect.width > 0 && rect.height > 0) return el;
  }
  return null;
}

export function OnboardingOverlay() {
  const { isOpen, stepIndex, steps, next, back, skip } = useOnboarding();
  const [mounted, setMounted] = useState(false);
  const [rect, setRect] = useState<DOMRect | null>(null);

  const step = steps[stepIndex];

  useEffect(() => setMounted(true), []);

  useLayoutEffect(() => {
    if (!isOpen) return;

    function measure() {
      if (!step.target) {
        setRect(null);
        return;
      }
      const el = findVisibleTarget(step.target);
      setRect(el ? el.getBoundingClientRect() : null);
      el?.scrollIntoView({ block: "center", behavior: "smooth" });
    }

    measure();
    window.addEventListener("resize", measure);
    window.addEventListener("scroll", measure, true);
    const id = window.setInterval(measure, 250);
    return () => {
      window.removeEventListener("resize", measure);
      window.removeEventListener("scroll", measure, true);
      window.clearInterval(id);
    };
  }, [isOpen, step]);

  if (!mounted || !isOpen) return null;

  const isLast = stepIndex === steps.length - 1;

  let cardTop: number;
  let cardLeft: number;
  if (rect) {
    const spaceBelow = window.innerHeight - rect.bottom;
    if (spaceBelow > 200) {
      cardTop = rect.bottom + CARD_MARGIN;
    } else {
      cardTop = Math.max(CARD_MARGIN, rect.top - CARD_MARGIN - 180);
    }
    cardLeft = Math.min(
      Math.max(CARD_MARGIN, rect.left),
      window.innerWidth - CARD_WIDTH - CARD_MARGIN
    );
  } else {
    cardTop = window.innerHeight / 2 - 100;
    cardLeft = window.innerWidth / 2 - CARD_WIDTH / 2;
  }

  return createPortal(
    <div className="fixed inset-0 z-[100]">
      {rect ? (
        <div
          className="pointer-events-none fixed rounded-sm border border-sand transition-all duration-200"
          style={{
            top: rect.top - SPOTLIGHT_PADDING,
            left: rect.left - SPOTLIGHT_PADDING,
            width: rect.width + SPOTLIGHT_PADDING * 2,
            height: rect.height + SPOTLIGHT_PADDING * 2,
            boxShadow: "0 0 0 9999px rgba(0,0,0,0.72)",
          }}
        />
      ) : (
        <div className="fixed inset-0 bg-black/72" />
      )}

      <div
        className="fixed z-[110] animate-fade-up rounded-sm border border-border-strong bg-panel p-5 glow-sand"
        style={{ top: cardTop, left: cardLeft, width: CARD_WIDTH }}
      >
        <div className="flex items-center justify-between">
          <p className="text-technical text-[11px] text-sand">
            {stepIndex + 1} / {steps.length}
          </p>
          <button
            onClick={skip}
            className="text-technical text-[11px] text-text-muted hover:text-off-white cursor-pointer"
          >
            Skip all
          </button>
        </div>

        <p className="mt-3 text-display text-lg text-off-white">{step.title}</p>
        <p className="mt-2 text-sm text-off-white/70">{step.body}</p>

        <div className="mt-5 flex items-center justify-between">
          <Button
            variant="ghost"
            size="sm"
            onClick={back}
            className={stepIndex === 0 ? "invisible" : ""}
          >
            Back
          </Button>
          <Button size="sm" onClick={next}>
            {isLast ? "Done" : "Next"}
          </Button>
        </div>
      </div>
    </div>,
    document.body
  );
}
