"use client";

import { createContext, useContext, useEffect, useRef, useState, type ReactNode } from "react";
import { useMutation, useQueryClient } from "@tanstack/react-query";

import { completeOnboarding } from "@/lib/api-client";
import { onboardingSteps } from "@/lib/onboarding-steps";

type OnboardingContextValue = {
  isOpen: boolean;
  stepIndex: number;
  steps: typeof onboardingSteps;
  start: () => void;
  next: () => void;
  back: () => void;
  skip: () => void;
};

const OnboardingContext = createContext<OnboardingContextValue | null>(null);

export function useOnboarding() {
  const ctx = useContext(OnboardingContext);
  if (!ctx) throw new Error("useOnboarding must be used within OnboardingProvider");
  return ctx;
}

export function OnboardingProvider({
  children,
  startAutomatically,
}: {
  children: ReactNode;
  /** True when the signed-in user has never completed the tour. */
  startAutomatically: boolean;
}) {
  const [isOpen, setIsOpen] = useState(false);
  const [stepIndex, setStepIndex] = useState(0);
  const hasCompletedRef = useRef(!startAutomatically);
  const queryClient = useQueryClient();

  const completeMutation = useMutation({
    mutationFn: completeOnboarding,
    onSuccess: (data) => queryClient.setQueryData(["me"], data.user),
  });

  useEffect(() => {
    if (startAutomatically) {
      setStepIndex(0);
      setIsOpen(true);
    }
    // Only ever auto-start once, on first mount after signup.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  function close() {
    setIsOpen(false);
    if (!hasCompletedRef.current) {
      hasCompletedRef.current = true;
      completeMutation.mutate();
    }
  }

  const value: OnboardingContextValue = {
    isOpen,
    stepIndex,
    steps: onboardingSteps,
    start: () => {
      setStepIndex(0);
      setIsOpen(true);
    },
    next: () => {
      setStepIndex((i) => {
        if (i >= onboardingSteps.length - 1) {
          close();
          return i;
        }
        return i + 1;
      });
    },
    back: () => setStepIndex((i) => Math.max(0, i - 1)),
    skip: close,
  };

  return <OnboardingContext.Provider value={value}>{children}</OnboardingContext.Provider>;
}
