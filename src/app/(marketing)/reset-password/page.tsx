"use client";

import { Suspense, useState } from "react";
import type { FormEvent } from "react";
import { useRouter, useSearchParams } from "next/navigation";
import Link from "next/link";

import { Button } from "@/components/ui/Button";
import { Container } from "@/components/ui/Container";
import { FieldError, FieldLabel, FieldWrap, TextInput } from "@/components/ui/Field";

function ResetPasswordForm() {
  const router = useRouter();
  const token = useSearchParams().get("token") ?? "";
  const [password, setPassword] = useState("");
  const [confirmPassword, setConfirmPassword] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  async function onSubmit(e: FormEvent) {
    e.preventDefault();
    setError(null);

    if (password !== confirmPassword) {
      setError("Passwords don't match");
      return;
    }

    setLoading(true);
    try {
      const res = await fetch("/api/auth/reset-password", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ token, password }),
      });
      const data = await res.json();

      if (!res.ok) {
        setError(data.error ?? "Something went wrong");
        setLoading(false);
        return;
      }

      router.push("/dashboard?reset=1");
      router.refresh();
    } catch {
      setError("Network error — please try again");
      setLoading(false);
    }
  }

  if (!token) {
    return (
      <p className="text-sm text-off-white/80">
        This reset link is missing its token. Request a new one from the{" "}
        <Link href="/forgot-password" className="text-sand hover:underline">
          forgot password
        </Link>{" "}
        page.
      </p>
    );
  }

  return (
    <form onSubmit={onSubmit}>
      <FieldWrap>
        <FieldLabel htmlFor="password">New password</FieldLabel>
        <TextInput
          id="password"
          name="password"
          type="password"
          autoComplete="new-password"
          required
          minLength={8}
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="At least 8 characters"
        />
      </FieldWrap>

      <FieldWrap>
        <FieldLabel htmlFor="confirmPassword">Confirm new password</FieldLabel>
        <TextInput
          id="confirmPassword"
          name="confirmPassword"
          type="password"
          autoComplete="new-password"
          required
          minLength={8}
          value={confirmPassword}
          onChange={(e) => setConfirmPassword(e.target.value)}
          placeholder="Re-enter password"
        />
      </FieldWrap>

      <FieldError>{error}</FieldError>

      <Button type="submit" disabled={loading} className="mt-2 w-full">
        {loading ? "Resetting…" : "Reset password"}
      </Button>
    </form>
  );
}

export default function ResetPasswordPage() {
  return (
    <section className="bg-grid py-20 sm:py-28">
      <Container className="max-w-md">
        <p className="text-technical text-xs text-sand mb-4">RESET PASSWORD</p>
        <h1 className="text-display text-4xl text-off-white">Choose a new password</h1>
        <p className="mt-3 text-sm text-off-white/70">
          Pick a new password for your Starforge account.
        </p>

        <div className="mt-10 rounded-sm border border-border bg-panel p-7 glow-sand">
          <Suspense fallback={null}>
            <ResetPasswordForm />
          </Suspense>
        </div>
      </Container>
    </section>
  );
}
