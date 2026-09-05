"use client";

import { useState } from "react";
import type { FormEvent } from "react";
import Link from "next/link";

import { Button } from "@/components/ui/Button";
import { Container } from "@/components/ui/Container";
import { FieldError, FieldLabel, FieldWrap, TextInput } from "@/components/ui/Field";

export default function ForgotPasswordPage() {
  const [email, setEmail] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [sent, setSent] = useState(false);

  async function onSubmit(e: FormEvent) {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      const res = await fetch("/api/auth/forgot-password", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ email }),
      });
      const data = await res.json();

      if (!res.ok) {
        setError(data.error ?? "Something went wrong");
        setLoading(false);
        return;
      }

      setSent(true);
    } catch {
      setError("Network error — please try again");
    } finally {
      setLoading(false);
    }
  }

  return (
    <section className="bg-grid py-20 sm:py-28">
      <Container className="max-w-md">
        <p className="text-technical text-xs text-sand mb-4">RESET PASSWORD</p>
        <h1 className="text-display text-4xl text-off-white">Forgot password?</h1>
        <p className="mt-3 text-sm text-off-white/70">
          Enter the email on your account and we&apos;ll send a link to reset your password.
        </p>

        <div className="mt-10 rounded-sm border border-border bg-panel p-7 glow-sand">
          {sent ? (
            <p className="text-sm text-off-white/80">
              If an account exists for <span className="text-sand">{email}</span>, we&apos;ve sent
              a reset link. Check your inbox.
            </p>
          ) : (
            <form onSubmit={onSubmit}>
              <FieldWrap>
                <FieldLabel htmlFor="email">Email</FieldLabel>
                <TextInput
                  id="email"
                  name="email"
                  type="email"
                  autoComplete="email"
                  required
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  placeholder="you@example.com"
                />
              </FieldWrap>

              <FieldError>{error}</FieldError>

              <Button type="submit" disabled={loading} className="mt-2 w-full">
                {loading ? "Sending…" : "Send reset link"}
              </Button>
            </form>
          )}

          <p className="mt-5 text-center text-xs text-text-muted">
            Remembered it?{" "}
            <Link href="/login" className="text-sand hover:underline">
              Log in
            </Link>
          </p>
        </div>
      </Container>
    </section>
  );
}
