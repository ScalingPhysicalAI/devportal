"use client";

import { useState } from "react";
import type { FormEvent } from "react";
import { useRouter } from "next/navigation";
import Link from "next/link";

import { Button } from "@/components/ui/Button";
import { Container } from "@/components/ui/Container";
import { FieldError, FieldLabel, FieldWrap, TextInput } from "@/components/ui/Field";

export default function LoginPage() {
  const router = useRouter();
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  async function onSubmit(e: FormEvent) {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      const res = await fetch("/api/auth/login", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ email, password }),
      });
      const data = await res.json();

      if (!res.ok) {
        setError(data.error ?? "Something went wrong");
        setLoading(false);
        return;
      }

      router.push("/dashboard");
      router.refresh();
    } catch {
      setError("Network error — please try again");
      setLoading(false);
    }
  }

  return (
    <section className="bg-grid py-20 sm:py-28">
      <Container className="max-w-md">
        <p className="text-technical text-xs text-sand mb-4">WELCOME BACK</p>
        <h1 className="text-display text-4xl text-off-white">Log in</h1>
        <p className="mt-3 text-sm text-off-white/70">
          Access your dashboard, robots, and credit balance.
        </p>

        <form
          onSubmit={onSubmit}
          className="mt-10 rounded-sm border border-border bg-panel p-7 glow-sand"
        >
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

          <FieldWrap>
            <FieldLabel htmlFor="password">Password</FieldLabel>
            <TextInput
              id="password"
              name="password"
              type="password"
              autoComplete="current-password"
              required
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
            />
          </FieldWrap>

          <div className="-mt-2 mb-5 text-right">
            <Link href="/forgot-password" className="text-xs text-sand hover:underline">
              Forgot password?
            </Link>
          </div>

          <FieldError>{error}</FieldError>

          <Button type="submit" disabled={loading} className="mt-2 w-full">
            {loading ? "Logging in…" : "Log in"}
          </Button>

          <p className="mt-5 text-center text-xs text-text-muted">
            Don&apos;t have an account?{" "}
            <Link href="/signup" className="text-sand hover:underline">
              Sign up
            </Link>
          </p>
        </form>
      </Container>
    </section>
  );
}
