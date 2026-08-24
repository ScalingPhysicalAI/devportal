"use client";

import { useState } from "react";
import type { FormEvent } from "react";
import { useRouter } from "next/navigation";
import Link from "next/link";

import { Button } from "@/components/ui/Button";
import { Container } from "@/components/ui/Container";
import { FieldError, FieldLabel, FieldWrap, Select, TextInput } from "@/components/ui/Field";

export default function SignupPage() {
  const router = useRouter();
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [role, setRole] = useState<"DEVELOPER" | "RESEARCHER">("DEVELOPER");
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  async function onSubmit(e: FormEvent) {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      const res = await fetch("/api/auth/signup", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, email, password, role }),
      });
      const data = await res.json();

      if (!res.ok) {
        setError(data.error ?? "Something went wrong");
        setLoading(false);
        return;
      }

      router.push("/dashboard?welcome=1");
      router.refresh();
    } catch {
      setError("Network error — please try again");
      setLoading(false);
    }
  }

  return (
    <section className="bg-grid py-20 sm:py-28">
      <Container className="max-w-md">
        <p className="text-technical text-xs text-sand mb-4">CREATE ACCOUNT</p>
        <h1 className="text-display text-4xl text-off-white">
          Join the developer portal
        </h1>
        <p className="mt-3 text-sm text-off-white/70">
          Sign up to start training Buildo, renting GPUs, and earning $20
          USDC credit.
        </p>

        <form
          onSubmit={onSubmit}
          className="mt-10 rounded-sm border border-border bg-panel p-7 glow-sand"
        >
          <FieldWrap>
            <FieldLabel htmlFor="name">Full name</FieldLabel>
            <TextInput
              id="name"
              name="name"
              autoComplete="name"
              required
              minLength={2}
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="Ada Lovelace"
            />
          </FieldWrap>

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
              autoComplete="new-password"
              required
              minLength={8}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="At least 8 characters"
            />
          </FieldWrap>

          <FieldWrap>
            <FieldLabel htmlFor="role">I am a</FieldLabel>
            <Select
              id="role"
              name="role"
              value={role}
              onChange={(e) => setRole(e.target.value as "DEVELOPER" | "RESEARCHER")}
            >
              <option value="DEVELOPER">Developer</option>
              <option value="RESEARCHER">Researcher</option>
            </Select>
          </FieldWrap>

          <FieldError>{error}</FieldError>

          <Button type="submit" disabled={loading} className="mt-2 w-full">
            {loading ? "Creating account…" : "Create account"}
          </Button>

          <p className="mt-5 text-center text-xs text-text-muted">
            Already have an account?{" "}
            <Link href="/login" className="text-sand hover:underline">
              Log in
            </Link>
          </p>
        </form>
      </Container>
    </section>
  );
}
