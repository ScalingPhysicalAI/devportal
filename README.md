# Starforge Developer Portal

The developer platform for **Buildo**, Starforge Robotics' Physical AI robot.
Developers and researchers sign up, connect a wallet, train Buildo, rent GPU
compute, and buy skills — paid for in STARFORGE (SFT) tokens.

Full-stack Next.js app (App Router) — one deployable unit for both the
frontend and the API.

## What's live vs. simulated right now

| Feature | Status |
|---|---|
| Signup / login / logout | **Live.** Real auth (bcrypt + signed JWT session cookie), real Postgres/SQLite-backed accounts. |
| Welcome email on signup | **Live.** Sends via nodemailer; falls back to a free Ethereal test inbox (preview URL logged to console) if no SMTP is configured. |
| Wallet connect | **Live.** Real browser wallet connection (MetaMask etc. via wagmi), address is linked to the account in the DB. |
| STARFORGE token balance | **Live, but off-chain.** A `tokenBalance` column in the DB — credited 20 SFT on first wallet connect, debited on GPU rental / skill purchase. This is a real ledger (`TokenTransaction` table), just not on-chain yet. |
| GPU rental / skills marketplace | **Live.** Real DB transactions debit the token balance; no real GPU is provisioned and no skill is actually deployed (there's no paired robot hardware yet). |
| Robot pairing / telemetry | **UI preview only** — no hardware integration yet. |
| Train on your own dataset | **UI only** — file picker works client-side, no upload/training pipeline yet. |
| On-chain ETH/token airdrop | **Not wired up yet, intentionally deferred.** See below. |

## Stack

- **Next.js 16** (App Router, TypeScript, Tailwind CSS v4)
- **Prisma + SQLite** for local dev (schema is Postgres-ready for AWS RDS — see below)
- **Auth**: bcrypt password hashing + `jose`-signed JWT session cookie (no third-party auth service)
- **Wallet connect**: `wagmi` + `viem`, injected connector (MetaMask, Rabby, etc.)
- **Email**: `nodemailer`, pluggable SMTP (AWS SES-ready)
- **Data fetching**: `@tanstack/react-query` on the client, direct Prisma queries in Server Components

## Getting started

```bash
npm install
cp .env.example .env   # already done in this checkout; edit values as needed
npx prisma migrate dev # creates prisma/dev.db
npm run dev
```

Open http://localhost:3000. Sign up, then connect a wallet from the
dashboard (needs a browser extension wallet like MetaMask installed) to see
the 20 SFT reward credit.

No SMTP setup is required to try the welcome email locally — the first time
one is sent, the server logs a line like:

```
[mailer] Welcome email preview: https://ethereal.email/message/...
```

Open that link to see the actual email that was sent.

## Environment variables

See `.env.example` for the full list. The only one required to run locally
is `AUTH_SECRET` (already generated in `.env`).

## Moving to AWS

- **Database**: swap `prisma/schema.prisma`'s datasource `provider` to
  `"postgresql"` and point `DATABASE_URL` at an RDS Postgres instance, then
  run `npx prisma migrate deploy`.
- **Email**: set `SMTP_HOST` / `SMTP_PORT` / `SMTP_USER` / `SMTP_PASS` to an
  AWS SES SMTP endpoint (or any SMTP provider) — no code changes needed.
- **Secrets**: put `AUTH_SECRET`, `DATABASE_URL`, and SMTP credentials in AWS
  Secrets Manager / Parameter Store, not in a committed `.env`.
- **App hosting**: this is a standard Next.js app — deploy via AWS Amplify
  Hosting, or containerize (`next build && next start`) and run on
  ECS/Fargate behind an ALB.

## Wiring up the real on-chain airdrop (next phase)

The schema already has what's needed to plug this in without a data model
change:

- `TokenTransaction.txHash` and `.status` are already there, unused today —
  fill them in once a transaction is actually broadcast.
- The reward-crediting logic lives in `src/app/api/wallet/connect/route.ts`.
  Today it just increments `User.tokenBalance` in a DB transaction; the real
  version would additionally call an ERC-20 `transfer` from a treasury
  wallet to `User.walletAddress`, store the resulting `txHash`, and only
  mark the `TokenTransaction` `COMPLETED` once it confirms.
- Recommended path: deploy a simple ERC-20 on **Sepolia testnet** first
  (zero financial risk), get the flow working end-to-end, then move to
  mainnet with a funded treasury wallet. Keep `TREASURY_PRIVATE_KEY` out of
  the app entirely if possible — call out to a small signer service or use
  AWS KMS/Secrets Manager, never commit it or put it in a client-reachable
  env var.

## Project structure

```
prisma/schema.prisma          User, TokenTransaction, GpuSession, SkillOrder
src/lib/auth.ts               session cookies, password hashing
src/lib/mailer.ts             welcome email
src/lib/web3-config.ts        wagmi chains/connectors
src/lib/constants.ts          GPU catalog, skills catalog, reward amount
src/app/(marketing)/          landing, signup, login (public)
src/app/dashboard/            overview, robots, train, gpu, skills (auth-gated)
src/app/api/                  signup, login, logout, me, wallet/connect, gpu/rent, skills/buy
src/proxy.ts                  route protection for /dashboard/* (Next 16's middleware convention)
```
