-- Revert: back to STARFORGE token (SFT) balance instead of USDC credits.
ALTER TABLE "User" RENAME COLUMN "creditBalance" TO "tokenBalance";
