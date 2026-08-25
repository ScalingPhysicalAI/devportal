-- Rename tokenBalance -> creditBalance: the balance is USD-denominated
-- (USDC-backed) credit, not a STARFORGE token amount. Column rename
-- preserves existing data.
ALTER TABLE "User" RENAME COLUMN "tokenBalance" TO "creditBalance";
