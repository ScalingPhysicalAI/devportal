-- AlterEnum
ALTER TYPE "TransactionType" ADD VALUE 'EMAIL_VERIFY_BONUS';

-- AlterTable
ALTER TABLE "User" ADD COLUMN "emailVerified" BOOLEAN NOT NULL DEFAULT false,
ADD COLUMN "emailVerifyExpiry" TIMESTAMP(3),
ADD COLUMN "emailVerifyToken" TEXT;

-- CreateIndex
CREATE UNIQUE INDEX "User_emailVerifyToken_key" ON "User"("emailVerifyToken");
