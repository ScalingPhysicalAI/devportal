const TRANSACTION_LABELS: Record<string, string> = {
  SIGNUP_BONUS: "Signup bonus",
  WALLET_CONNECT_BONUS: "Wallet connect reward",
  GPU_RENTAL: "GPU rental",
  SKILL_PURCHASE: "Skill purchase",
  AIRDROP: "Airdrop",
};

export function transactionLabel(type: string) {
  return TRANSACTION_LABELS[type] ?? type;
}

export function formatDate(date: Date | string) {
  return new Date(date).toLocaleDateString("en-US", {
    month: "short",
    day: "numeric",
    hour: "2-digit",
    minute: "2-digit",
  });
}
