"use client";

import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import { useAccount, useConnect, useDisconnect } from "wagmi";
import { useMutation, useQueryClient } from "@tanstack/react-query";

import { Button } from "@/components/ui/Button";
import { connectWallet } from "@/lib/api-client";
import { EARLY_WALLET_REWARD, TOKEN_SYMBOL } from "@/lib/constants";
import type { SafeUser } from "@/lib/auth";

function shortAddress(address: string) {
  return `${address.slice(0, 6)}…${address.slice(-4)}`;
}

export function WalletConnectCard({ initialUser }: { initialUser: SafeUser }) {
  const { address, isConnected } = useAccount();
  const { connectors, connect, isPending: isConnecting } = useConnect();
  const { disconnect } = useDisconnect();
  const queryClient = useQueryClient();
  const router = useRouter();

  const [rewardToast, setRewardToast] = useState<number | null>(null);
  const [linkedAddress, setLinkedAddress] = useState<string | null>(
    initialUser.walletAddress
  );
  const [linkError, setLinkError] = useState<string | null>(null);
  const [noWalletFound, setNoWalletFound] = useState(false);

  useEffect(() => {
    setNoWalletFound(!(window as unknown as { ethereum?: unknown }).ethereum);
  }, []);

  const linkMutation = useMutation({
    mutationFn: (addr: string) => connectWallet(addr),
    onSuccess: (data) => {
      queryClient.setQueryData(["me"], data.user);
      setLinkedAddress(data.user.walletAddress);
      setLinkError(null);
      if (data.rewardGranted > 0) {
        setRewardToast(data.rewardGranted);
        setTimeout(() => setRewardToast(null), 5000);
      }
      // Re-fetch server-rendered data (overview stat cards, recent activity)
      // so the new balance/transaction shows without a manual page reload.
      router.refresh();
    },
    onError: (err: Error) => setLinkError(err.message),
  });

  // Once a wallet is connected in the browser, link it to the account
  // (idempotent server-side — reward is only granted the first time).
  useEffect(() => {
    if (isConnected && address && linkedAddress?.toLowerCase() !== address.toLowerCase()) {
      linkMutation.mutate(address);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isConnected, address]);

  const injectedConnector = connectors[0];

  return (
    <div className="relative rounded-sm border border-border-strong bg-panel p-7">
      {rewardToast !== null && (
        <div className="absolute -top-3 right-6 rounded-sm border border-success/40 bg-black px-3 py-1.5 text-xs text-success animate-fade-up">
          + {rewardToast} {TOKEN_SYMBOL} added to your account
        </div>
      )}

      <p className="text-technical text-xs text-sand mb-3">WALLET</p>

      {linkedAddress ? (
        <>
          <p className="text-display text-2xl text-off-white">Wallet connected</p>
          <p className="mt-2 text-technical text-sm text-off-white/70">
            {shortAddress(linkedAddress)}
          </p>
          <p className="mt-4 text-sm text-text-muted max-w-sm">
            Your early-signup reward has been credited. Balances update
            instantly as you spend {TOKEN_SYMBOL} across the portal.
          </p>
          {isConnected && (
            <Button
              variant="ghost"
              size="sm"
              className="mt-5"
              onClick={() => disconnect()}
            >
              Disconnect
            </Button>
          )}
        </>
      ) : (
        <>
          <p className="text-display text-2xl text-off-white">Connect your wallet</p>
          <p className="mt-2 text-sm text-text-muted max-w-sm">
            Link an Ethereum wallet to claim your {EARLY_WALLET_REWARD}{" "}
            {TOKEN_SYMBOL} early-developer reward.
          </p>
          <Button
            data-tour="connect-wallet"
            className="mt-5"
            disabled={isConnecting || linkMutation.isPending}
            onClick={() => injectedConnector && connect({ connector: injectedConnector })}
          >
            {isConnecting || linkMutation.isPending ? "Connecting…" : "Connect wallet"}
          </Button>
          {noWalletFound && (
            <p className="mt-3 text-xs text-text-muted">
              No browser wallet detected. Install MetaMask or another
              injected wallet to continue.
            </p>
          )}
          {linkError && <p className="mt-3 text-xs text-error">{linkError}</p>}
        </>
      )}
    </div>
  );
}
