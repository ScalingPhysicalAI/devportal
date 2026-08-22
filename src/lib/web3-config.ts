import { createConfig, http, injected } from "wagmi";
import { mainnet, sepolia } from "wagmi/chains";

// Injected wallets (MetaMask, Rabby, Coinbase extension, etc.) need no API
// key. Sepolia is included so the portal is ready to demo the future token
// airdrop on a free testnet before any real funds are involved.
export const wagmiConfig = createConfig({
  chains: [mainnet, sepolia],
  connectors: [injected()],
  transports: {
    [mainnet.id]: http(),
    [sepolia.id]: http(),
  },
  ssr: true,
});
