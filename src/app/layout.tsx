import type { Metadata } from "next";
import { Bebas_Neue, Barlow, Space_Mono } from "next/font/google";
import "./globals.css";

import { Providers } from "@/components/providers/Providers";

const bebasNeue = Bebas_Neue({
  variable: "--font-bebas-neue",
  subsets: ["latin"],
  weight: "400",
});

const barlow = Barlow({
  variable: "--font-barlow",
  subsets: ["latin"],
  weight: ["300", "400", "500", "600", "700"],
});

const spaceMono = Space_Mono({
  variable: "--font-space-mono",
  subsets: ["latin"],
  weight: ["400", "700"],
});

export const metadata: Metadata = {
  title: "Starforge Developer Portal | Buildo Robot",
  description:
    "Train, control, and deploy your Buildo robot. Connect a wallet, rent GPU compute, and buy skills — the developer platform for Starforge's Physical AI robots.",
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html
      lang="en"
      className={`${bebasNeue.variable} ${barlow.variable} ${spaceMono.variable} h-full`}
    >
      <body className="min-h-full flex flex-col bg-black text-off-white antialiased">
        <Providers>{children}</Providers>
      </body>
    </html>
  );
}
