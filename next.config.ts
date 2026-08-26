import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  experimental: {
    // Turbopack's on-disk dev cache repeatedly corrupts on this repo's
    // filesystem (Rust panics: "Unable to open static sorted file", "range
    // start index out of range") and takes the whole dev server down with
    // it. Keeping the cache in memory instead avoids the crash.
    turbopackFileSystemCacheForDev: false,
  },
};

export default nextConfig;
