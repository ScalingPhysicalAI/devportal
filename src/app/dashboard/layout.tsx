import { redirect } from "next/navigation";

import { getCurrentUser } from "@/lib/auth";
import { DashboardChrome } from "@/components/dashboard/DashboardChrome";
import { OnboardingProvider } from "@/components/onboarding/OnboardingProvider";
import { OnboardingOverlay } from "@/components/onboarding/OnboardingOverlay";

export default async function DashboardLayout({ children }: { children: React.ReactNode }) {
  const user = await getCurrentUser();
  if (!user) {
    redirect("/login?next=/dashboard");
  }

  return (
    <OnboardingProvider startAutomatically={!user.onboardingCompletedAt}>
      <DashboardChrome user={user}>{children}</DashboardChrome>
      <OnboardingOverlay />
    </OnboardingProvider>
  );
}
