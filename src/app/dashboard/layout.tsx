import { redirect } from "next/navigation";

import { getCurrentUser } from "@/lib/auth";
import { TopBar } from "@/components/dashboard/TopBar";
import { Sidebar } from "@/components/dashboard/Sidebar";
import { MobileTabs } from "@/components/dashboard/MobileTabs";
import { OnboardingProvider } from "@/components/onboarding/OnboardingProvider";
import { OnboardingOverlay } from "@/components/onboarding/OnboardingOverlay";

export default async function DashboardLayout({ children }: { children: React.ReactNode }) {
  const user = await getCurrentUser();
  if (!user) {
    redirect("/login?next=/dashboard");
  }

  return (
    <OnboardingProvider startAutomatically={!user.onboardingCompletedAt}>
      <div className="flex min-h-screen flex-col">
        <TopBar user={user} />
        <MobileTabs />
        <div className="flex flex-1">
          <Sidebar />
          <main className="flex-1 px-6 py-10 lg:px-10">{children}</main>
        </div>
      </div>
      <OnboardingOverlay />
    </OnboardingProvider>
  );
}
