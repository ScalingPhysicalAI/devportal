import { MujocoViewer } from "@/components/simulation/MujocoViewer";

// Full-bleed: DashboardChrome drops the sidebar column and content padding
// for this route (see FULL_BLEED_PREFIXES there), so this fills the entire
// viewport below the navbar. Navigate elsewhere via the topbar's menu
// button or logo -- there's deliberately no in-page chrome here.
export default function SimulatePage() {
  return (
    <div className="h-full w-full bg-black">
      <MujocoViewer interactive className="h-full w-full" />
    </div>
  );
}
