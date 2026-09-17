import { MujocoViewer } from "@/components/simulation/MujocoViewer";

// Full-bleed: DashboardChrome drops the sidebar column and content padding
// for this route (see FULL_BLEED_PREFIXES there), so this fills the entire
// viewport below the navbar. Navigate elsewhere via the topbar's menu
// button or logo -- there's deliberately no in-page chrome here.
export default function SimulatePage() {
  return (
    // bg-[#f3efe4] matches MujocoViewer's own scene.background (its bright
    // kitchen theme, see that file's comment) -- this is what shows through
    // for an instant before the WebGL canvas paints, not a themed choice of
    // its own. The rest of the dashboard (topbar, sidebar, other routes)
    // keeps its existing dark theme unchanged.
    <div className="h-full w-full bg-[#f3efe4]">
      <MujocoViewer interactive className="h-full w-full" />
    </div>
  );
}
