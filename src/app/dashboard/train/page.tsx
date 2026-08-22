import { TrainTabs } from "@/components/dashboard/TrainTabs";

export default function TrainPage() {
  return (
    <div className="max-w-5xl">
      <p className="text-technical text-xs text-sand mb-2">TRAINING</p>
      <h1 className="text-display text-4xl text-off-white">Train your robot</h1>
      <p className="mt-3 max-w-lg text-sm text-off-white/70">
        Start from a pre-trained Buildo model, or bring your own dataset
        collected with the data collection SDK.
      </p>

      <div className="mt-10">
        <TrainTabs />
      </div>
    </div>
  );
}
