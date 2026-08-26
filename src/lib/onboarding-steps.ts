export type OnboardingStep = {
  id: string;
  title: string;
  body: string;
  /** data-tour value of the element to spotlight; omit for a centered step. */
  target?: string;
};

export const onboardingSteps: OnboardingStep[] = [
  {
    id: "welcome",
    title: "Welcome to Starforge",
    body: "Quick tour of the developer portal — test Buildo in simulation, train your robot, rent GPU compute, and pick up new skills. Takes about a minute.",
  },
  {
    id: "simulation",
    title: "Test out Buildo on MuJoCo",
    body: "Run Buildo in a physics simulation environment before deploying to hardware. Your 20$ credit is ready to use.",
  },
  {
    id: "check-email",
    title: "Check your inbox",
    body: "We've sent a confirmation email to the address you signed up with. Keep it — it's where activity notices land.",
  },
  {
    id: "nav-overview",
    title: "Overview",
    body: "Your credit balance, GPU sessions, skills owned, and recent activity, all in one place.",
    target: "nav-overview",
  },
  {
    id: "nav-robots",
    title: "Robots",
    body: "Pair a Buildo unit to your account to monitor telemetry and manage installed skills remotely.",
    target: "nav-robots",
  },
  {
    id: "nav-train",
    title: "Train",
    body: "Train your robot starting from a pre-trained Buildo model, or bring your own dataset.",
    target: "nav-train",
  },
  {
    id: "nav-gpu",
    title: "GPU compute",
    body: "Rent on-demand GPU time, paid with your credit balance, to train and evaluate your models.",
    target: "nav-gpu",
  },
  {
    id: "nav-skills",
    title: "Skills",
    body: "Buy pre-built skill packages with your credit balance and load them onto your Buildo robot.",
    target: "nav-skills",
  },
  {
    id: "nav-tutorial",
    title: "Come back anytime",
    body: "You're all set. Replay this tour whenever you like from Tutorial at the bottom of this menu.",
    target: "nav-tutorial",
  },
];
