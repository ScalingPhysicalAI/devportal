export const TOKEN_SYMBOL = "$";
export const EARLY_WALLET_REWARD = 20;

export const SKILLS_CATALOG = [
  { id: "pick-place", name: "Pick & Place", price: 15, category: "Manipulation", desc: "Grasp and place arbitrary small objects with visual servoing." },
  { id: "nav-indoor", name: "Indoor Navigation", price: 20, category: "Mobility", desc: "SLAM-based navigation and obstacle avoidance for indoor spaces." },
  { id: "cooking-assist", name: "Cooking Assistant", price: 35, category: "Household", desc: "Utensil handling, ingredient sorting, and stovetop-safe motion limits." },
  { id: "warehouse-sort", name: "Warehouse Sorting", price: 30, category: "Industrial", desc: "Barcode-guided sorting and palletizing routines." },
  { id: "door-handle", name: "Door & Handle Ops", price: 12, category: "Mobility", desc: "Detect and operate levers, knobs, and push/pull doors." },
  { id: "human-follow", name: "Human Following", price: 18, category: "Household", desc: "Safe-distance visual tracking and following of a person." },
] as const;

export const PRETRAINED_MODELS = [
  { id: "buildo-base-v1", name: "Buildo Base v1", desc: "General-purpose manipulation and locomotion baseline.", params: "1.2B" },
  { id: "buildo-house-v1", name: "Buildo Household v1", desc: "Fine-tuned on kitchen and household manipulation tasks.", params: "1.2B" },
  { id: "buildo-warehouse-v1", name: "Buildo Warehouse v1", desc: "Fine-tuned on sorting, palletizing, and navigation tasks.", params: "1.2B" },
] as const;
