import { chromium } from "playwright";

const BASE_URL = process.env.BASE_URL ?? "http://localhost:3000";
const EMAIL = process.env.TEST_USER_EMAIL;
const PASSWORD = process.env.TEST_USER_PASSWORD;
const OUT = process.argv[2] ?? "/tmp/simulate.png";
const PATH = process.argv[3] ?? "/dashboard/simulate";
const SETTLE_MS = Number(process.argv[4] ?? 4000);

if (!EMAIL || !PASSWORD) {
  throw new Error("Missing TEST_USER_EMAIL / TEST_USER_PASSWORD in env");
}

const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
page.on("console", (msg) => console.log(`[browser:${msg.type()}]`, msg.text()));
page.on("pageerror", (err) => console.log("[pageerror]", err.message));

await page.goto(`${BASE_URL}/login`);
await page.fill("#email", EMAIL);
await page.fill("#password", PASSWORD);
await page.click('button[type="submit"]');
try {
  await page.waitForURL("**/dashboard**", { timeout: 15000 });
} catch (e) {
  await page.screenshot({ path: OUT.replace(".png", "-debug.png") });
  console.log("URL after failed login attempt:", page.url());
  console.log("Body text:", (await page.textContent("body"))?.slice(0, 500));
  throw e;
}

await page.goto(`${BASE_URL}${PATH}`);
const skipButton = page.getByRole("button", { name: "Skip all" });
try {
  await skipButton.click({ timeout: 5000 });
  await skipButton.waitFor({ state: "hidden", timeout: 5000 });
  console.log("dismissed onboarding tour");
} catch (e) {
  console.log("no onboarding tour to dismiss (or dismiss failed):", e.message.split("\n")[0]);
}
try {
  await page.waitForSelector("canvas", { timeout: 30000 });
  // Let the physics engine finish loading + a few frames settle before capturing.
  await page.waitForTimeout(SETTLE_MS);
} catch (e) {
  console.log("canvas never appeared, screenshotting current state anyway");
}

if (process.env.ORBIT_TOP_DOWN === "1") {
  const canvas = page.locator("canvas");
  const box = await canvas.boundingBox();
  const cx = box.x + box.width / 2;
  const cy = box.y + box.height / 2;
  await page.mouse.move(cx, cy + 150);
  await page.mouse.down();
  await page.mouse.move(cx, cy - 400, { steps: 20 });
  await page.mouse.up();
  await page.waitForTimeout(500);
}

await page.screenshot({ path: OUT });
console.log(`saved -> ${OUT}`);

await browser.close();
