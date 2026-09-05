import "server-only";

const APP_URL = process.env.NEXT_PUBLIC_APP_URL ?? "http://localhost:3000";
const FROM_ADDRESS = "Starforge Robotics <no-reply@starforgerobotics.com>";

// ---------------------------------------------------------------------------
// HTML helpers
// ---------------------------------------------------------------------------

function emailShell(body: string) {
  return `<!doctype html>
<html>
  <body style="margin:0;padding:0;background-color:#f4f2ee;font-family:Helvetica,Arial,sans-serif;">
    <table role="presentation" width="100%" cellpadding="0" cellspacing="0" style="background-color:#f4f2ee;padding:40px 0;">
      <tr>
        <td align="center">
          <table role="presentation" width="480" cellpadding="0" cellspacing="0" style="background-color:#ffffff;border:1px solid #e6e0d4;border-radius:8px;overflow:hidden;">
            <tr>
              <td style="background-color:#050505;padding:28px 32px;">
                <span style="font-family:'Courier New',monospace;font-size:12px;letter-spacing:3px;color:#b89c72;text-transform:uppercase;">Starforge Robotics</span>
              </td>
            </tr>
            ${body}
            <tr>
              <td style="padding:28px 32px 32px 32px;border-top:1px solid #ececec;">
                <p style="margin:0;font-size:12px;line-height:1.6;color:#9a9284;">
                  Starforge Robotics &middot; Building the autonomous factory for space exploration.
                </p>
              </td>
            </tr>
          </table>
        </td>
      </tr>
    </table>
  </body>
</html>`;
}

function verifyEmailHtml(name: string, verifyUrl: string) {
  return emailShell(`
    <tr>
      <td style="padding:36px 32px 8px 32px;">
        <h1 style="margin:0 0 16px 0;font-size:22px;line-height:1.3;color:#111111;">Verify your email, ${name}.</h1>
        <p style="margin:0 0 16px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
          Click the button below to verify your email address and receive <strong>20$ credit</strong> as a bonus.
        </p>
        <p style="margin:0 0 28px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
          This link expires in 24 hours.
        </p>
        <a href="${verifyUrl}"
           style="display:inline-block;background-color:#b89c72;color:#050505;text-decoration:none;font-size:14px;font-weight:600;padding:12px 22px;border-radius:4px;">
          Verify email &amp; claim 20$ credit
        </a>
      </td>
    </tr>`);
}

function resetPasswordEmailHtml(name: string, resetUrl: string) {
  return emailShell(`
    <tr>
      <td style="padding:36px 32px 8px 32px;">
        <h1 style="margin:0 0 16px 0;font-size:22px;line-height:1.3;color:#111111;">Reset your password, ${name}.</h1>
        <p style="margin:0 0 16px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
          Click the button below to choose a new password for your Starforge account.
        </p>
        <p style="margin:0 0 28px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
          This link expires in 1 hour. If you didn't request this, you can ignore this email.
        </p>
        <a href="${resetUrl}"
           style="display:inline-block;background-color:#b89c72;color:#050505;text-decoration:none;font-size:14px;font-weight:600;padding:12px 22px;border-radius:4px;">
          Reset password
        </a>
      </td>
    </tr>`);
}

function welcomeEmailHtml(name: string) {
  return emailShell(`
    <tr>
      <td style="padding:36px 32px 8px 32px;">
        <h1 style="margin:0 0 16px 0;font-size:22px;line-height:1.3;color:#111111;">You're in, ${name}.</h1>
        <p style="margin:0 0 16px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
          Your Starforge Developer Portal account is registered. From here you can connect a wallet, train your Buildo robot, rent GPU compute, and pick up new skills.
        </p>
        <p style="margin:0 0 28px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
          Connect your wallet from the dashboard to claim your early-signup 20$ credit reward.
        </p>
        <a href="${APP_URL}/dashboard"
           style="display:inline-block;background-color:#b89c72;color:#050505;text-decoration:none;font-size:14px;font-weight:600;padding:12px 22px;border-radius:4px;">
          Open dashboard
        </a>
      </td>
    </tr>`);
}

// ---------------------------------------------------------------------------
// Transport — Resend in production, Ethereal nodemailer in local dev
// ---------------------------------------------------------------------------

async function sendViaResend(to: string, subject: string, html: string, text: string) {
  const { Resend } = await import("resend");
  const resend = new Resend(process.env.RESEND_API_KEY);

  const { error } = await resend.emails.send({
    from: FROM_ADDRESS,
    to,
    subject,
    html,
    text,
  });

  if (error) throw new Error(`[resend] ${error.message}`);
}

async function sendViaEthereal(to: string, subject: string, html: string, text: string) {
  const nodemailer = (await import("nodemailer")).default;
  const testAccount = await nodemailer.createTestAccount();
  const transporter = nodemailer.createTransport({
    host: "smtp.ethereal.email",
    port: 587,
    secure: false,
    auth: { user: testAccount.user, pass: testAccount.pass },
  });

  const info = await transporter.sendMail({ from: FROM_ADDRESS, to, subject, html, text });
  const previewUrl = nodemailer.getTestMessageUrl(info);
  if (previewUrl) console.log(`[mailer] Preview: ${previewUrl}`);
}

async function send(to: string, subject: string, html: string, text: string) {
  if (process.env.RESEND_API_KEY) {
    return sendViaResend(to, subject, html, text);
  }
  if (process.env.NODE_ENV === "production") {
    console.error("[mailer] RESEND_API_KEY is not set — falling back to Ethereal test inbox.");
  }
  return sendViaEthereal(to, subject, html, text);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

export async function sendVerificationEmail(to: string, name: string, token: string) {
  const verifyUrl = `${APP_URL}/api/auth/verify-email?token=${token}`;
  await send(
    to,
    "Verify your Starforge email — claim 20$ credit",
    verifyEmailHtml(name, verifyUrl),
    `Verify your email and claim 20$ credit: ${verifyUrl}\n\nThis link expires in 24 hours.`,
  );
}

export async function sendPasswordResetEmail(to: string, name: string, token: string) {
  const resetUrl = `${APP_URL}/reset-password?token=${token}`;
  await send(
    to,
    "Reset your Starforge password",
    resetPasswordEmailHtml(name, resetUrl),
    `Reset your password: ${resetUrl}\n\nThis link expires in 1 hour. If you didn't request this, you can ignore this email.`,
  );
}

export async function sendWelcomeEmail(to: string, name: string) {
  await send(
    to,
    "Welcome to the Starforge Developer Portal",
    welcomeEmailHtml(name),
    `You're in, ${name}.\n\nYour Starforge Developer Portal account is registered. Open your dashboard: ${APP_URL}/dashboard`,
  );
}
