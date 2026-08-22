import "server-only";

import nodemailer from "nodemailer";
import type { Transporter } from "nodemailer";

const globalForMail = globalThis as unknown as {
  transporter: Transporter | undefined;
  transporterFrom: string | undefined;
};

/**
 * Returns a cached nodemailer transporter. In production, point SMTP_HOST at
 * a real provider (AWS SES, Postmark, etc). In local dev with no SMTP_HOST
 * set, a free Ethereal test inbox is created on first use so signup emails
 * are genuinely sent and previewable via a logged URL, with no setup needed.
 */
async function getTransporter(): Promise<{ transporter: Transporter; from: string }> {
  if (globalForMail.transporter && globalForMail.transporterFrom) {
    return { transporter: globalForMail.transporter, from: globalForMail.transporterFrom };
  }

  if (process.env.SMTP_HOST) {
    const transporter = nodemailer.createTransport({
      host: process.env.SMTP_HOST,
      port: Number(process.env.SMTP_PORT ?? 587),
      secure: Number(process.env.SMTP_PORT) === 465,
      auth: process.env.SMTP_USER
        ? { user: process.env.SMTP_USER, pass: process.env.SMTP_PASS }
        : undefined,
    });
    const from = process.env.SMTP_FROM ?? "Starforge Robotics <no-reply@starforgerobotics.com>";
    globalForMail.transporter = transporter;
    globalForMail.transporterFrom = from;
    return { transporter, from };
  }

  if (process.env.NODE_ENV === "production") {
    console.error(
      "[mailer] SMTP_HOST is not set in production — welcome emails will be sent to a throwaway Ethereal test inbox instead of real users. Set SMTP_HOST/SMTP_USER/SMTP_PASS."
    );
  }

  const testAccount = await nodemailer.createTestAccount();
  const transporter = nodemailer.createTransport({
    host: "smtp.ethereal.email",
    port: 587,
    secure: false,
    auth: { user: testAccount.user, pass: testAccount.pass },
  });
  const from = "Starforge Robotics <no-reply@starforgerobotics.com>";
  globalForMail.transporter = transporter;
  globalForMail.transporterFrom = from;

  console.log(
    `[mailer] No SMTP_HOST configured — using a temporary Ethereal test inbox (${testAccount.user}). Set SMTP_HOST/SMTP_USER/SMTP_PASS in .env for real delivery.`
  );

  return { transporter, from };
}

function welcomeEmailHtml(name: string) {
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
            <tr>
              <td style="padding:36px 32px 8px 32px;">
                <h1 style="margin:0 0 16px 0;font-size:22px;line-height:1.3;color:#111111;">You're in, ${name}.</h1>
                <p style="margin:0 0 16px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
                  Your Starforge Developer Portal account is registered. From here you can connect a wallet, train your Buildo robot, rent GPU compute, and pick up new skills.
                </p>
                <p style="margin:0 0 28px 0;font-size:15px;line-height:1.6;color:#3f3a33;">
                  Connect your wallet from the dashboard to claim your early-signup STARFORGE token reward.
                </p>
                <a href="${process.env.NEXT_PUBLIC_APP_URL ?? "http://localhost:3000"}/dashboard"
                   style="display:inline-block;background-color:#b89c72;color:#050505;text-decoration:none;font-size:14px;font-weight:600;padding:12px 22px;border-radius:4px;">
                  Open dashboard
                </a>
              </td>
            </tr>
            <tr>
              <td style="padding:28px 32px 32px 32px;border-top:1px solid #ececec;margin-top:24px;">
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

export async function sendWelcomeEmail(to: string, name: string) {
  const { transporter, from } = await getTransporter();

  const info = await transporter.sendMail({
    from,
    to,
    subject: "Welcome to the Starforge Developer Portal",
    text: `You're in, ${name}.\n\nYour Starforge Developer Portal account is registered. Connect your wallet from the dashboard to claim your early-signup STARFORGE token reward.\n\n${process.env.NEXT_PUBLIC_APP_URL ?? "http://localhost:3000"}/dashboard`,
    html: welcomeEmailHtml(name),
  });

  const previewUrl = nodemailer.getTestMessageUrl(info);
  if (previewUrl) {
    console.log(`[mailer] Welcome email preview: ${previewUrl}`);
  }

  return info;
}
