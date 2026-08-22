import { z } from "zod";

export const signupSchema = z.object({
  name: z.string().trim().min(2, "Name must be at least 2 characters").max(80),
  email: z.string().trim().toLowerCase().email("Enter a valid email address"),
  password: z
    .string()
    .min(8, "Password must be at least 8 characters")
    .max(200),
  role: z.enum(["DEVELOPER", "RESEARCHER"]),
});

export type SignupInput = z.infer<typeof signupSchema>;

export const loginSchema = z.object({
  email: z.string().trim().toLowerCase().email("Enter a valid email address"),
  password: z.string().min(1, "Password is required"),
});

export type LoginInput = z.infer<typeof loginSchema>;

export const walletConnectSchema = z.object({
  address: z
    .string()
    .regex(/^0x[a-fA-F0-9]{40}$/, "Not a valid Ethereum address"),
});

export const gpuRentSchema = z.object({
  gpuType: z.enum(["RTX 4090", "A100 80GB", "H100"]),
  hours: z.number().int().min(1).max(24 * 14),
});

export const skillBuySchema = z.object({
  skillId: z.string().min(1),
});
