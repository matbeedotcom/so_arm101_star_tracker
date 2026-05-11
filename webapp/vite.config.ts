import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Web Bluetooth requires a secure context. `vite dev` serves on
// http://localhost which Chrome treats as secure — fine for local
// development. For LAN access, run behind HTTPS or use `vite preview`
// with a TLS proxy (Caddy/Tailscale Funnel/etc.).
export default defineConfig({
  plugins: [react()],
  server: { port: 5173 },
});
