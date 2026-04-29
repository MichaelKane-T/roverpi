import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

const backend = 'http://roverpi.local:5000'

export default defineConfig({
  plugins: [react(), tailwindcss()],
  server: {
    proxy: {
      '/video': backend,
      '/map': backend,
      '/cmd': backend,
      '/cam': backend,
      '/status': backend,
      '/esp32_status': backend,
      '/auto': backend,
      '/health': backend,
      '/battery': backend,
      '/gyro': backend,
    },
  },
})