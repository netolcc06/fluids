import { defineConfig } from 'vite';

export default defineConfig({
  root: '.',
  // Relative asset URLs so the build works under a GitHub Pages subpath
  // (https://<user>.github.io/<repo>/) as well as at a domain root.
  base: './',
  publicDir: 'public',
  server: {
    port: 5173,
  },
  build: {
    outDir: 'dist',
    target: 'es2020',
  },
  // The Emscripten glue module is a plain ES module emitted at build time by
  // scripts/build_wasm.ps1 into src/wasm-gen/ (fluids.js + fluids.wasm) —
  // deliberately NOT under public/, since Vite's dev server refuses to serve
  // public/ files through its ES module import() pipeline. Living under
  // src/ means it's a normal module in both dev and build, and Vite's
  // built-in `new URL('fluids.wasm', import.meta.url)` asset handling
  // (which the Emscripten glue uses internally) takes care of the .wasm
  // binary automatically.
  optimizeDeps: {
    exclude: ['/src/wasm-gen/fluids.js'],
  },
});
