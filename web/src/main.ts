// Phase 1-4c (complete): SPH/FLIP/MPM water plus Smoke for all three solver
// families — FLIP Smoke (grid), MPM Smoke (particle/APIC), SPH Smoke
// (particle/kernel) — switchable live from the UI. Loads the compiled WASM
// module (exposing `Simulation` (SPH), `FlipSimulation`/
// `FlipSmokeSimulation` (FLIP water/smoke), `MpmSimulation`/
// `MpmSmokeSimulation` (MPM water/smoke), and `SphSmokeSimulation` (SPH
// smoke), per docs/sim-api.md, docs/flip-sim-api.md, docs/mpm-sim-api.md,
// docs/flip-smoke-sim-api.md, docs/mpm-smoke-sim-api.md,
// docs/sph-smoke-sim-api.md), drives whichever is active via a
// fixed-timestep substep loop independent of display refresh rate, and
// renders it via WebGL2 — particles colored by velocity magnitude for
// water, a density/temperature field texture for FLIP Smoke, or
// density/temperature-colored particles for MPM/SPH Smoke (see
// SolverManager.renderMode / .particleSmoke). Exposes each solver/
// material's own param sliders plus scenario switching through a Tweakpane
// panel. Built strictly against the JS contract docs (not the C++ source).

import { loadFluidsModule } from './sim/loadFluidsModule';
import { SolverManager } from './sim/SolverManager';
import { FixedTimestepLoop } from './sim/FixedTimestepLoop';
import { ParticleRenderer, computeCameraTransform } from './renderer/ParticleRenderer';
import { FieldRenderer } from './renderer/FieldRenderer';
import { createPanel, Dimension } from './ui/Panel';
import { Sph3DSimulationController } from './sim/Sph3DSimulationController';
import { OrbitCamera } from './renderer/OrbitCamera';
import { ParticleRenderer3D } from './renderer/ParticleRenderer3D';

function createGL(canvas: HTMLCanvasElement): WebGL2RenderingContext {
  const gl = canvas.getContext('webgl2');
  if (!gl) {
    throw new Error('WebGL2 is not available in this browser.');
  }
  return gl;
}

// Point radius in pixels is derived from particle spacing and the current
// pixels-per-meter camera scale, so points look reasonably "packed"
// regardless of domain size or spacing slider value, rather than a fixed
// pixel constant that would look wrong across presets/spacing changes.
function computePointSizePx(spacingMeters: number, pixelsPerMeter: number): number {
  const raw = spacingMeters * pixelsPerMeter * 1.4;
  return Math.min(30, Math.max(2, raw));
}

async function main() {
  const canvas = document.getElementById('gl-canvas') as HTMLCanvasElement;

  // Size the canvas immediately (independent of the WASM module load) so
  // it isn't stuck at the default 300x150 intrinsic canvas size while the
  // module fetches/instantiates.
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;

  const gl = createGL(canvas);
  const particleRenderer = new ParticleRenderer(gl);
  const fieldRenderer = new FieldRenderer(gl);
  const particleRenderer3D = new ParticleRenderer3D(gl);
  gl.viewport(0, 0, canvas.width, canvas.height);

  const Module = await loadFluidsModule();
  const manager = new SolverManager(Module);
  // A wholly separate 3D vertical slice (see docs/sph3d-sim-api.md and
  // Sph3DSimulationController's doc comment) — constructed eagerly here,
  // alongside `manager`, so the Panel always has a stable instance to bind
  // its "3D SPH Params"/"3D Scenario" controls to regardless of which
  // Dimension is currently active, exactly mirroring how `manager` itself
  // is constructed eagerly with a default SPH Water controller. It only
  // actually steps while `dimension === 'threeD'` (see the loop below), so
  // sitting idle in 2D mode costs nothing per frame.
  const sph3d = new Sph3DSimulationController(Module);
  const orbitCamera = new OrbitCamera(canvas);

  // Which top-level pipeline is currently driving the step loop and render
  // call. Defaults to '2D' so the app boots into exactly the same behavior
  // it always has; switching to '3D' (via the Panel's Dimension toggle)
  // does not reset or otherwise touch `manager`'s state, so switching back
  // to '2D' resumes it exactly where it was left.
  let dimension: Dimension = 'twoD';

  // Both 2D renderers share the same domain-to-clip-space camera transform
  // (computeCameraTransform) so particles and the smoke field respect the
  // identical letterbox/pillarbox fit — only the point-size computation is
  // particle-mode-specific.
  function updateCameraAndPointSize() {
    const camera = computeCameraTransform(
      canvas.width,
      canvas.height,
      manager.domainWidth,
      manager.domainHeight
    );
    particleRenderer.setCamera(camera);
    fieldRenderer.setCamera(camera);
    if (manager.renderMode !== 'field') {
      particleRenderer.setPointSizePx(computePointSizePx(manager.spacing, camera.pixelsPerMeter));
    }
  }

  // The 3D orbit camera's projection aspect is recomputed fresh every frame
  // from the current canvas size (see the render loop below), so resize
  // needs no dedicated 3D-specific handling beyond the existing
  // gl.viewport() call — the orbit camera's *target* (domain center) only
  // needs recomputing on a 3D reset/dimension-switch, not on resize.
  function updateOrbitCameraTarget() {
    orbitCamera.setTarget(sph3d.domainWidth / 2, sph3d.domainHeight / 2, sph3d.domainDepth / 2);
  }
  updateOrbitCameraTarget();
  // Initial distance framed to comfortably fit the default Dam Break 3D
  // domain's bounding box; left as the user's own zoom level afterward
  // (dimension switches / 3D resets only recompute the target, not the
  // distance, the same "recenter but don't undo the user's zoom" behavior
  // a 3D content viewer would have).
  orbitCamera.distance = Math.max(sph3d.domainWidth, sph3d.domainHeight, sph3d.domainDepth) * 2.4;

  function resize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    gl.viewport(0, 0, canvas.width, canvas.height);
    updateCameraAndPointSize();
  }
  updateCameraAndPointSize();
  window.addEventListener('resize', resize);

  // Substep is solver-aware (see SolverManager.recommendedSubstep): SPH/FLIP
  // are stable at 1/120, but MPM's stiffer explicit EOS integration needs
  // 1/480 (docs/mpm-sim-api.md's empirical finding — 1/240 diverges). The
  // loop re-reads this every tick, so switching solvers live also switches
  // the substep the fixed-timestep accumulator steps by. The 3D path reuses
  // this exact same loop (its stepFn/getFixedDt callbacks are dimension-
  // agnostic) rather than needing a second loop instance — it just branches
  // on `dimension` inside the callback, same as the render loop below.
  // dt = 1/120 matches docs/sph3d-sim-api.md's documented stable substep.
  const loop = new FixedTimestepLoop(
    (fixedDt) => {
      if (dimension === 'threeD') {
        sph3d.step(fixedDt);
      } else {
        manager.step(fixedDt);
      }
    },
    () => (dimension === 'threeD' ? 1 / 120 : manager.recommendedSubstep)
  );

  createPanel(
    manager,
    loop,
    {
      onAfterReset: () => {
        // reset()/solver switch can change particle count/domain size; camera
        // + point size must be recomputed and the renderer will pick up the
        // freshly rebuilt manager.positions/velocities views on the next
        // frame.
        updateCameraAndPointSize();
      },
      onAfter3DReset: () => {
        // A 3D reset can change the domain size (different presets have
        // different domainWidth/Height/Depth) — recompute the orbit
        // camera's target (new domain center) so it stays framed correctly.
        updateOrbitCameraTarget();
      },
      onDimensionChange: (dim) => {
        dimension = dim;
        if (dim === 'twoD') {
          // Recompute the 2D camera/point-size in case the canvas was
          // resized while 3D was active (resize() only recomputes the
          // *active* dimension's camera state today would be an
          // unnecessary special case — simplest is to just recompute both
          // on every dimension switch, which is cheap and always correct).
          updateCameraAndPointSize();
        } else {
          updateOrbitCameraTarget();
        }
      },
    },
    sph3d
  );

  function frame(now: number) {
    loop.tick(now);
    if (dimension === 'threeD') {
      const aspect = canvas.width / canvas.height;
      particleRenderer3D.render(
        sph3d.positions,
        sph3d.velocities,
        sph3d.particleCount,
        orbitCamera.getViewMatrix(),
        orbitCamera.getProjectionMatrix(aspect),
        canvas.height
      );
    } else {
      // Guard against the 3D renderer having left depth testing enabled —
      // it's the only place in this project that ever turns it on, and the
      // 2D renderers were never built to coexist with it (no depth writes
      // of their own, so a stale depth buffer from a previous 3D frame
      // could otherwise incorrectly cull 2D points against it).
      gl.disable(gl.DEPTH_TEST);
      // Branch purely on the manager's render-mode discriminant — main.ts
      // doesn't need to know which solver/material combination produced it,
      // just whether this frame's state is particle-shaped or field-shaped
      // (see SolverManager.renderMode's doc comment).
      if (manager.renderMode === 'field') {
        const smoke = manager.smoke!;
        const p = smoke.getParams();
        fieldRenderer.render(
          smoke.density,
          smoke.temperature,
          smoke.gridWidth,
          smoke.gridHeight,
          smoke.domainWidth,
          smoke.domainHeight,
          p.ambientTemperature,
          p.sourceTemperature
        );
      } else if (manager.renderMode === 'particlesDensity') {
        // MPM Smoke or SPH Smoke: both particle-shaped (unlike FLIP Smoke's
        // field), but colored by density/temperature rather than velocity —
        // see docs/mpm-smoke-sim-api.md / docs/sph-smoke-sim-api.md and
        // ParticleRenderer's colorMode option. `manager.particleSmoke`
        // collapses both controllers into one accessor (they share the exact
        // same positions/velocities/particleCount/densities/temperatures/
        // getParams() shape) so this branch doesn't need to know or care
        // which specific solver is active. Its positions/.velocities/
        // .particleCount already reflect this frame's re-sliced active range
        // (both controllers re-read getParticleCount() and re-slice every
        // step(), without re-fetching the underlying pointers) — no special
        // per-frame handling needed here beyond reading them fresh, which
        // this loop already does every frame regardless of solver.
        const particleSmoke = manager.particleSmoke!;
        const p = particleSmoke.getParams();
        particleRenderer.render(particleSmoke.positions, particleSmoke.velocities, particleSmoke.particleCount, {
          colorMode: 'density',
          densities: particleSmoke.densities,
          temperatures: particleSmoke.temperatures,
          ambientTemperature: p.ambientTemperature,
          sourceTemperature: p.sourceTemperature,
        });
      } else {
        particleRenderer.render(manager.positions, manager.velocities, manager.particleCount);
      }
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}

main().catch((err) => {
  // eslint-disable-next-line no-console
  console.error('Fluids failed to start:', err);
});
