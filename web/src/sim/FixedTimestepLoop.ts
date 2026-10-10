/**
 * Fixed-timestep substep accumulator driven by requestAnimationFrame.
 * Accumulates real elapsed time (scaled by `timeScale` and gated by
 * `playing`) and calls `stepFn(fixedDt)` in a loop, independent of display
 * refresh rate, per the plan's Phase 1 sim-loop spec.
 */
export class FixedTimestepLoop {
  playing = true;
  timeScale = 1;

  private accumulator = 0;
  private lastTime: number;
  /** Real time is clamped to this many seconds per rAF callback before
   * being added to the accumulator, to avoid a "spiral of death" after a
   * tab-switch stall (the browser can otherwise hand us a multi-second
   * `now - lastTime` on the first frame back). */
  private readonly maxRealDtSeconds = 0.25;
  /** Hard cap on substeps per rendered frame, as a second line of defense
   * against spiral-of-death if a stall coincides with a high timeScale;
   * excess accumulated time is dropped rather than run down over many
   * subsequent frames. */
  private readonly maxSubstepsPerFrame = 240;

  /**
   * `getFixedDt` is consulted fresh on every tick/stepOnce (rather than
   * captured once at construction) specifically so a solver-aware caller
   * (see SolverManager.recommendedSubstep) can change the effective substep
   * live when the active solver changes — e.g. MPM needs a much smaller
   * stable substep (1/480) than SPH/FLIP (1/120), per docs/mpm-sim-api.md.
   * A plain number is also accepted for callers that don't need per-solver
   * substeps.
   */
  constructor(
    private readonly stepFn: (fixedDt: number) => void,
    private readonly getFixedDt: number | (() => number) = 1 / 120
  ) {
    this.lastTime = performance.now();
  }

  private currentFixedDt(): number {
    return typeof this.getFixedDt === 'function' ? this.getFixedDt() : this.getFixedDt;
  }

  /** Call once per requestAnimationFrame callback with the rAF timestamp. */
  tick(nowMs: number): void {
    const rawDt = Math.min((nowMs - this.lastTime) / 1000, this.maxRealDtSeconds);
    this.lastTime = nowMs;
    if (!this.playing) return;

    this.accumulator += rawDt * this.timeScale;
    let substeps = 0;
    // Re-read the fixed substep each iteration: cheap, and correct even if
    // it were to change mid-flush (it won't within a single tick() call in
    // practice, but re-reading costs nothing and avoids any staleness
    // question).
    while (
      this.accumulator >= this.currentFixedDt() &&
      substeps < this.maxSubstepsPerFrame
    ) {
      const dt = this.currentFixedDt();
      this.stepFn(dt);
      this.accumulator -= dt;
      substeps++;
    }
    if (substeps >= this.maxSubstepsPerFrame) {
      // We're irrecoverably behind (e.g. huge timeScale) — drop the rest
      // rather than let the accumulator grow unbounded.
      this.accumulator = 0;
    }
  }

  /** Advance exactly one fixedDt, for the Step button while paused. */
  stepOnce(): void {
    this.stepFn(this.currentFixedDt());
  }

  /** Clear accumulated time and resync the real-time clock. Call after
   * pausing/resuming or resetting the scenario so a long gap doesn't
   * translate into a burst of substeps. */
  resync(): void {
    this.accumulator = 0;
    this.lastTime = performance.now();
  }
}
