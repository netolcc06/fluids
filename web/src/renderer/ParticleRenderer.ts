// WebGL2 particle renderer: gl.POINTS with a soft-edge circular fragment
// shader (discard + alpha fade via gl_PointCoord), colored by velocity
// magnitude through a 3-stop cool -> warm -> hot gradient computed in the
// vertex shader and interpolated per-fragment.

function compileShader(gl: WebGL2RenderingContext, type: number, source: string): WebGLShader {
  const shader = gl.createShader(type);
  if (!shader) throw new Error('Failed to create shader.');
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    const info = gl.getShaderInfoLog(shader);
    gl.deleteShader(shader);
    throw new Error(`Shader compile error: ${info}`);
  }
  return shader;
}

function createProgram(gl: WebGL2RenderingContext, vsSource: string, fsSource: string): WebGLProgram {
  const vs = compileShader(gl, gl.VERTEX_SHADER, vsSource);
  const fs = compileShader(gl, gl.FRAGMENT_SHADER, fsSource);
  const program = gl.createProgram();
  if (!program) throw new Error('Failed to create program.');
  gl.attachShader(program, vs);
  gl.attachShader(program, fs);
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    const info = gl.getProgramInfoLog(program);
    gl.deleteProgram(program);
    throw new Error(`Program link error: ${info}`);
  }
  gl.deleteShader(vs);
  gl.deleteShader(fs);
  return program;
}

const VERTEX_SHADER_SRC = `#version 300 es
layout(location = 0) in vec2 a_position;
layout(location = 1) in vec2 a_velocity;
layout(location = 2) in float a_density;
layout(location = 3) in float a_temperature;

uniform vec2 uScale;
uniform vec2 uOffset;
uniform float uPointSize;
uniform float uMaxSpeed;
// 0.0 = water's velocity-magnitude rainbow gradient (default, unchanged
// behavior). 1.0 = smoke's density-as-opacity/temperature-as-tint mode (see
// FieldRenderer's fragment shader for the grid-smoke equivalent of this
// same visual language, kept consistent between the two smoke variants).
uniform float uColorMode;
uniform float uAmbientTemperature;
uniform float uSourceTemperature;

out vec3 v_color;
out float v_alpha;

vec3 speedColor(float t) {
  // cool (calm) -> teal (mid) -> hot (energetic), 3-stop gradient.
  vec3 cold = vec3(0.13, 0.35, 0.90);
  vec3 mid  = vec3(0.20, 0.85, 0.55);
  vec3 hot  = vec3(0.98, 0.25, 0.10);
  float lo = clamp(t * 2.0, 0.0, 1.0);
  float hi = clamp(t * 2.0 - 1.0, 0.0, 1.0);
  vec3 loHalf = mix(cold, mid, lo);
  vec3 hiHalf = mix(mid, hot, hi);
  return mix(loHalf, hiHalf, step(0.5, t));
}

void main() {
  // Maps sim-space [0,domainWidth] x [0,domainHeight] (+y up) to clip
  // space; uScale/uOffset are recomputed by the app whenever the domain
  // size or canvas size changes (see computeCameraTransform below).
  gl_Position = vec4(a_position * uScale + uOffset, 0.0, 1.0);
  gl_PointSize = uPointSize;

  if (uColorMode > 0.5) {
    // Smoke (MPM, particle-based): light/white base tinted warm near the
    // source temperature, density used directly as alpha (per
    // docs/mpm-smoke-sim-api.md: density is already ~[0,1] and safe to use
    // directly as opacity) — same palette/approach as FieldRenderer's grid
    // density-as-opacity rendering, for visual consistency between the two
    // smoke variants.
    vec3 coolColor = vec3(0.86, 0.87, 0.90);
    vec3 hotColor = vec3(1.0, 0.55, 0.18);
    float tRange = max(uSourceTemperature - uAmbientTemperature, 1e-4);
    float hotT = clamp((a_temperature - uAmbientTemperature) / tRange, 0.0, 1.0);
    v_color = mix(coolColor, hotColor, hotT * 0.55);
    v_alpha = clamp(a_density, 0.0, 1.0);
  } else {
    // Water: unchanged velocity-magnitude rainbow gradient, full opacity
    // (fragment shader's circular edge fade is the only alpha falloff).
    float speed = length(a_velocity);
    float t = clamp(speed / max(uMaxSpeed, 1e-4), 0.0, 1.0);
    v_color = speedColor(t);
    v_alpha = 1.0;
  }
}
`;

const FRAGMENT_SHADER_SRC = `#version 300 es
precision mediump float;
in vec3 v_color;
in float v_alpha;
out vec4 outColor;

void main() {
  vec2 c = gl_PointCoord * 2.0 - 1.0;
  float r = length(c);
  if (r > 1.0) discard;
  float edgeAlpha = smoothstep(1.0, 0.65, r);
  outColor = vec4(v_color, edgeAlpha * v_alpha);
}
`;

export interface CameraTransform {
  scaleX: number;
  scaleY: number;
  offsetX: number;
  offsetY: number;
  /** Pixels per simulation meter at the current fit — useful for sizing
   * points proportionally to particle spacing. */
  pixelsPerMeter: number;
}

/**
 * Maps sim domain [0,domainWidth] x [0,domainHeight] (+y up) into WebGL
 * clip space, preserving aspect ratio via letterbox/pillarbox (a
 * "contain" fit: the domain is centered and scaled uniformly so it never
 * gets stretched).
 */
export function computeCameraTransform(
  canvasWidthPx: number,
  canvasHeightPx: number,
  domainWidth: number,
  domainHeight: number
): CameraTransform {
  const pixelsPerMeter = Math.min(canvasWidthPx / domainWidth, canvasHeightPx / domainHeight);
  const scaleX = (2 * pixelsPerMeter) / canvasWidthPx;
  const scaleY = (2 * pixelsPerMeter) / canvasHeightPx;
  const offsetX = -(domainWidth / 2) * scaleX;
  const offsetY = -(domainHeight / 2) * scaleY;
  return { scaleX, scaleY, offsetX, offsetY, pixelsPerMeter };
}

/** Options for coloring smoke particles by density/temperature instead of
 * water's velocity-magnitude rainbow. Omit entirely (or pass no options
 * argument at all) to get the original water rendering path unchanged. */
export interface ParticleColorOptions {
  colorMode: 'velocity' | 'density';
  /** Required when colorMode === 'density'. Length must equal particleCount. */
  densities?: Float32Array;
  /** Required when colorMode === 'density'. Length must equal particleCount. */
  temperatures?: Float32Array;
  ambientTemperature?: number;
  sourceTemperature?: number;
}

export class ParticleRenderer {
  private readonly gl: WebGL2RenderingContext;
  private readonly program: WebGLProgram;
  private readonly vao: WebGLVertexArrayObject;
  private readonly positionBuffer: WebGLBuffer;
  private readonly velocityBuffer: WebGLBuffer;
  private readonly densityBuffer: WebGLBuffer;
  private readonly temperatureBuffer: WebGLBuffer;

  private readonly uScale: WebGLUniformLocation;
  private readonly uOffset: WebGLUniformLocation;
  private readonly uPointSize: WebGLUniformLocation;
  private readonly uMaxSpeed: WebGLUniformLocation;
  private readonly uColorMode: WebGLUniformLocation;
  private readonly uAmbientTemperature: WebGLUniformLocation;
  private readonly uSourceTemperature: WebGLUniformLocation;

  // Fixed reference "hot" speed (m/s) used to normalize the color
  // gradient. Chosen empirically against Dam Break / Block Drop peak
  // splash speeds rather than computed as a dynamic per-frame max, so the
  // color scale doesn't flicker/rescale as the fastest particle changes
  // frame to frame.
  private maxSpeedReference = 3.5;

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl;
    this.program = createProgram(gl, VERTEX_SHADER_SRC, FRAGMENT_SHADER_SRC);

    const vao = gl.createVertexArray();
    if (!vao) throw new Error('Failed to create VAO.');
    this.vao = vao;
    gl.bindVertexArray(vao);

    const positionBuffer = gl.createBuffer();
    const velocityBuffer = gl.createBuffer();
    if (!positionBuffer || !velocityBuffer) throw new Error('Failed to create buffers.');
    this.positionBuffer = positionBuffer;
    this.velocityBuffer = velocityBuffer;

    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, velocityBuffer);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 2, gl.FLOAT, false, 0, 0);

    const densityBuffer = gl.createBuffer();
    const temperatureBuffer = gl.createBuffer();
    if (!densityBuffer || !temperatureBuffer) throw new Error('Failed to create buffers.');
    this.densityBuffer = densityBuffer;
    this.temperatureBuffer = temperatureBuffer;

    // Locations 2/3 (density/temperature) are only meaningful in
    // colorMode === 'density' (smoke); left enabled permanently (upload is
    // skipped/zero-filled for water draws in render() below) rather than
    // toggled per-draw, keeping the VAO's enabled-attribute set static.
    gl.bindBuffer(gl.ARRAY_BUFFER, densityBuffer);
    gl.enableVertexAttribArray(2);
    gl.vertexAttribPointer(2, 1, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, temperatureBuffer);
    gl.enableVertexAttribArray(3);
    gl.vertexAttribPointer(3, 1, gl.FLOAT, false, 0, 0);

    gl.bindVertexArray(null);

    const uScale = gl.getUniformLocation(this.program, 'uScale');
    const uOffset = gl.getUniformLocation(this.program, 'uOffset');
    const uPointSize = gl.getUniformLocation(this.program, 'uPointSize');
    const uMaxSpeed = gl.getUniformLocation(this.program, 'uMaxSpeed');
    const uColorMode = gl.getUniformLocation(this.program, 'uColorMode');
    const uAmbientTemperature = gl.getUniformLocation(this.program, 'uAmbientTemperature');
    const uSourceTemperature = gl.getUniformLocation(this.program, 'uSourceTemperature');
    if (
      !uScale ||
      !uOffset ||
      !uPointSize ||
      !uMaxSpeed ||
      !uColorMode ||
      !uAmbientTemperature ||
      !uSourceTemperature
    ) {
      throw new Error('Failed to locate shader uniforms.');
    }
    this.uScale = uScale;
    this.uOffset = uOffset;
    this.uPointSize = uPointSize;
    this.uMaxSpeed = uMaxSpeed;
    this.uColorMode = uColorMode;
    this.uAmbientTemperature = uAmbientTemperature;
    this.uSourceTemperature = uSourceTemperature;

    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.clearColor(0.06, 0.07, 0.09, 1.0);

    gl.useProgram(this.program);
    gl.uniform1f(this.uMaxSpeed, this.maxSpeedReference);
    gl.uniform1f(this.uColorMode, 0.0);
  }

  setCamera(camera: CameraTransform): void {
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.uniform2f(this.uScale, camera.scaleX, camera.scaleY);
    gl.uniform2f(this.uOffset, camera.offsetX, camera.offsetY);
  }

  setPointSizePx(px: number): void {
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.uniform1f(this.uPointSize, px);
  }

  setMaxSpeedReference(maxSpeed: number): void {
    this.maxSpeedReference = maxSpeed;
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.uniform1f(this.uMaxSpeed, maxSpeed);
  }

  /** Scratch zero-filled buffer reused to feed the density/temperature
   * attributes on water draws (colorMode 'velocity'), where the vertex
   * shader's branch never reads them but WebGL still requires the enabled
   * attribute arrays to have a buffer of at least `particleCount` elements
   * bound. Grown (never shrunk) lazily rather than reallocated every frame. */
  private zeroScratch = new Float32Array(0);
  private zeroScratchOfLength(n: number): Float32Array {
    if (this.zeroScratch.length < n) {
      this.zeroScratch = new Float32Array(n);
    }
    return this.zeroScratch.subarray(0, n);
  }

  /** Upload the current frame's positions/velocities (plus, for smoke,
   * densities/temperatures) and draw.
   *
   * Uses `bufferData` (full re-upload) rather than `bufferSubData` every
   * frame: particle count can change on every scenario reset (the
   * buffers are reallocated per docs/sim-api.md) AND, for MPM smoke,
   * legitimately every single step() (see docs/mpm-smoke-sim-api.md), so the
   * upload size is not fixed across frames. `bufferData` handles a
   * shrinking/growing count uniformly without extra bookkeeping to detect
   * the size change, and at this project's particle counts (thousands)
   * copy-per-frame cost is negligible next to the physics step; the
   * zero-copy win described in the API docs is CPU-side (no per-particle
   * marshalling out of WASM), not about avoiding this GPU upload.
   *
   * `options` omitted (or `colorMode: 'velocity'`) reproduces the original
   * water rendering path exactly unchanged — every existing call site
   * (SPH/FLIP/MPM water) keeps working with no changes. Pass
   * `colorMode: 'density'` with `densities`/`temperatures` (both required,
   * length === particleCount) for MPM smoke's density-as-opacity/
   * temperature-as-tint look. */
  render(
    positions: Float32Array,
    velocities: Float32Array,
    particleCount: number,
    options?: ParticleColorOptions
  ): void {
    const gl = this.gl;
    const isDensityMode = options?.colorMode === 'density';

    gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.DYNAMIC_DRAW);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.velocityBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, velocities, gl.DYNAMIC_DRAW);

    const densities = isDensityMode ? options!.densities! : this.zeroScratchOfLength(particleCount);
    const temperatures = isDensityMode ? options!.temperatures! : this.zeroScratchOfLength(particleCount);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.densityBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, densities, gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.temperatureBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, temperatures, gl.DYNAMIC_DRAW);

    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.useProgram(this.program);
    gl.uniform1f(this.uColorMode, isDensityMode ? 1.0 : 0.0);
    if (isDensityMode) {
      gl.uniform1f(this.uAmbientTemperature, options?.ambientTemperature ?? 0);
      gl.uniform1f(this.uSourceTemperature, options?.sourceTemperature ?? 1);
    }
    gl.bindVertexArray(this.vao);
    gl.drawArrays(gl.POINTS, 0, particleCount);
    gl.bindVertexArray(null);
  }
}
