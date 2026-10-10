// WebGL2 3D particle renderer for the SPH 3D water solver — a new, separate
// renderer (not a variant of ./ParticleRenderer.ts, which is built around an
// orthographic domain-to-clip-space transform with no depth concept at all).
// Renders gl.POINTS with a real model/view/projection transform, depth
// testing enabled so nearer particles occlude farther ones, perspective-
// correct point sizing (apparent size shrinks with view-space distance, the
// way real geometry does), and a pseudo-spherical fragment shader (fake
// normal reconstructed from gl_PointCoord, one fixed directional light) so
// an orbiting camera actually reads as looking at a 3D blob of spheres
// rather than a flat 2D sprite scatter. Colored by velocity magnitude with
// the same cool-to-warm gradient concept as the 2D renderer, for visual
// consistency across the project.

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
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_velocity;

uniform mat4 uView;
uniform mat4 uProjection;
uniform float uMaxSpeed;
// World-space sphere diameter (meters) particles are drawn at; converted to
// a screen-space pixel size every vertex based on view-space depth, so
// particles shrink with distance the way real 3D geometry does.
uniform float uWorldPointSize;
uniform float uScreenHeightPx;

out vec3 v_color;

vec3 speedColor(float t) {
  // Same cool -> teal -> hot 3-stop gradient as the 2D renderer, for visual
  // consistency across the project.
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
  vec4 viewPos = uView * vec4(a_position, 1.0);
  gl_Position = uProjection * viewPos;

  // Camera looks down -z in view space (standard right-handed look-at), so
  // a particle in front of the camera has a negative view-space z — flip
  // sign to get a positive depth to divide by. Floor it so a particle
  // exactly at (or behind) the eye doesn't blow up the point size.
  float viewDepth = max(-viewPos.z, 0.02);
  float pixelsPerMeterAtDepth = uScreenHeightPx / (2.0 * viewDepth * tan(radians(27.5)));
  gl_PointSize = clamp(uWorldPointSize * pixelsPerMeterAtDepth, 2.0, 48.0);

  float speed = length(a_velocity);
  float t = clamp(speed / max(uMaxSpeed, 1e-4), 0.0, 1.0);
  v_color = speedColor(t);
}
`;

// uFovYHalfTan (tan(fovY/2)) is baked in as a uniform-free constant above
// via radians(27.5) (half of the camera's 55-degree vertical FOV) —
// intentionally simple for this first 3D slice rather than threading the
// camera's actual FOV through as a uniform; see setCamera's doc comment.

const FRAGMENT_SHADER_SRC = `#version 300 es
precision mediump float;
in vec3 v_color;
out vec4 outColor;

void main() {
  // Reconstruct a fake sphere normal from the point sprite's local
  // [-1,1]^2 coordinate (gl_PointCoord is y-down; flip to match a
  // right-handed +y-up normal). Fragments outside the unit circle are
  // discarded so the sprite reads as a circle, not a square.
  vec2 c = gl_PointCoord * 2.0 - 1.0;
  c.y = -c.y;
  float r2 = dot(c, c);
  if (r2 > 1.0) discard;
  float z = sqrt(1.0 - r2);
  vec3 normal = normalize(vec3(c, z));

  // One fixed directional light (view-space-ish direction, up and toward
  // the camera) — enough to read as a lit sphere without needing per-light
  // uniforms for this first 3D slice.
  vec3 lightDir = normalize(vec3(0.45, 0.65, 0.9));
  float diffuse = max(dot(normal, lightDir), 0.0);
  float ambient = 0.32;
  float specular = pow(diffuse, 24.0) * 0.45;

  vec3 shaded = v_color * (ambient + diffuse * 0.85) + vec3(specular);
  outColor = vec4(shaded, 1.0);
}
`;

export class ParticleRenderer3D {
  private readonly gl: WebGL2RenderingContext;
  private readonly program: WebGLProgram;
  private readonly vao: WebGLVertexArrayObject;
  private readonly positionBuffer: WebGLBuffer;
  private readonly velocityBuffer: WebGLBuffer;

  private readonly uView: WebGLUniformLocation;
  private readonly uProjection: WebGLUniformLocation;
  private readonly uMaxSpeed: WebGLUniformLocation;
  private readonly uWorldPointSize: WebGLUniformLocation;
  private readonly uScreenHeightPx: WebGLUniformLocation;

  // Reference "hot" speed (m/s) normalizing the color gradient — chosen
  // empirically against Dam Break 3D's peak collapse speeds, mirroring the
  // 2D renderer's fixed (non-rescaling) reference so the color scale
  // doesn't flicker frame to frame.
  private maxSpeedReference = 3.5;
  // World-space sphere diameter (meters). All three built-in 3D presets use
  // spacing = 0.06m (docs/sph3d-sim-api.md) — sized slightly larger than
  // spacing so particles visually overlap into a continuous-looking body,
  // same "packed" visual goal as the 2D renderer's computePointSizePx.
  private worldPointSize = 0.075;

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
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, velocityBuffer);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0);

    gl.bindVertexArray(null);

    const uView = gl.getUniformLocation(this.program, 'uView');
    const uProjection = gl.getUniformLocation(this.program, 'uProjection');
    const uMaxSpeed = gl.getUniformLocation(this.program, 'uMaxSpeed');
    const uWorldPointSize = gl.getUniformLocation(this.program, 'uWorldPointSize');
    const uScreenHeightPx = gl.getUniformLocation(this.program, 'uScreenHeightPx');
    if (!uView || !uProjection || !uMaxSpeed || !uWorldPointSize || !uScreenHeightPx) {
      throw new Error('Failed to locate shader uniforms.');
    }
    this.uView = uView;
    this.uProjection = uProjection;
    this.uMaxSpeed = uMaxSpeed;
    this.uWorldPointSize = uWorldPointSize;
    this.uScreenHeightPx = uScreenHeightPx;

    gl.useProgram(this.program);
    gl.uniform1f(this.uMaxSpeed, this.maxSpeedReference);
    gl.uniform1f(this.uWorldPointSize, this.worldPointSize);
  }

  setMaxSpeedReference(maxSpeed: number): void {
    this.maxSpeedReference = maxSpeed;
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.uniform1f(this.uMaxSpeed, maxSpeed);
  }

  setWorldPointSize(sizeMeters: number): void {
    this.worldPointSize = sizeMeters;
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.uniform1f(this.uWorldPointSize, sizeMeters);
  }

  /** Uploads the current frame's positions/velocities (3 floats/particle —
   * see Sph3DSimulationController) and draws with the given camera
   * matrices. Clears both the color and depth buffers and enables
   * gl.DEPTH_TEST itself every call (idempotent, cheap) so main.ts's 2D
   * render path doesn't need to know or care about 3D-specific GL state —
   * it just needs to gl.disable(gl.DEPTH_TEST) once before its own draws
   * (see main.ts), since this is the only place depth testing is ever
   * turned on in this project. */
  render(
    positions: Float32Array,
    velocities: Float32Array,
    particleCount: number,
    viewMatrix: Float32Array,
    projectionMatrix: Float32Array,
    canvasHeightPx: number
  ): void {
    const gl = this.gl;

    gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.velocityBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, velocities, gl.DYNAMIC_DRAW);

    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LESS);
    gl.clearColor(0.06, 0.07, 0.09, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    gl.useProgram(this.program);
    gl.uniformMatrix4fv(this.uView, false, viewMatrix);
    gl.uniformMatrix4fv(this.uProjection, false, projectionMatrix);
    gl.uniform1f(this.uScreenHeightPx, canvasHeightPx);

    gl.bindVertexArray(this.vao);
    gl.drawArrays(gl.POINTS, 0, particleCount);
    gl.bindVertexArray(null);
  }
}
