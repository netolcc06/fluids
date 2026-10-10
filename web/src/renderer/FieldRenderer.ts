// WebGL2 renderer for smoke's grid-shaped density/temperature field
// (docs/flip-smoke-sim-api.md's "Turning the density field into a texture"
// section — this is the frontend implementation of that worked example).
// Unlike ParticleRenderer's gl.POINTS, this draws a single textured quad
// spanning the sim domain [0,domainWidth] x [0,domainHeight], mapped through
// the same uScale/uOffset camera-transform convention ParticleRenderer uses
// (see computeCameraTransform in ParticleRenderer.ts) so smoke respects the
// same letterbox/pillarbox fit as particles do.
import type { CameraTransform } from './ParticleRenderer';

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
layout(location = 1) in vec2 a_uv;

uniform vec2 uScale;
uniform vec2 uOffset;

out vec2 v_uv;

void main() {
  // Same sim-space -> clip-space mapping as ParticleRenderer's vertex
  // shader; uScale/uOffset come from the identical computeCameraTransform
  // helper so smoke and particles share one letterbox/pillarbox fit.
  gl_Position = vec4(a_position * uScale + uOffset, 0.0, 1.0);
  v_uv = a_uv;
}
`;

const FRAGMENT_SHADER_SRC = `#version 300 es
precision highp float;
in vec2 v_uv;

uniform sampler2D uDensity;
uniform sampler2D uTemperature;
uniform float uAmbientTemperature;
uniform float uSourceTemperature;

out vec4 outColor;

void main() {
  float density = texture(uDensity, v_uv).r;
  float temperature = texture(uTemperature, v_uv).r;

  // Baseline: light gray/white smoke, density as opacity (per
  // docs/flip-smoke-sim-api.md: density is already ~[0,1] and safe to use
  // directly as alpha). Blended toward a warm "hot core" color near the
  // source, using temperature relative to the ambient/source range — a
  // nice-to-have per the doc, not required for a recognizable plume.
  vec3 coolColor = vec3(0.86, 0.87, 0.90);
  vec3 hotColor = vec3(1.0, 0.55, 0.18);
  float tRange = max(uSourceTemperature - uAmbientTemperature, 1e-4);
  float hotT = clamp((temperature - uAmbientTemperature) / tRange, 0.0, 1.0);
  vec3 color = mix(coolColor, hotColor, hotT * 0.55);

  float alpha = clamp(density, 0.0, 1.0);
  outColor = vec4(color, alpha);
}
`;

export class FieldRenderer {
  private readonly gl: WebGL2RenderingContext;
  private readonly program: WebGLProgram;
  private readonly vao: WebGLVertexArrayObject;
  private readonly quadBuffer: WebGLBuffer;
  private readonly densityTexture: WebGLTexture;
  private readonly temperatureTexture: WebGLTexture;

  private readonly uScale: WebGLUniformLocation;
  private readonly uOffset: WebGLUniformLocation;
  private readonly uDensity: WebGLUniformLocation;
  private readonly uTemperature: WebGLUniformLocation;
  private readonly uAmbientTemperature: WebGLUniformLocation;
  private readonly uSourceTemperature: WebGLUniformLocation;

  /** Cached quad domain size, to avoid re-uploading the (tiny) vertex
   * buffer when the domain hasn't changed frame-to-frame (it only changes
   * on reset(), and there's currently only one SmokeScenario preset). */
  private lastDomainWidth = -1;
  private lastDomainHeight = -1;

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl;
    this.program = createProgram(gl, VERTEX_SHADER_SRC, FRAGMENT_SHADER_SRC);

    const vao = gl.createVertexArray();
    if (!vao) throw new Error('Failed to create VAO.');
    this.vao = vao;
    gl.bindVertexArray(vao);

    const quadBuffer = gl.createBuffer();
    if (!quadBuffer) throw new Error('Failed to create quad buffer.');
    this.quadBuffer = quadBuffer;
    gl.bindBuffer(gl.ARRAY_BUFFER, quadBuffer);
    // Interleaved [x, y, u, v] per vertex, 4 vertices (triangle strip).
    const stride = 4 * Float32Array.BYTES_PER_ELEMENT;
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, stride, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 2, gl.FLOAT, false, stride, 2 * Float32Array.BYTES_PER_ELEMENT);

    gl.bindVertexArray(null);

    const uScale = gl.getUniformLocation(this.program, 'uScale');
    const uOffset = gl.getUniformLocation(this.program, 'uOffset');
    const uDensity = gl.getUniformLocation(this.program, 'uDensity');
    const uTemperature = gl.getUniformLocation(this.program, 'uTemperature');
    const uAmbientTemperature = gl.getUniformLocation(this.program, 'uAmbientTemperature');
    const uSourceTemperature = gl.getUniformLocation(this.program, 'uSourceTemperature');
    if (!uScale || !uOffset || !uDensity || !uTemperature || !uAmbientTemperature || !uSourceTemperature) {
      throw new Error('Failed to locate shader uniforms.');
    }
    this.uScale = uScale;
    this.uOffset = uOffset;
    this.uDensity = uDensity;
    this.uTemperature = uTemperature;
    this.uAmbientTemperature = uAmbientTemperature;
    this.uSourceTemperature = uSourceTemperature;

    // R32F textures are not filterable with LINEAR under plain WebGL2
    // unless OES_texture_float_linear is available — request it, and fall
    // back to NEAREST (still fine at this solver's grid resolutions,
    // ~60-100 cells per axis) if the extension isn't present.
    const linearFloatExt = gl.getExtension('OES_texture_float_linear');
    const filter = linearFloatExt ? gl.LINEAR : gl.NEAREST;

    this.densityTexture = this.createFieldTexture(filter);
    this.temperatureTexture = this.createFieldTexture(filter);

    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.clearColor(0.06, 0.07, 0.09, 1.0);

    gl.useProgram(this.program);
    gl.uniform1i(this.uDensity, 0);
    gl.uniform1i(this.uTemperature, 1);
  }

  private createFieldTexture(filter: number): WebGLTexture {
    const gl = this.gl;
    const tex = gl.createTexture();
    if (!tex) throw new Error('Failed to create texture.');
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, filter);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, filter);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    return tex;
  }

  setCamera(camera: CameraTransform): void {
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.uniform2f(this.uScale, camera.scaleX, camera.scaleY);
    gl.uniform2f(this.uOffset, camera.offsetX, camera.offsetY);
  }

  private updateQuadIfNeeded(domainWidth: number, domainHeight: number): void {
    if (domainWidth === this.lastDomainWidth && domainHeight === this.lastDomainHeight) return;
    this.lastDomainWidth = domainWidth;
    this.lastDomainHeight = domainHeight;

    // Domain corners in sim space, paired with UVs where v=0 maps to the
    // domain's bottom edge (y=0) and v=1 to the top (y=domainHeight). Row 0
    // of the density/temperature arrays is the domain's BOTTOM row (per
    // docs/flip-smoke-sim-api.md), and gl.texImage2D places that same row 0
    // at texture v=0 with no UNPACK_FLIP_Y — so this v mapping already
    // lines up bottom-to-bottom with no shader flip needed. (Verified
    // visually: the plume rises from the bottom of the rendered domain, not
    // the top — see Phase 4a verification screenshots.)
    // prettier-ignore
    const verts = new Float32Array([
      0,           0,            0, 0,
      domainWidth, 0,            1, 0,
      0,           domainHeight, 0, 1,
      domainWidth, domainHeight, 1, 1,
    ]);
    const gl = this.gl;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.quadBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.DYNAMIC_DRAW);
  }

  /** Upload the current frame's density/temperature fields as R32F textures
   * and draw the domain-spanning quad. `domainWidth`/`domainHeight` should
   * come from the smoke controller (SmokeScenario's fields) — see
   * FlipSmokeSimulationController.domainWidth/domainHeight. */
  render(
    density: Float32Array,
    temperature: Float32Array,
    gridWidth: number,
    gridHeight: number,
    domainWidth: number,
    domainHeight: number,
    ambientTemperature: number,
    sourceTemperature: number
  ): void {
    const gl = this.gl;

    this.updateQuadIfNeeded(domainWidth, domainHeight);

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.densityTexture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.R32F, gridWidth, gridHeight, 0, gl.RED, gl.FLOAT, density);

    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.temperatureTexture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.R32F, gridWidth, gridHeight, 0, gl.RED, gl.FLOAT, temperature);

    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.useProgram(this.program);
    gl.uniform1f(this.uAmbientTemperature, ambientTemperature);
    gl.uniform1f(this.uSourceTemperature, sourceTemperature);
    gl.bindVertexArray(this.vao);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    gl.bindVertexArray(null);
  }
}
