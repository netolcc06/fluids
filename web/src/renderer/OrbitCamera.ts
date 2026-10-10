// Standard orbit camera: spherical coordinates (azimuth, elevation,
// distance) around a target point, converted to a view matrix via a
// standard right-handed look-at construction, combined with a perspective
// projection matrix. Self-contained plain math — no matrix library
// dependency (this project loads none anywhere; a 4x4 helper this small
// doesn't justify starting).
//
// Coordinate convention (per docs/sph3d-sim-api.md): +x right, +y up, +z
// toward the viewer, right-handed. A standard look-at camera (forward = -z
// in view space) matches this directly, so no axis remapping is needed at
// the render boundary.

interface Vec3 {
  x: number;
  y: number;
  z: number;
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  };
}

function dot(a: Vec3, b: Vec3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function normalize(v: Vec3): Vec3 {
  const len = Math.hypot(v.x, v.y, v.z) || 1;
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

/** Column-major 4x4 look-at view matrix (standard right-handed
 * construction, camera looks down -z in its own view space). */
function lookAt(eye: Vec3, target: Vec3, up: Vec3): Float32Array {
  const f = normalize(sub(target, eye)); // forward
  const s = normalize(cross(f, up)); // right
  const u = cross(s, f); // recomputed up (orthogonal to f and s)
  const m = new Float32Array(16);
  m[0] = s.x;
  m[1] = u.x;
  m[2] = -f.x;
  m[3] = 0;
  m[4] = s.y;
  m[5] = u.y;
  m[6] = -f.y;
  m[7] = 0;
  m[8] = s.z;
  m[9] = u.z;
  m[10] = -f.z;
  m[11] = 0;
  m[12] = -dot(s, eye);
  m[13] = -dot(u, eye);
  m[14] = dot(f, eye);
  m[15] = 1;
  return m;
}

/** Column-major 4x4 right-handed perspective projection matrix, NDC z in
 * [-1, 1] (standard WebGL/OpenGL convention). */
function perspective(fovYRad: number, aspect: number, near: number, far: number): Float32Array {
  const f = 1.0 / Math.tan(fovYRad / 2);
  const m = new Float32Array(16);
  m[0] = f / aspect;
  m[5] = f;
  m[10] = (far + near) / (near - far);
  m[11] = -1;
  m[14] = (2 * far * near) / (near - far);
  return m;
}

/**
 * Orbit camera around a target point. Mouse-drag rotates (azimuth/
 * elevation), scroll wheel zooms (distance). Elevation is clamped strictly
 * inside +-90 degrees so the camera can never flip through the poles (up
 * vector would degenerate at exactly +-90).
 *
 * Listens on the given canvas for drag/zoom input directly (self-contained,
 * per the project's "keep it minimal, no framework" pattern) — `mousemove`/
 * `mouseup` are attached to `window` rather than the canvas so a drag
 * continues correctly even if the cursor leaves the canvas mid-drag, a
 * standard orbit-camera UX detail.
 */
export class OrbitCamera {
  azimuth = Math.PI / 4;
  elevation = 0.5;
  distance = 2.0;
  target: Vec3 = { x: 0, y: 0, z: 0 };

  minDistance = 0.15;
  maxDistance = 30;
  private readonly maxElevation = 1.45; // radians, ~83 degrees — stays clear of the poles
  private readonly minElevation = -1.45;

  fovYRad = (55 * Math.PI) / 180;
  near = 0.02;
  far = 100;

  private dragging = false;
  private lastX = 0;
  private lastY = 0;

  private readonly onMouseDown = (e: MouseEvent): void => {
    this.dragging = true;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
  };
  private readonly onMouseMove = (e: MouseEvent): void => {
    if (!this.dragging) return;
    const dx = e.clientX - this.lastX;
    const dy = e.clientY - this.lastY;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
    const rotateSpeed = 0.006;
    this.azimuth -= dx * rotateSpeed;
    this.elevation = Math.max(this.minElevation, Math.min(this.maxElevation, this.elevation - dy * rotateSpeed));
  };
  private readonly onMouseUp = (): void => {
    this.dragging = false;
  };
  private readonly onWheel = (e: WheelEvent): void => {
    e.preventDefault();
    const zoomSpeed = 0.0018;
    const factor = Math.exp(e.deltaY * zoomSpeed);
    this.distance = Math.max(this.minDistance, Math.min(this.maxDistance, this.distance * factor));
  };

  constructor(private readonly canvas: HTMLCanvasElement) {
    canvas.addEventListener('mousedown', this.onMouseDown);
    window.addEventListener('mousemove', this.onMouseMove);
    window.addEventListener('mouseup', this.onMouseUp);
    canvas.addEventListener('wheel', this.onWheel, { passive: false });
  }

  setTarget(x: number, y: number, z: number): void {
    this.target = { x, y, z };
  }

  private eye(): Vec3 {
    const cosEl = Math.cos(this.elevation);
    return {
      x: this.target.x + this.distance * cosEl * Math.sin(this.azimuth),
      y: this.target.y + this.distance * Math.sin(this.elevation),
      z: this.target.z + this.distance * cosEl * Math.cos(this.azimuth),
    };
  }

  getViewMatrix(): Float32Array {
    return lookAt(this.eye(), this.target, { x: 0, y: 1, z: 0 });
  }

  getProjectionMatrix(aspect: number): Float32Array {
    return perspective(this.fovYRad, aspect, this.near, this.far);
  }

  /** Removes this camera's DOM listeners. Not currently called anywhere
   * (the app constructs exactly one OrbitCamera for its lifetime), provided
   * for completeness/future use rather than left as a latent leak. */
  dispose(): void {
    this.canvas.removeEventListener('mousedown', this.onMouseDown);
    window.removeEventListener('mousemove', this.onMouseMove);
    window.removeEventListener('mouseup', this.onMouseUp);
    this.canvas.removeEventListener('wheel', this.onWheel);
  }
}
