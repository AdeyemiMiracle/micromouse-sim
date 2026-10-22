/* ============================================================================
   maze.js — 16x16 maze model, wall geometry and VL6180 ray-cast sensor physics
   ----------------------------------------------------------------------------
   Grid convention (identical to the floodfill module):
     index = row * 16 + col,  row 0 / col 0 = bottom-left corner (the start cell)
     direction bits: 0 = N (row+1), 1 = E (col+1), 2 = S (row-1), 3 = W (col-1)
   ========================================================================== */

const MAZE_N = 16;            // 16 x 16 cells
const CELL_MM = 180.0;        // cell pitch (firmware uses 168 mm for its 8x8 maze)
const WALL_T_MM = 12.0;       // wall thickness
const HALF_T = WALL_T_MM / 2;

const DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3;
const DIR_NAME = ['N', 'E', 'S', 'W'];
const DIR_DR = [1, 0, -1, 0];   // N increases the row index
const DIR_DC = [0, 1, 0, -1];
const DIR_ANGLE = [90, 0, 270, 180];   // degrees CCW from +x (East). N = +y = 90 deg

const idx = (r, c) => r * MAZE_N + c;
const rowOf = (i) => Math.floor(i / MAZE_N);
const colOf = (i) => i % MAZE_N;

/* --------------------------- maze generation ------------------------------ */

function makeMaze(seed) {
  let s = (seed === undefined) ? (Math.random() * 1e9) | 0 : seed;
  const rnd = () => { // xorshift — reproducible mazes
    s ^= s << 13; s ^= s >>> 17; s ^= s << 5; s |= 0;
    return ((s >>> 0) % 100000) / 100000;
  };

  const walls = new Uint8Array(MAZE_N * MAZE_N);   // bitmask per cell
  const visited = new Uint8Array(MAZE_N * MAZE_N);
  const clearWall = (r, c, d) => {
    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    walls[idx(r, c)] &= ~(1 << d);
    if (nr >= 0 && nr < MAZE_N && nc >= 0 && nc < MAZE_N) walls[idx(nr, nc)] &= ~(1 << ((d + 2) % 4));
  };

  // start with every wall up, carve with randomised DFS
  walls.fill(0x0F);
  const stack = [[0, 0]];
  visited[idx(0, 0)] = 1;
  while (stack.length) {
    const [r, c] = stack[stack.length - 1];
    const cand = [];
    for (let d = 0; d < 4; d++) {
      const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
      if (nr >= 0 && nr < MAZE_N && nc >= 0 && nc < MAZE_N && !visited[idx(nr, nc)]) cand.push(d);
    }
    if (!cand.length) { stack.pop(); continue; }
    const d = cand[(rnd() * cand.length) | 0];
    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    clearWall(r, c, d);
    visited[idx(nr, nc)] = 1;
    stack.push([nr, nc]);
  }

  // --- goal block: the four centre cells, opened internally ---------------
  const G = [[7, 7], [7, 8], [8, 7], [8, 8]];
  clearWall(7, 7, DIR_E);          // (7,7)-(7,8)
  clearWall(8, 7, DIR_E);          // (8,7)-(8,8)
  clearWall(7, 7, DIR_N);          // (7,7)-(8,7)
  clearWall(7, 8, DIR_N);          // (7,8)-(8,8)
  // guarantee at least two entrances into the goal block
  const entrances = [
    [7, 7, DIR_S], [8, 7, DIR_S], [7, 7, DIR_W], [8, 7, DIR_W],
    [7, 8, DIR_E], [8, 8, DIR_E], [8, 7, DIR_N], [8, 8, DIR_N]
  ];
  for (let k = 0; k < 2; k++) {
    const pick = entrances.splice((rnd() * entrances.length) | 0, 1)[0];
    if (pick) clearWall(pick[0], pick[1], pick[2]);
  }

  // a few extra loops make the maze less "perfect" and the flood fill more interesting
  const extra = 14;
  for (let k = 0; k < extra; k++) {
    const r = (rnd() * MAZE_N) | 0, c = (rnd() * MAZE_N) | 0, d = (rnd() * 4) | 0;
    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    if (nr < 0 || nr >= MAZE_N || nc < 0 || nc >= MAZE_N) continue;
    clearWall(r, c, d);
  }

  return { walls, seedN: s, goal: G.map(([r, c]) => idx(r, c)) };
}

/* --------------------------- wall rectangles ------------------------------ */
/* Each wall is an axis-aligned rectangle used both for drawing and for the
   sensor ray cast (slab intersection). Units: mm, world origin = maze corner. */

function wallRects(walls, filterKnown) {
  const rects = [];
  const N = MAZE_N;
  for (let r = 0; r < N; r++) {
    for (let c = 0; c < N; c++) {
      const m = walls[idx(r, c)];
      const x0 = c * CELL_MM, x1 = x0 + CELL_MM;
      const y0 = r * CELL_MM, y1 = y0 + CELL_MM;
      // N wall (top edge of the cell)
      if (m & (1 << DIR_N)) rects.push([x0 - HALF_T, y1 - HALF_T, x1 + HALF_T, y1 + HALF_T, idx(r, c), DIR_N]);
      // E wall
      if (m & (1 << DIR_E)) rects.push([x1 - HALF_T, y0 - HALF_T, x1 + HALF_T, y1 + HALF_T, idx(r, c), DIR_E]);
      // perimeter S / W only on the boundary rows/cols (avoids duplicates)
      if (r === 0 && (m & (1 << DIR_S))) rects.push([x0 - HALF_T, y0 - HALF_T, x1 + HALF_T, y0 + HALF_T, idx(r, c), DIR_S]);
      if (c === 0 && (m & (1 << DIR_W))) rects.push([x0 - HALF_T, y0 - HALF_T, x0 + HALF_T, y1 + HALF_T, idx(r, c), DIR_W]);
    }
  }
  if (filterKnown) return rects.filter(([, , , , ci, d]) => filterKnown[ci] & (1 << d));
  return rects;
}

/* ------------------------------ ray cast ---------------------------------- */
/* Nearest hit distance (mm) along a unit ray, or Infinity when nothing is hit.
   Slab method against an axis-aligned rectangle. */
function rayRect(px, py, dx, dy, rect) {
  let tmin = 0, tmax = Infinity;
  // x slab
  if (Math.abs(dx) < 1e-9) {
    if (px < rect[0] || px > rect[2]) return Infinity;
  } else {
    let t1 = (rect[0] - px) / dx, t2 = (rect[2] - px) / dx;
    if (t1 > t2) { const t = t1; t1 = t2; t2 = t; }
    tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
    if (tmin > tmax) return Infinity;
  }
  // y slab
  if (Math.abs(dy) < 1e-9) {
    if (py < rect[1] || py > rect[3]) return Infinity;
  } else {
    let t1 = (rect[1] - py) / dy, t2 = (rect[3] - py) / dy;
    if (t1 > t2) { const t = t1; t1 = t2; t2 = t; }
    tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
    if (tmin > tmax) return Infinity;
  }
  return tmin;
}

const VL6180_MAX_MM = 200;   // sensor specification used by readsingleSensor()

/* Cast one sensor beam and return the VL6180-style reading:
   millimetres to the wall surface, or 999 when there is no echo inside 200 mm. */
function castSensor(px, py, angleDeg, rects, noise) {
  const a = angleDeg * Math.PI / 180;
  const dx = Math.cos(a), dy = Math.sin(a);
  let best = Infinity;
  for (let i = 0; i < rects.length; i++) {
    const t = rayRect(px, py, dx, dy, rects[i]);
    if (t < best) best = t;
  }
  if (!isFinite(best) || best > VL6180_MAX_MM) return 999;
  let d = best + (noise ? (Math.random() * 2 - 1) * noise : 0);
  if (d < 0) d = 0;
  return Math.round(d);
}

/* --------------------------- body / collision ----------------------------- */
/* Wall contact for the circular chassis (radius rb): push the body out of any
   wall rectangle it overlaps and project the attempted displacement (nx-px,
   ny-py) onto the wall, so a car that touches a wall scrapes ALONG it instead of
   sticking to it. Returns the corrected position and whether contact happened. */
function resolveCollision(px, py, nx, ny, rb, rects) {
  let bumped = false;
  let vx = nx - px, vy = ny - py;
  for (let k = 0; k < 2; k++) {          // two relaxation passes
    for (let i = 0; i < rects.length; i++) {
      const [x0, y0, x1, y1] = rects[i];
      if (x0 > px + rb || x1 < px - rb || y0 > py + rb || y1 < py - rb) continue;  // far away
      const cx = Math.max(x0, Math.min(px, x1));
      const cy = Math.max(y0, Math.min(py, y1));
      let dx = px - cx, dy = py - cy;
      let d2 = dx * dx + dy * dy;
      if (d2 >= rb * rb) continue;
      bumped = true;
      let d = Math.sqrt(d2);
      if (d < 1e-6) {                     // centre inside the rectangle: push out on the shallow axis
        const cxr = (x0 + x1) / 2, cyr = (y0 + y1) / 2;
        const ox = (x1 - x0) / 2 + rb - Math.abs(px - cxr);
        const oy = (y1 - y0) / 2 + rb - Math.abs(py - cyr);
        if (ox < oy) { px = (px < cxr) ? x0 - rb : x1 + rb; dx = (px < cxr) ? -1 : 1; dy = 0; }
        else { py = (py < cyr) ? y0 - rb : y1 + rb; dx = 0; dy = (py < cyr) ? -1 : 1; }
        d = 1;
      }
      const nxu = dx / d, nyu = dy / d;
      px += nxu * (rb - d);
      py += nyu * (rb - d);
      const into = vx * nxu + vy * nyu;   // negative = moving into the wall
      if (into < 0) { vx -= nxu * into; vy -= nyu * into; }   // slide along it
    }
  }
  return { x: px + vx, y: py + vy, bumped };
}
