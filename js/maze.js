/* ============================================================================
   maze.js — 16x16 maze model, wall geometry and VL6180 ray-cast sensor physics
   ----------------------------------------------------------------------------
   Grid convention (identical to the floodfill module):
     index = row * 16 + col,  row 0 / col 0 = bottom-left corner (the start cell)
     direction bits: 0 = N (row+1), 1 = E (col+1), 2 = S (row-1), 3 = W (col-1)

   Maze difficulty is controlled by four knobs:
     PULL_AWAY_FROM_GOAL   the primary difficulty lever. When true, the DFS
                           neighbour choice is biased AWAY from the goal cells,
                           so the start-to-centre path is forced to wander
                           before arriving. This produces long outbound routes
                           (typically 80-140 cells) without needing to densify
                           the maze, so the divergent-return phase still has a
                           real alternate route to find.
     EXTRA_LOOPS           number of additional wall openings after the DFS
                           carve. Lower = fewer shortcuts = harder. The
                           two-disjoint-routes guarantee below still fires
                           regardless, so the return route always exists.
     CARVE_DENSITY         probability a candidate DFS move is taken. Leave at
                           1.0 for the standard tree; lower it only if you want
                           extra walls at the cost of longer solve times.
     PRUNE_CHANCE          probability an accidental loop edge is re-closed
                           post-carve. Leave at 0.0 for a healthy loop network.
   ========================================================================== */

const MAZE_N = 16;
const CELL_MM = 180.0;
const WALL_T_MM = 12.0;
const HALF_T = WALL_T_MM / 2;

const DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3;
const DIR_NAME = ['N', 'E', 'S', 'W'];
const DIR_DR = [1, 0, -1, 0];
const DIR_DC = [0, 1, 0, -1];
const DIR_ANGLE = [90, 0, 270, 180];

const idx = (r, c) => r * MAZE_N + c;
const rowOf = (i) => Math.floor(i / MAZE_N);
const colOf = (i) => i % MAZE_N;

/* --------------------------- maze generation ------------------------------ */

// --- difficulty knobs ------------------------------------------------------
const CARVE_DENSITY       = 1.0;
const PRUNE_CHANCE        = 0.0;
const EXTRA_LOOPS         = 10;
const PULL_AWAY_FROM_GOAL = true;
// ---------------------------------------------------------------------------

function makeMaze(seed) {
  let s = (seed === undefined) ? (Math.random() * 1e9) | 0 : seed;
  const rnd = () => {
    s ^= s << 13; s ^= s >>> 17; s ^= s << 5; s |= 0;
    return ((s >>> 0) % 100000) / 100000;
  };

  const walls = new Uint8Array(MAZE_N * MAZE_N);
  const visited = new Uint8Array(MAZE_N * MAZE_N);
  const clearWall = (r, c, d) => {
    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    walls[idx(r, c)] &= ~(1 << d);
    if (nr >= 0 && nr < MAZE_N && nc >= 0 && nc < MAZE_N) walls[idx(nr, nc)] &= ~(1 << ((d + 2) % 4));
  };
  const setWall = (r, c, d) => {
    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    walls[idx(r, c)] |= (1 << d);
    if (nr >= 0 && nr < MAZE_N && nc >= 0 && nc < MAZE_N) walls[idx(nr, nc)] |= (1 << ((d + 2) % 4));
  };

  // --- goal block ---------------------------------------------------------
  const G = [[7, 7], [7, 8], [8, 7], [8, 8]];
  const goalCells = G.map(([r, c]) => idx(r, c));
  const goalDist = new Int16Array(MAZE_N * MAZE_N);
  for (let i = 0; i < goalCells.length; i++) {
    const g = goalCells[i];
    const gr = rowOf(g), gc = colOf(g);
    for (let r = 0; r < MAZE_N; r++) {
      for (let c = 0; c < MAZE_N; c++) {
        const d = Math.abs(r - gr) + Math.abs(c - gc);
        if (i === 0 || d < goalDist[idx(r, c)]) goalDist[idx(r, c)] = d;
      }
    }
  }

  // --- DFS carve with goal-distance bias ----------------------------------
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

    let d;
    if (PULL_AWAY_FROM_GOAL && rnd() < 0.70) {
      // Prefer the neighbour with the largest distance to the goal, so the
      // current path curls away from the centre before it turns back.
      let bestD = -1, bestVal = -1;
      for (const cd of cand) {
        const nr = r + DIR_DR[cd], nc = c + DIR_DC[cd];
        const v = goalDist[idx(nr, nc)];
        if (v > bestVal) { bestVal = v; bestD = cd; }
      }
      d = bestD >= 0 ? bestD : cand[(rnd() * cand.length) | 0];
    } else {
      d = cand[(rnd() * cand.length) | 0];
    }

    // Density gate (leave at 1.0 for the standard tree).
    if (rnd() >= CARVE_DENSITY && cand.length > 1) {
      const alt = cand.filter(x => x !== d);
      if (alt.length) d = alt[(rnd() * alt.length) | 0];
    }

    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    clearWall(r, c, d);
    visited[idx(nr, nc)] = 1;
    stack.push([nr, nc]);
  }

  // --- goal block internals + at least two entrances ----------------------
  clearWall(7, 7, DIR_E);
  clearWall(8, 7, DIR_E);
  clearWall(7, 7, DIR_N);
  clearWall(7, 8, DIR_N);
  const entrances = [
    [7, 7, DIR_S], [8, 7, DIR_S], [7, 7, DIR_W], [8, 7, DIR_W],
    [7, 8, DIR_E], [8, 8, DIR_E], [8, 7, DIR_N], [8, 8, DIR_N]
  ];
  for (let k = 0; k < 2; k++) {
    const pick = entrances.splice((rnd() * entrances.length) | 0, 1)[0];
    if (pick) clearWall(pick[0], pick[1], pick[2]);
  }

  // --- extra loops --------------------------------------------------------
  for (let k = 0; k < EXTRA_LOOPS; k++) {
    const r = (rnd() * MAZE_N) | 0, c = (rnd() * MAZE_N) | 0, d = (rnd() * 4) | 0;
    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    if (nr < 0 || nr >= MAZE_N || nc < 0 || nc >= MAZE_N) continue;
    clearWall(r, c, d);
  }

  // --- optional post-carve pruning (off by default) -----------------------
  if (PRUNE_CHANCE > 0) {
    function isConnected() {
      const seen = new Uint8Array(MAZE_N * MAZE_N);
      const q = [0];
      seen[0] = 1;
      let h = 0, count = 1;
      while (h < q.length) {
        const u = q[h++];
        for (let d = 0; d < 4; d++) {
          if (walls[u] & (1 << d)) continue;
          const r = (u / MAZE_N) | 0, c = u % MAZE_N;
          const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
          if (nr < 0 || nr >= MAZE_N || nc < 0 || nc >= MAZE_N) continue;
          const v = idx(nr, nc);
          if (seen[v]) continue;
          seen[v] = 1; count++; q.push(v);
        }
      }
      return count === MAZE_N * MAZE_N;
    }
    const interiorEdges = [];
    for (let r = 0; r < MAZE_N; r++) {
      for (let c = 0; c < MAZE_N; c++) {
        const i = idx(r, c);
        for (const d of [DIR_N, DIR_E]) {
          if (walls[i] & (1 << d)) continue;
          const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
          if (nr < 0 || nr >= MAZE_N || nc < 0 || nc >= MAZE_N) continue;
          interiorEdges.push([r, c, d]);
        }
      }
    }
    for (let k = interiorEdges.length - 1; k > 0; k--) {
      const j = (rnd() * (k + 1)) | 0;
      [interiorEdges[k], interiorEdges[j]] = [interiorEdges[j], interiorEdges[k]];
    }
    for (const [r, c, d] of interiorEdges) {
      if (rnd() >= PRUNE_CHANCE) continue;
      setWall(r, c, d);
      if (!isConnected()) clearWall(r, c, d);
    }
  }

  // --- guarantee two disjoint start->goal routes --------------------------
  function bfsPath(from, to, removedEdges) {
    const prev = new Int16Array(MAZE_N * MAZE_N).fill(-1);
    const seen = new Uint8Array(MAZE_N * MAZE_N);
    const q = [from];
    seen[from] = 1;
    let h = 0;
    while (h < q.length) {
      const u = q[h++];
      if (u === to) {
        const path = [];
        let v = to;
        while (v !== -1) { path.push(v); v = prev[v]; }
        return path.reverse();
      }
      for (let d = 0; d < 4; d++) {
        if (walls[u] & (1 << d)) continue;
        const r = (u / MAZE_N) | 0, c = u % MAZE_N;
        const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
        if (nr < 0 || nr >= MAZE_N || nc < 0 || nc >= MAZE_N) continue;
        const v = idx(nr, nc);
        if (seen[v]) continue;
        if (removedEdges && removedEdges.has(u * 1000 + v)) continue;
        seen[v] = 1;
        prev[v] = u;
        q.push(v);
      }
    }
    return null;
  }

  function hasTwoDisjointRoutes() {
    let p1 = null;
    for (const g of goalCells) {
      p1 = bfsPath(0, g, null);
      if (p1) break;
    }
    if (!p1) return false;
    const removed = new Set();
    for (let i = 0; i + 1 < p1.length; i++) {
      removed.add(p1[i] * 1000 + p1[i + 1]);
      removed.add(p1[i + 1] * 1000 + p1[i]);
    }
    for (const g of goalCells) {
      if (bfsPath(0, g, removed)) return true;
    }
    return false;
  }

  let attempts = 0;
  while (!hasTwoDisjointRoutes() && attempts++ < 400) {
    const r = (rnd() * MAZE_N) | 0, c = (rnd() * MAZE_N) | 0, d = (rnd() * 4) | 0;
    const nr = r + DIR_DR[d], nc = c + DIR_DC[d];
    if (nr < 0 || nr >= MAZE_N || nc < 0 || nc >= MAZE_N) continue;
    clearWall(r, c, d);
  }

  return { walls, seedN: s, goal: goalCells };
}

/* --------------------------- wall rectangles ------------------------------ */

function wallRects(walls, filterKnown) {
  const rects = [];
  const N = MAZE_N;
  for (let r = 0; r < N; r++) {
    for (let c = 0; c < N; c++) {
      const m = walls[idx(r, c)];
      const x0 = c * CELL_MM, x1 = x0 + CELL_MM;
      const y0 = r * CELL_MM, y1 = y0 + CELL_MM;
      if (m & (1 << DIR_N)) rects.push([x0 - HALF_T, y1 - HALF_T, x1 + HALF_T, y1 + HALF_T, idx(r, c), DIR_N]);
      if (m & (1 << DIR_E)) rects.push([x1 - HALF_T, y0 - HALF_T, x1 + HALF_T, y1 + HALF_T, idx(r, c), DIR_E]);
      if (r === 0 && (m & (1 << DIR_S))) rects.push([x0 - HALF_T, y0 - HALF_T, x1 + HALF_T, y0 + HALF_T, idx(r, c), DIR_S]);
      if (c === 0 && (m & (1 << DIR_W))) rects.push([x0 - HALF_T, y0 - HALF_T, x0 + HALF_T, y1 + HALF_T, idx(r, c), DIR_W]);
    }
  }
  if (filterKnown) return rects.filter(([, , , , ci, d]) => filterKnown[ci] & (1 << d));
  return rects;
}

/* ------------------------------ ray cast ---------------------------------- */

function rayRect(px, py, dx, dy, rect) {
  let tmin = 0, tmax = Infinity;
  if (Math.abs(dx) < 1e-9) {
    if (px < rect[0] || px > rect[2]) return Infinity;
  } else {
    let t1 = (rect[0] - px) / dx, t2 = (rect[2] - px) / dx;
    if (t1 > t2) { const t = t1; t1 = t2; t2 = t; }
    tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
    if (tmin > tmax) return Infinity;
  }
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

const VL6180_MAX_MM = 200;

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

function resolveCollision(px, py, nx, ny, rb, rects) {
  let bumped = false;
  let vx = nx - px, vy = ny - py;
  for (let k = 0; k < 2; k++) {
    for (let i = 0; i < rects.length; i++) {
      const [x0, y0, x1, y1] = rects[i];
      if (x0 > px + rb || x1 < px - rb || y0 > py + rb || y1 < py - rb) continue;
      const cx = Math.max(x0, Math.min(px, x1));
      const cy = Math.max(y0, Math.min(py, y1));
      let dx = px - cx, dy = py - cy;
      let d2 = dx * dx + dy * dy;
      if (d2 >= rb * rb) continue;
      bumped = true;
      let d = Math.sqrt(d2);
      if (d < 1e-6) {
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
      const into = vx * nxu + vy * nyu;
      if (into < 0) { vx -= nxu * into; vy -= nyu * into; }
    }
  }
  return { x: px + vx, y: py + vy, bumped };
}
