/* ============================================================================
   app.js — rendering, telemetry, pipeline display and the animation clock
   ========================================================================== */

/* Surface any startup/loop exception on the page instead of failing silently. */
window.addEventListener('error', (e) => {
  const b = document.getElementById('bootError');
  if (!b) return;
  b.hidden = false;
  b.textContent = 'Error: ' + (e.message || e) +
    (e.filename ? `  (${e.filename}:${e.lineno})` : '') + ' — see the browser console.';
});

const canvas = document.getElementById('mazeCanvas');
const ctx2d = canvas.getContext('2d');
const MARGIN = 22;
const CELL_PX = (canvas.width - 2 * MARGIN) / MAZE_N;

const MODULES = [
  { key: 'MAIN', name: 'main.ino', cls: '' },
  { key: 'VL6180', name: 'VL6180 module', cls: '' },
  { key: 'FF', name: 'floodfill module', cls: '' },
  { key: 'MPU', name: 'mpu6050 module', cls: '' },
  { key: 'MOTION', name: 'motion module', cls: 'turn' },
  { key: 'DISTANCE', name: 'distance module', cls: 'drive' },
  { key: 'MOTOR', name: 'MotorControl module', cls: '' },
  { key: 'ENC', name: 'Encoder module', cls: '' }
];

let maze = makeMaze();
let robot = new Robot();
let sup = null;
let running = false;
let speedMul = 8;
let trailPts = [];
let lastTrailT = -1;
let active = new Set();
let msgs = {};
const ctxObj = {
  log: (mod, msg) => addLog(mod, msg)
};

/* ----------------------------- boot --------------------------------------- */
function boot(newMaze) {
  if (newMaze) maze = makeMaze();
  robot.reset(maze);
  sup = new Supervisor(robot, ctxObj);
  trailPts = [];
  lastTrailT = -1;
  setActiveReporter((key, msg) => { active.add(key); msgs[key] = msg; });
  addLog('MAIN', `new maze generated — start cell (0,0) facing N, goal = centre 2x2`);
}

/* -------------------------------- log ------------------------------------- */
const logEl = document.getElementById('log');
function addLog(mod, msg) {
  const d = document.createElement('div');
  const t = (sup ? sup.r.t : robot.t) / 1000;
  d.innerHTML = `<span class="t">${t.toFixed(2)}s</span> <span class="m ${mod}">[${mod}]</span> ${msg}`;
  logEl.appendChild(d);
  while (logEl.childElementCount > 300) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop = logEl.scrollHeight;
}

/* ----------------------------- pipeline ----------------------------------- */
const pipeEl = document.getElementById('pipeline');
pipeEl.innerHTML = MODULES.map(m =>
  `<div class="pcard ${m.cls}" id="pc_${m.key}"><span class="pname">${m.name}</span><span class="pmsg" id="pm_${m.key}"></span></div>`
).join('<div class="parrow">▼</div>');

function paintPipeline() {
  for (const m of MODULES) {
    const card = document.getElementById('pc_' + m.key);
    const msg = document.getElementById('pm_' + m.key);
    card.classList.toggle('active', active.has(m.key));
    if (msgs[m.key] !== undefined) msg.textContent = msgs[m.key];
  }
}

/* ---------------------------- telemetry ----------------------------------- */
const telEl = document.getElementById('tel');
const TEL_ROWS = [
  ['ToF left', () => fmtMM(robot.sens.l), () => robot.sens.l < WALL_SIDE_MM],
  ['ToF front', () => fmtMM(robot.sens.m), () => robot.sens.m < WALL_FRONT_MM],
  ['ToF right', () => fmtMM(robot.sens.r), () => robot.sens.r < WALL_SIDE_MM],
  ['heading', () => robot.th.toFixed(1) + '°', () => false],
  ['cell', () => `(${rowOf(FF.currentCell)},${colOf(FF.currentCell)})`, () => false],
  ['dir', () => DIR_NAME[FF.currentDir], () => false],
  ['flood', () => String(FF.flood[FF.currentCell].flood), () => false],
  ['target', () => sup && sup.target >= 0 ? String(sup.target) : '–', () => false],
  ['yaw error', () => robot.dbg.yawError.toFixed(2) + '°', () => Math.abs(robot.dbg.yawError) > 3],
  ['yaw corr', () => robot.dbg.correction.toFixed(1), () => false],
  ['yaw integral', () => robot.dbg.integral.toFixed(0), () => false],
  ['enc diff', () => robot.dbg.encDiff.toFixed(1) + ' rpm', () => false],
  ['wall trim', () => robot.dbg.trim.toFixed(1), () => Math.abs(robot.dbg.trim) > 20],
  ['PWM L / R', () => `${robot.pwmL.toFixed(0)} / ${robot.pwmR.toFixed(0)}`, () => false],
  ['RPM L / R', () => { const r = Encoder.rpm(robot); return `${r.l.toFixed(0)} / ${r.r.toFixed(0)}`; }, () => false],
  ['steps L / R', () => { const c = Encoder.counts(robot); return `${c.l} / ${c.r}`; }, () => false],
  ['odometry', () => (robot.odometer / 1000).toFixed(2) + ' m', () => false]
];
telEl.innerHTML = TEL_ROWS.map((r, i) =>
  `<div class="tel-row"><span>${r[0]}</span><b id="tv_${i}">–</b></div>`).join('');

function fmtMM(v) { return v >= 999 ? '999 (no echo)' : v + ' mm'; }

let lastTel = 0;
function paintTelemetry(now) {
  if (now - lastTel < 100) return;
  lastTel = now;
  for (let i = 0; i < TEL_ROWS.length; i++) {
    const el = document.getElementById('tv_' + i);
    let v;
    try { v = TEL_ROWS[i][1](); } catch (e) { v = '–'; }
    el.textContent = v;
    el.className = TEL_ROWS[i][2]() ? 'warn' : (i < 3 || i >= 8 ? 'hi' : '');
  }
  document.getElementById('hudCell').textContent = `(${rowOf(FF.currentCell)},${colOf(FF.currentCell)})`;
  document.getElementById('hudDir').textContent = DIR_NAME[FF.currentDir];
  document.getElementById('hudFlood').textContent = FF.flood[FF.currentCell].flood;
  document.getElementById('hudTarget').textContent = sup && sup.target >= 0 ? sup.target : '–';
  const badge = document.getElementById('phaseBadge');
  badge.textContent = sup ? sup.phase : 'IDLE';
  badge.className = 'phase-badge ' + (sup ? sup.phase : '');
  document.getElementById('simClock').textContent = (robot.t / 1000).toFixed(2) + ' s';
  document.getElementById('actionName').textContent = sup && sup.action ? sup.action.type : '–';

  const pct = wallLearnPct();
  document.getElementById('stCells').textContent = `${FF.visitedCount()} / ${CELLS}`;
  document.getElementById('stWalls').textContent = pct.toFixed(0) + '%';
  document.getElementById('stMoves').textContent = String(sup ? sup.moves : 0);
  document.getElementById('stDist').textContent = (robot.odometer / 1000).toFixed(2) + ' m';
  document.getElementById('stTurns').textContent = String(sup ? sup.turns : 0);
  document.getElementById('stAborts').textContent = String(sup ? sup.aborts : 0);

  document.getElementById('strategyLabel').textContent =
    STRATEGY === 'GOAL_FIRST' ? 'goal-first' : 'full-map';
  document.getElementById('stBefore').textContent = sup && sup.cellsAtGoal
    ? `${sup.cellsAtGoal} cells · ${(sup.timeAtGoalMs / 1000).toFixed(0)} s`
    : '–';
  const opt = optimalRouteLen();
  document.getElementById('stSprint').textContent =
    (sup && sup.sprintLen ? String(sup.sprintLen) : '–') + ` / opt ${opt < 0 ? '?' : opt}`;
}

/* Shortest start->centre route through the TRUE maze (BFS). Used only as a
   yardstick in the stats panel: it shows how close a sprint planned from a
   partial map (goal-first) gets to the unbeatable full-map route. */
function optimalRouteLen() {
  const dist = new Int16Array(CELLS).fill(-1);
  const q = [0];
  dist[0] = 0;
  let h = 0;
  while (h < q.length) {
    const loc = q[h++];
    if (GOAL_CELLS.includes(loc)) return dist[loc];
    for (let d = 0; d < 4; d++) {
      if (!FF.neighbourValid(loc, d)) continue;
      if (maze.walls[loc] & (1 << d)) continue;
      const n = loc + DIR_DR[d] * MAZE_N + DIR_DC[d];
      if (dist[n] !== -1) continue;
      dist[n] = dist[loc] + 1;
      q.push(n);
    }
  }
  return -1;
}

function wallLearnPct() {
  const seen = new Map();
  for (let r = 0; r < MAZE_N; r++) {
    for (let c = 0; c < MAZE_N; c++) {
      const m = maze.walls[idx(r, c)], i = idx(r, c);
      for (let d = 0; d < 4; d++) {
        if (!(m & (1 << d))) continue;
        let key;
        if (d === DIR_N) key = 'h' + (r + 1) + '_' + c;
        else if (d === DIR_S) key = 'h' + r + '_' + c;
        else if (d === DIR_E) key = 'v' + r + '_' + (c + 1);
        else key = 'v' + r + '_' + c;
        if (!seen.has(key)) seen.set(key, ((FF.flood[i].nb >> d) & 1) ? 1 : 0);
      }
    }
  }
  let known = 0, total = 0;
  for (const v of seen.values()) { total++; known += v; }
  return total ? known / total * 100 : 0;
}

/* ------------------------------ drawing ----------------------------------- */
const sx = (x) => MARGIN + (x / CELL_MM) * CELL_PX;
const sy = (y) => canvas.height - MARGIN - (y / CELL_MM) * CELL_PX;

function draw() {
  const g = ctx2d;
  g.clearRect(0, 0, canvas.width, canvas.height);

  // floor + cell grid
  g.fillStyle = '#0a111f';
  g.fillRect(0, 0, canvas.width, canvas.height);
  g.strokeStyle = '#132240';
  g.lineWidth = 1;
  for (let i = 0; i <= MAZE_N; i++) {
    g.beginPath();
    g.moveTo(MARGIN + i * CELL_PX, MARGIN); g.lineTo(MARGIN + i * CELL_PX, canvas.height - MARGIN);
    g.moveTo(MARGIN, MARGIN + i * CELL_PX); g.lineTo(canvas.width - MARGIN, MARGIN + i * CELL_PX);
    g.stroke();
  }

  // visited cells
  g.fillStyle = 'rgba(56,189,248,0.13)';
  for (let i = 0; i < CELLS; i++) {
    if (!FF.flood[i].visited) continue;
    const r = rowOf(i), c = colOf(i);
    g.fillRect(MARGIN + c * CELL_PX + 1, canvas.height - MARGIN - (r + 1) * CELL_PX + 1, CELL_PX - 2, CELL_PX - 2);
  }

  // goal block
  g.fillStyle = 'rgba(34,197,94,0.20)';
  g.strokeStyle = '#22c55e';
  g.lineWidth = 1.5;
  for (const gc of GOAL_CELLS) {
    const r = rowOf(gc), c = colOf(gc);
    g.fillRect(MARGIN + c * CELL_PX + 1, canvas.height - MARGIN - (r + 1) * CELL_PX + 1, CELL_PX - 2, CELL_PX - 2);
  }
  g.strokeRect(MARGIN + 7 * CELL_PX, canvas.height - MARGIN - 9 * CELL_PX, 2 * CELL_PX, 2 * CELL_PX);

  // start cell
  g.strokeStyle = 'rgba(245,158,11,0.8)';
  g.strokeRect(MARGIN + 1, canvas.height - MARGIN - CELL_PX + 1, CELL_PX - 2, CELL_PX - 2);

  // walls: true maze (faint) then what the mouse has learned (bright)
  const rects = robot.rects;
  if (showTruth) {
    g.strokeStyle = '#2b3d59';
    g.lineWidth = 3;
    g.beginPath();
    for (const R of rects) {
      const x0 = sx(R[0]), x1 = sx(R[2]);
      const y0 = sy(R[3]), y1 = sy(R[1]);
      g.rect(x0, y0, x1 - x0, y1 - y0);
    }
    g.stroke();
  }
  g.strokeStyle = '#7dd3fc';
  g.lineWidth = 3.5;
  g.lineCap = 'round';
  g.beginPath();
  for (const R of rects) {
    if (!((FF.flood[R[4]].nb >> R[5]) & 1)) continue;
    const x0 = sx(R[0]), x1 = sx(R[2]);
    const y0 = sy(R[3]), y1 = sy(R[1]);
    g.rect(x0, y0, x1 - x0, y1 - y0);
  }
  g.stroke();

  // flood values
  if (showFlood) {
    g.font = '9px ui-monospace, monospace';
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    for (let i = 0; i < CELLS; i++) {
      const v = FF.flood[i].flood;
      if (v === 255) continue;
      const r = rowOf(i), c = colOf(i);
      g.fillStyle = GOAL_CELLS.includes(i) ? '#86efac' : 'rgba(147,197,253,0.75)';
      g.fillText(String(v), MARGIN + c * CELL_PX + CELL_PX / 2, canvas.height - MARGIN - (r + 0.5) * CELL_PX);
    }
  }

  // speed-run path
  if (sup && sup.speedPath.length > 1) {
    g.strokeStyle = 'rgba(34,197,94,0.85)';
    g.lineWidth = 2.5;
    g.beginPath();
    sup.speedPath.forEach((cell, k) => {
      const r = rowOf(cell), c = colOf(cell);
      const X = MARGIN + (c + 0.5) * CELL_PX, Y = canvas.height - MARGIN - (r + 0.5) * CELL_PX;
      if (k === 0) g.moveTo(X, Y); else g.lineTo(X, Y);
    });
    g.stroke();
  }

  // travelled trail
  if (showTrail && trailPts.length > 1) {
    g.strokeStyle = 'rgba(56,189,248,0.35)';
    g.lineWidth = 1.5;
    g.beginPath();
    trailPts.forEach((p, k) => {
      const X = sx(p.x), Y = sy(p.y);
      if (k === 0) g.moveTo(X, Y); else g.lineTo(X, Y);
    });
    g.stroke();
  }

  // ---- the mouse ----
  const X = sx(robot.x), Y = sy(robot.y);
  const rad = (BODY_R_MM / CELL_MM) * CELL_PX;
  const th = robot.th * Math.PI / 180;

  if (showSensors) {
    const beams = [
      { a: th + Math.PI / 2, d: robot.sens.l, thr: WALL_SIDE_MM },
      { a: th, d: robot.sens.m, thr: WALL_FRONT_MM },
      { a: th - Math.PI / 2, d: robot.sens.r, thr: WALL_SIDE_MM }
    ];
    for (const b of beams) {
      const len = (Math.min(b.d === 999 ? VL6180_MAX_MM : b.d, VL6180_MAX_MM) / CELL_MM) * CELL_PX;
      const ex = X + Math.cos(b.a) * len, ey = Y - Math.sin(b.a) * len;
      g.strokeStyle = b.d < b.thr ? 'rgba(244,63,94,0.95)' : 'rgba(125,211,252,0.5)';
      g.lineWidth = 2;
      g.setLineDash([4, 3]);
      g.beginPath(); g.moveTo(X, Y); g.lineTo(ex, ey); g.stroke();
      g.setLineDash([]);
      if (b.d < VL6180_MAX_MM) {
        g.fillStyle = b.d < b.thr ? '#fda4af' : '#93c5fd';
        g.font = '9px ui-monospace, monospace';
        g.textAlign = 'center';
        g.fillText(b.d + '', ex, ey - 5);
      }
    }
  }

  // chassis
  g.save();
  g.translate(X, Y);
  g.rotate(-th);                       // screen y is flipped
  g.fillStyle = '#f59e0b';
  g.strokeStyle = '#7c2d12';
  g.lineWidth = 1.5;
  g.beginPath();
  g.roundRect(-rad * 0.9, -rad * 0.9, rad * 1.8, rad * 1.8, 4);
  g.fill(); g.stroke();
  // nose points +x in world = East-facing at th = 0
  g.fillStyle = '#fff7ed';
  g.beginPath();
  g.moveTo(rad * 0.9, 0); g.lineTo(rad * 0.25, -rad * 0.42); g.lineTo(rad * 0.25, rad * 0.42);
  g.closePath(); g.fill();
  // wheels
  g.fillStyle = '#1f2937';
  g.fillRect(-rad * 0.75, -rad * 1.05, rad * 0.5, rad * 0.18);
  g.fillRect(-rad * 0.75, rad * 0.87, rad * 0.5, rad * 0.18);
  g.restore();
}

/* ------------------------------- clock ------------------------------------ */
let lastFrame = 0;
function frame(now) {
  if (!lastFrame) lastFrame = now;
  let wallMs = now - lastFrame;
  lastFrame = now;
  if (wallMs > 120) wallMs = 120;

  active.clear();

  if (running && sup && sup.phase !== 'DONE') {
    let budget = wallMs * speedMul;
    let guard = 0;
    while (budget > 0 && guard++ < 4000) {
      const dt = Math.min(DT_MS, budget);
      sup.step(dt);
      budget -= dt;
      if (robot.t - lastTrailT >= 40) {
        lastTrailT = robot.t;
        trailPts.push({ x: robot.x, y: robot.y });
        if (trailPts.length > 6000) trailPts.shift();
      }
      if (sup.phase === 'DONE') break;
    }
  }

  draw();
  paintPipeline();
  paintTelemetry(now);
  requestAnimationFrame(frame);
}

/* ------------------------------- controls --------------------------------- */
let showFlood = true, showTruth = true, showSensors = true, showTrail = true;

document.getElementById('btnPlay').onclick = (e) => {
  running = !running;
  e.target.textContent = running ? '❚❚ Pause' : '▶ Run';
  if (running && sup && sup.phase === 'DONE') {
    // finished: pressing run restarts with the same maze
    boot(false);
  }
};
document.getElementById('btnStep').onclick = () => {
  running = false;
  document.getElementById('btnPlay').textContent = '▶ Run';
  if (!sup || sup.phase === 'DONE') return;
  active.clear();
  let a0 = sup.action;
  if (!a0) { let g = 0; while (!sup.action && g++ < 60) sup.step(DT_MS); a0 = sup.action; }
  if (!a0) return;
  let n = 0;
  while (sup.action === a0 && n++ < 40000) {
    sup.step(DT_MS);
    if (robot.t - lastTrailT >= 40) { lastTrailT = robot.t; trailPts.push({ x: robot.x, y: robot.y }); }
  }
};
document.getElementById('btnReset').onclick = () => { boot(false); document.getElementById('log').innerHTML = ''; };
document.getElementById('btnNewMaze').onclick = () => { boot(true); document.getElementById('log').innerHTML = ''; };
document.getElementById('btnClearLog').onclick = () => { document.getElementById('log').innerHTML = ''; };
document.getElementById('speedRange').oninput = (e) => {
  speedMul = +e.target.value;
  document.getElementById('speedLabel').textContent = speedMul + '×';
};
document.getElementById('showFlood').onchange = (e) => showFlood = e.target.checked;
document.getElementById('showTruth').onchange = (e) => showTruth = e.target.checked;
document.getElementById('showSensors').onchange = (e) => showSensors = e.target.checked;
document.getElementById('showTrail').onchange = (e) => showTrail = e.target.checked;
document.getElementById('strategySel').onchange = (e) => {
  setStrategy(e.target.value);
  addLog('MAIN', `strategy selected: ${STRATEGY_NAME[STRATEGY]} — restarting the run`);
  boot(false);
};
document.getElementById('gyroAlive').onchange = (e) => {
  GYRO_ALIVE = e.target.checked;
  robot.gyroAlive = GYRO_ALIVE;
  addLog('MPU', GYRO_ALIVE
    ? 'gyroAlive = true — runStraight() uses the P/I heading loop'
    : 'gyroAlive = false — frozen gz yaw: encoder equaliser + wall trim only (as on the real car)');
};

if (!CanvasRenderingContext2D.prototype.roundRect) {
  CanvasRenderingContext2D.prototype.roundRect = function (x, y, w, h, r) {
    this.beginPath(); this.moveTo(x + r, y); this.arcTo(x + w, y, x + w, y + h, r);
    this.arcTo(x + w, y + h, x, y + h, r); this.arcTo(x, y + h, x, y, r);
    this.arcTo(x, y, x + w, y, r); this.closePath(); return this;
  };
}

boot(false);
requestAnimationFrame(frame);
