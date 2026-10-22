/* ============================================================================
   solver.js — floodfill module (map + BFS solver) and the main.ino supervisor
   ----------------------------------------------------------------------------
   Mirrors the floodfill module cell for cell:
     floodArray[{flood, neighbours, visited}]  bit0=N bit1=E bit2=S bit3=W
     setFloodToTargets()            BFS flood
     findNearestUnvisitedCell()     explorer target selection
     ff_readWall()                  3-sample majority wall vote
     ff_goToTargetCell()            turn (motion module) + ff_moveForwardCells()
     computeWallCenteringTrim()     side-wall corridor centring
   and runFloodFillSequence(): explore -> return to start -> speed run to centre.
   ========================================================================== */

const CELLS = MAZE_N * MAZE_N;
const GOAL_CELLS = [idx(7, 7), idx(7, 8), idx(8, 7), idx(8, 8)];

const FF = {
  flood: [], startCell: 0, currentCell: 0, targetCell: 0,
  startDir: 0, currentDir: 0, leftDir: 0, rightDir: 0,
  cachedLv: 999, cachedRv: 999, readLeftNext: true, wallCenterTrim: 0, wallTrimSign: 1,
  frontMm: 999,

  begin() {
    this.flood = [];
    for (let i = 0; i < CELLS; i++) this.flood.push({ flood: 255, nb: 0, visited: 0 });
    this.startCell = 0; this.currentCell = 0; this.startDir = DIR_N;
    this.currentDir = DIR_N;
    this.leftDir = (this.currentDir + 3) % 4;
    this.rightDir = (this.currentDir + 1) % 4;
    this.cachedLv = this.cachedRv = 999;
    this.wallCenterTrim = 0;
    this.flood[this.currentCell].visited = 1;
    this.setFloodToTargets(GOAL_CELLS);
  },

  // ---- BFS flood (setFloodToTargets) -------------------------------------
  setFloodToTargets(targets) {
    for (let i = 0; i < CELLS; i++) this.flood[i].flood = 255;
    const q = [];
    for (const t of targets) { if (t < CELLS) { this.flood[t].flood = 0; q.push(t); } }
    let h = 0;
    while (h < q.length) {
      const loc = q[h++];
      const cur = this.flood[loc].flood;
      for (let d = 0; d < 4; d++) {
        if (!this.neighbourValid(loc, d)) continue;
        if (this.wallExists(loc, d)) continue;
        const n = loc + DIR_DR[d] * MAZE_N + DIR_DC[d];
        if (this.flood[n].flood > cur + 1) { this.flood[n].flood = cur + 1; q.push(n); }
      }
    }
  },

  neighbourValid(loc, d) {
    const r = rowOf(loc), c = colOf(loc);
    if (d === DIR_N) return r < MAZE_N - 1;
    if (d === DIR_E) return c < MAZE_N - 1;
    if (d === DIR_S) return r > 0;
    return c > 0;                                        // W
  },
  wallExists(loc, d) { return (this.flood[loc].nb >> d) & 1; },
  markWall(loc, d) {
    this.flood[loc].nb |= (1 << d);
    if (this.neighbourValid(loc, d)) {
      const n = loc + DIR_DR[d] * MAZE_N + DIR_DC[d];
      this.flood[n].nb |= (1 << ((d + 2) % 4));
    }
  },
  allVisited() {
    for (let i = 0; i < CELLS; i++) if (!this.flood[i].visited && this.flood[i].nb !== 0x0F) return false;
    return true;
  },
  visitedCount() { let n = 0; for (let i = 0; i < CELLS; i++) if (this.flood[i].visited) n++; return n; },

  // ---- explorer: nearest cell the map still needs ------------------------
  findNearestUnvisited() {
    const dist = new Int16Array(CELLS).fill(-1);
    const q = [this.currentCell];
    dist[this.currentCell] = 0;
    let h = 0;
    while (h < q.length) {
      const loc = q[h++];
      for (let d = 0; d < 4; d++) {
        if (!this.neighbourValid(loc, d)) continue;
        if (this.wallExists(loc, d)) continue;
        const n = loc + DIR_DR[d] * MAZE_N + DIR_DC[d];
        if (dist[n] !== -1) continue;
        dist[n] = dist[loc] + 1;
        if (!this.flood[n].visited && this.flood[n].nb !== 0x0F) return n;
        q.push(n);
      }
    }
    return -1;
  },

  // ---- ff_updateTargetCell(): step down the flood gradient ---------------
  updateTargetCell() {
    const scoreOf = [0, 1, 2, 1];                        // prefer going straight
    let best = this.currentCell, bestD = 255, bestScore = 3;
    for (let d = 0; d < 4; d++) {
      if (this.wallExists(this.currentCell, d)) continue;
      if (!this.neighbourValid(this.currentCell, d)) continue;   // maze border
      const n = this.currentCell + DIR_DR[d] * MAZE_N + DIR_DC[d];
      const nd = this.flood[n].flood;
      const score = scoreOf[(d + 4 - this.currentDir) % 4];
      if (nd < bestD || (nd === bestD && score < bestScore)) { bestD = nd; bestScore = score; best = n; }
    }
    this.targetCell = best;
    return best;
  },

  absDirTo(target) {
    const diff = target - this.currentCell;
    if (diff === MAZE_N) return DIR_N;
    if (diff === 1) return DIR_E;
    if (diff === -MAZE_N) return DIR_S;
    if (diff === -1) return DIR_W;
    return this.currentDir;
  },
  setDir(d) { this.currentDir = d; this.leftDir = (d + 3) % 4; this.rightDir = (d + 1) % 4; },

  // ---- computeWallCenteringTrim(): lateral corridor centring -------------
  computeWallCenteringTrim(robot) {
    if (robot.justTurned) { this.cachedLv = this.cachedRv = 999; this.readLeftNext = true; }
    if (this.readLeftNext) {
      const v = VL6180.cast(robot, robot.th + 90);
      this.cachedLv = (v >= 5) ? v : 999;
      this.readLeftNext = false;
    } else {
      const v = VL6180.cast(robot, robot.th - 90);
      this.cachedRv = (v >= 5) ? v : 999;
      this.readLeftNext = true;
    }
    const lv = this.cachedLv, rv = this.cachedRv;
    const lOk = lv >= 5 && lv <= CENTER_MAX_VALID_MM;
    const rOk = rv >= 5 && rv <= CENTER_MAX_VALID_MM;
    let target = 0;
    if (lOk && rOk) {
      const err = rv - lv;                               // steer toward the larger gap
      if (Math.abs(err) > CENTER_DEADBAND_MM) target = err * CENTER_TRIM_GAIN;
    } else if (lOk) {
      target = (CENTER_ONE_WALL_GAP_MM - lv) * CENTER_TRIM_GAIN;
    } else if (rOk) {
      target = (rv - CENTER_ONE_WALL_GAP_MM) * CENTER_TRIM_GAIN;
    }
    target = clamp(target * this.wallTrimSign, -CENTER_MAX_TRIM, CENTER_MAX_TRIM);
    this.wallCenterTrim += (target - this.wallCenterTrim) * 0.35;
    if (Math.abs(this.wallCenterTrim) < 0.5) this.wallCenterTrim = 0;
    return this.wallCenterTrim;
  }
};

/* ======================== supervisor (main.ino) ============================ */
const DT_MS = 5;            // firmware loop period: runStraight() + delay(5)

const MODULE_SEQ = {
  EXPLORE: ['scan', 'plan', 'turn', 'drive', 'align'],
  RETURN: ['plan', 'turn', 'drive', 'align'],
  SPEED: ['plan', 'turn', 'drive', 'align'],
  DONE: []
};

class Supervisor {
  constructor(robot, ctx) {
    this.r = robot;
    this.ctx = ctx;                     // {log(mod,msg), ...}
    this.reset();
  }

  reset() {
    FF.begin();
    this.phase = 'EXPLORE';
    this.action = null;
    this.sub = undefined;
    this.moves = 0; this.turns = 0; this.aborts = 0; this.scans = 0;
    this.consecFail = 0; this.consecOpen = 0; this.anyWallEver = false;
    this.speedPath = [];
    this.target = -1; this.rel = 0; this.absDir = DIR_N;
    this.phaseStart = 0;
    this.ctx.log('MAIN', 'boot: I2C + VL6180 + MPU6050 + motors + encoders OK — flood-fill run armed');
    this.ctx.log('FF', `geometry: wheel ${WHEEL_DIAMETER_MM} mm, cell ${CELL_MM} mm -> ${STEPS_PER_CELL} steps/cell`);
  }

  /* ------------------------- action factories ---------------------------- */

  actScan() {
    const r = this.r, sup = this;
    MotorControl.brake(r);
    return {
      type: 'scan', r, t0: r.t, i: 0, votes: [0, 0, 0], last: [999, 999, 999],
      update() {
        const a = this;
        while (a.i < 3 && r.t - a.t0 >= a.i * 15) {
          const s = VL6180.readall(r);
          a.last = [s.l, s.m, s.r];
          if (s.l < WALL_SIDE_MM) a.votes[0]++;
          if (s.m < WALL_FRONT_MM) a.votes[1]++;
          if (s.r < WALL_SIDE_MM) a.votes[2]++;
          a.i++;
          r.sens = s;
          ctxActive('VL6180', `readallSensor()  L=${s.l}  F=${s.m}  R=${s.r} mm`);
        }
        return a.i >= 3;
      },
      done() {
        const left = this.votes[0] >= 2, front = this.votes[1] >= 2, right = this.votes[2] >= 2;
        if (left) FF.markWall(FF.currentCell, FF.leftDir);
        if (front) FF.markWall(FF.currentCell, FF.currentDir);
        if (right) FF.markWall(FF.currentCell, FF.rightDir);
        if (left || front || right) { sup.anyWallEver = true; sup.consecOpen = 0; }
        else sup.consecOpen++;
        sup.scans++;
        r.sens = { l: this.last[0], m: this.last[1], r: this.last[2] };
        sup.ctx.log('FF', `cell ${FF.currentCell} (${rowOf(FF.currentCell)},${colOf(FF.currentCell)}) ` +
          `L=${this.last[0]} F=${this.last[1]} R=${this.last[2]} -> ` +
          `${left ? 'W' : 'O'}${front ? 'W' : 'O'}${right ? 'W' : 'O'}`);
        ctxActive('FF', `ff_readWall(): map updated for cell ${FF.currentCell}`);
      }
    };
  }

  actPlan(mode) {
    const r = this.r, sup = this;
    MotorControl.brake(r);
    return {
      type: 'plan', r, t0: r.t,
      update() {
        ctxActive('FF', mode === 'explore'
          ? 'findNearestUnvisitedCell() + setFloodToTargets() + BFS'
          : 'setFloodToTargets(goal) + BFS');
        return r.t - this.t0 >= 20;
      },
      done() {
        if (mode === 'explore') {
          const nxt = FF.findNearestUnvisited();
          if (nxt < 0) { sup.target = -1; return; }
          FF.setFloodToTargets([nxt, nxt, nxt, nxt]);
        }
        sup.target = FF.updateTargetCell();
        if (sup.target === FF.currentCell) { sup.target = -1; return; }
        sup.absDir = FF.absDirTo(sup.target);
        sup.rel = (sup.absDir + 4 - FF.currentDir) % 4;
        sup.ctx.log('FF', `flood ${FF.flood[FF.currentCell].flood} -> target ${sup.target} ` +
          `(${DIR_NAME[sup.absDir]}, ${['straight', 'right', 'back', 'left'][sup.rel]})`);
      }
    };
  }

  actTurn(rel, absDir) {
    const r = this.r, sup = this;
    return {
      type: 'turn', r, started: false,
      update() {
        if (!this.started) {
          this.started = true;
          if (rel === 1) Motion.turnRight90(r);
          else if (rel === 2) Motion.turn180(r);
          else Motion.turnLeft90(r);
          sup.turns++;
          sup.ctx.log('MOTION', `turn${rel === 1 ? 'Right90' : rel === 2 ? '180PivotLeft' : 'Left90'}() ` +
            `— gyro-closed pivot at PWM ${TURN_SPEED}`);
        }
        const turning = Motion.step(r);
        const T = r.turn;
        ctxActive('MOTION', `pivot ${(T ? (r.gyroHeading - T.head0).toFixed(1) : 0)}° / ${T ? T.targetDeg : 0}° ` +
          `| PWM ${r.pwmL}/${r.pwmR}`);
        ctxActive('MPU', `headingTrueDeg ${r.gyroHeading.toFixed(1)}° (gyroAlive=${r.gyroAlive})`);
        if (!turning) {
          FF.setDir(absDir);
          sup.ctx.log('MPU', `turn closed on heading ${r.gyroHeading.toFixed(1)}° (${T ? T.done : '?'})`);
        }
        return !turning;
      }
    };
  }

  /* Recovery after an aborted segment (simulator assist). The firmware leaves
     the car where the emergency brake caught it — half-way into the corridor —
     and the next ff_readWall() then reads walls from a position that belongs to
     no cell, which marks phantom walls into the map. Here the segment is unwound
     on the encoders so the car is back on the cell centre before it re-scans. */
  actRecover(steps) {
    const r = this.r, sup = this;
    return {
      type: 'recover', r, started: false, t0: 0,
      update() {
        if (!this.started) {
          this.started = true; this.t0 = r.t;
          sup.ctx.log('FF', `recover: unwinding ${steps} encoder steps back to the cell centre`);
        }
        const c = Encoder.counts(r);
        const done = Math.min(c.l, c.r);
        MotorControl.run(r, -110, -110);
        ctxActive('MOTOR', `motorRun(-110, -110) — recovery, ${done} steps left`);
        if (done <= 0 || r.t - this.t0 > 3000) { MotorControl.brake(r); return true; }
        return false;
      }
    };
  }

  /* ff_updateYawSetpoint(): the firmware declares this alignment helper but the
     maze path never calls it. It is enabled here so a 256-cell exploration stays
     on the cell grid despite the frozen gz-yaw channel. */
  actAlign() {
    const r = this.r, sup = this;
    const want = DIR_ANGLE[FF.currentDir];
    const err = angleDiff(want, r.th);
    return {
      type: 'align', r, started: false, err,
      update() {
        if (!this.started) {
          this.started = true;
          if (Math.abs(this.err) < 2.5) return true;
          const fb = Math.abs(this.err) * (TURN_90_LEFT_MS / 90);
          const pwm = Math.round(clamp(Math.abs(this.err) * 4, 60, 120));
          Motion.startPivot(r, this.err > 0 ? -pwm : pwm, this.err > 0 ? pwm : -pwm, this.err, fb, 'ALIGN');
          sup.ctx.log('FF', `ff_updateYawSetpoint(): heading off by ${this.err.toFixed(1)}° — re-aligning`);
        }
        const turning = Motion.step(r);
        ctxActive('MOTION', `align: ${angleDiff(want, r.th).toFixed(1)}° to go | PWM ${r.pwmL}/${r.pwmR}`);
        return !turning;
      }
    };
  }

  /* ff_moveForwardCells(1): encoder-driven segment with front-wall abort and
     side-wall centring, exactly as in the floodfill module */
  actDrive(destCell) {
    const r = this.r, sup = this;
    Encoder.reset(r);
    // ff_moveForwardCells() normally drives a blind STEPS_PER_CELL burst, so any
    // positioning error carries over into the next cell and grows without bound.
    // A real mouse keeps its pose bounded by fusing the encoders with the wall
    // sensors; this simulation assumes that fused estimate (ideal odometry) and
    // drives the measured remaining distance to the next cell centre. The stop
    // itself is still purely encoder-driven, so the Encoder module stays in the loop.
    const cx = colOf(destCell) * CELL_MM + CELL_MM / 2;
    const cy = rowOf(destCell) * CELL_MM + CELL_MM / 2;
    const along = (cx - r.x) * Math.cos(r.th * Math.PI / 180) + (cy - r.y) * Math.sin(r.th * Math.PI / 180);
    const targetSteps = Math.round(clamp(along, 0.45 * CELL_MM, 1.6 * CELL_MM) * STEPS_PER_MM);
    return {
      type: 'drive', r, target: targetSteps, start: r.t, ok: false, reason: '',
      frontStreak: 0, prevM: 999, lastFront: 0, lastSide: 0,
      update(dt) {
        const c = Encoder.counts(r);
        const done = Math.min(c.l, c.r);
        if (done >= this.target) {
          MotorControl.brake(r); Distance.setWallTrim(r, 0);
          this.ok = true; return true;
        }
        if (r.t - this.start > 6000) {
          MotorControl.brake(r); Distance.setWallTrim(r, 0);
          this.ok = false; this.reason = 'TIMEOUT';
          sup.ctx.log('WARN', 'ff_moveForwardCells(): segment TIMEOUT — move aborted');
          return true;
        }
        // Wheels stalled against a wall: a real car never reaches its encoder
        // target either, so treat it as an unexpected wall.
        if (r.stalled > 40) {
          MotorControl.brake(r); Distance.setWallTrim(r, 0);
          this.ok = false; this.reason = 'BLOCKED'; this.abortSteps = done;
          FF.markWall(FF.currentCell, FF.currentDir);
          sup.ctx.log('WARN', `wheels stalled against a wall at ${done}/${this.target} steps — wall marked`);
          return true;
        }
        // emergency front-wall check: armed after 1/4 cell, disarmed for the last
        // FRONT_POLL_STOP_MM before the encoder takes over
        if (this.target - done > STEPS_PER_MM * FRONT_POLL_STOP_MM && done >= STEPS_PER_CELL / 4 && r.t - this.lastFront >= 15) {
          this.lastFront = r.t;
          const fd = VL6180.cast(r, r.th);
          FF.frontMm = fd;
          const hugging = (FF.cachedLv < 45) || (FF.cachedRv < 45);
          const open = (FF.cachedLv >= CENTER_MAX_VALID_MM) && (FF.cachedRv >= CENTER_MAX_VALID_MM);
          const suspect = hugging || open;
          const near = fd < (hugging ? 80 : 130);
          const jump = near && this.prevM > 200;
          this.prevM = fd;
          this.frontStreak = near ? this.frontStreak + 1 : 0;
          if (this.frontStreak >= (jump ? 6 : 3)) {
            MotorControl.brake(r); Distance.setWallTrim(r, 0);
            this.ok = false; this.reason = 'FRONT_WALL'; this.abortSteps = done;
            if (!suspect) FF.markWall(FF.currentCell, FF.currentDir);
            sup.ctx.log('WARN', `wall ${fd} mm ahead at ${done}/${this.target} steps — braking` +
              (suspect ? ' (suspect: side-on/open — map NOT updated)' : ' — wall marked'));
            return true;
          }
        }
        if (r.t - this.lastSide >= 25) {
          this.lastSide = r.t;
          Distance.setWallTrim(r, FF.computeWallCenteringTrim(r));
        }
        Distance.runStraight(r, dt);
        const rpm = Encoder.rpm(r);
        ctxActive('DISTANCE', `runStraight(): err ${r.dbg.yawError.toFixed(1)}° corr ${r.dbg.correction.toFixed(1)} ` +
          `enc ${r.dbg.encDiff.toFixed(1)} trim ${r.dbg.trim.toFixed(1)} -> PWM ${r.pwmL.toFixed(0)}/${r.pwmR.toFixed(0)}`);
        ctxActive('MOTOR', `motorRun(${r.pwmL.toFixed(0)}, ${r.pwmR.toFixed(0)})`);
        ctxActive('ENC', `steps ${c.l}/${c.r} (${done}/${this.target})  ${rpm.l.toFixed(0)}/${rpm.r.toFixed(0)} RPM`);
        return false;
      }
    };
  }

  /* --------------------------- phase control ----------------------------- */

  nextSub() {
    const seq = MODULE_SEQ[this.phase] || [];
    if (!seq.length) return null;
    const i = seq.indexOf(this.sub);
    return seq[(i + 1) % seq.length];      // i = -1 -> first step
  }

  nextAction(sub) {
    if (!sub) return null;
    if (this.phase === 'EXPLORE' && sub === 'scan') {
      if (FF.allVisited()) return this.finishExplore();
      return this.actScan();
    }
    if (sub === 'plan') {
      if (this.phase === 'RETURN' && FF.currentCell === FF.startCell) return this.finishPhase();
      if (this.phase === 'SPEED' && GOAL_CELLS.includes(FF.currentCell)) return this.finishPhase();
      return this.actPlan(this.phase === 'EXPLORE' ? 'explore' : 'goal');
    }
    if (sub === 'turn') {
      if (this.target < 0 || this.rel === 0) return null;
      return this.actTurn(this.rel, this.absDir);
    }
    if (sub === 'drive') return this.actDrive(this.target);
    if (sub === 'align') return this.actAlign();
    return null;
  }

  finishExplore() {
    this.ctx.log('FF', `explore complete: ${FF.visitedCount()} cells mapped — FF_saveMap() to EEPROM`);
    this.phase = 'RETURN';
    this.sub = undefined;
    this.target = -1;
    FF.setFloodToTargets([FF.startCell, FF.startCell, FF.startCell, FF.startCell]);
    this.ctx.log('MAIN', 'phase 2/3: FF_returnToStart() — driving back to the start cell');
    this.phaseStart = this.r.t;
    return null;
  }

  finishPhase() {
    if (this.phase === 'RETURN') {
      this.ctx.log('MAIN', 'back at the start cell — phase 3/3: FF_runShortestPathToGoal()');
      this.phase = 'SPEED';
      this.sub = undefined;
      this.target = -1;
      this.phaseStart = this.r.t;
      FF.setFloodToTargets(GOAL_CELLS);
      return null;
    }
    this.ctx.log('MAIN', `GOAL REACHED in cell ${FF.currentCell} — run finished`);
    this.phase = 'DONE';
    MotorControl.brake(this.r);
    return null;
  }

  /* ------------------------- one control tick ---------------------------- */
  step(dtMs) {
    const r = this.r;
    if (this.phase === 'DONE') { MotorControl.brake(r); return; }

    if (!this.action) {
      let guard = 0;
      while (!this.action && guard++ < 8) {
        this.sub = this.nextSub();
        const a = this.nextAction(this.sub);
        if (a) { this.action = a; this.actionStart = r.t; }
      }
      if (!this.action) return;
    }

    const a = this.action;
    const done = a.update(dtMs);
    r.tick(dtMs);                       // plant integration for this control period
    if (done) {
      if (a.done) a.done();
      this.onActionFinished(a);
      this.action = null;
    }
  }

  onActionFinished(a) {
    if (a.type === 'scan' && !this.anyWallEver && this.consecOpen >= 6) {
      this.ctx.log('WARN', '6 cells with no wall on any sensor — sensorPreflight() would abort the run');
      this.phase = 'DONE';
      return;
    }
    if (a.type === 'plan' && this.target < 0) {
      if (this.phase === 'EXPLORE') this.finishExplore();
      else {
        this.ctx.log('WARN', 'no route to the destination — stopping');
        this.phase = 'DONE';
      }
      this.sub = undefined;
      return;
    }
    if (a.type === 'drive') {
      if (a.ok) {
        FF.currentCell = this.target;
        FF.flood[FF.currentCell].visited = 1;
        this.moves++;
        this.consecFail = 0;
        if (this.phase === 'SPEED') this.speedPath.push(FF.currentCell);
        this.ctx.log('DRIVE', `segment OK — arrived in cell ${FF.currentCell} ` +
          `(${rowOf(FF.currentCell)},${colOf(FF.currentCell)})`);
      } else {
        this.aborts++;
        this.consecFail++;
        this.ctx.log('FF', `move ${a.reason} — re-planning from cell ${FF.currentCell}`);
        if ((a.reason === 'FRONT_WALL' || a.reason === 'BLOCKED') && (a.abortSteps || 0) > 40) {
          this.action = this.actRecover(a.abortSteps);   // unwind to the cell centre
          return;
        }
        if (this.consecFail >= 4) {
          this.ctx.log('WARN', '4 consecutive blocked moves — map inconsistent, stopping');
          this.phase = 'DONE';
        }
      }
    }
    // Phase watchdog. The firmware allows 90 s for the explore phase, which fits
    // its 8x8 maze; sweeping a 16x16 grid needs ~5 minutes of simulated time, so
    // the budgets are scaled with the cell count.
    const budget = this.phase === 'EXPLORE' ? 600000 : 300000;
    if (this.r.t - this.phaseStart > budget) {
      this.ctx.log('WARN', `phase watchdog (${budget / 1000} s) — stopping`);
      this.phase = 'DONE';
    }
  }
}

/* module-activity reporter, installed by app.js */
let ctxActive = function () { };
function setActiveReporter(fn) { ctxActive = fn; }
