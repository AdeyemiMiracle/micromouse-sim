/* ============================================================================
   modules.js — JavaScript twins of the Grizzly firmware modules
   ----------------------------------------------------------------------------
   VL6180 module   : time-of-flight ranging (readsingleSensor / readallSensor)
   mpu6050 module  : readangle() + mounting-independent gravity-axis heading
   Encoder module  : quadrature odometry, wheel RPM
   MotorControl    : TB6612FNG H-bridge, signed PWM -> wheels
   distance module : runStraight()  (soft start + P/I yaw + encoder match + trim)
   motion module   : turnLeft90 / turnRight90 / turn180PivotLeft
   All constants below are the values compiled into the firmware.
   ========================================================================== */

const WHEEL_DIAMETER_MM = 23.0;                        // floodfill module
const WHEEL_CIRC_MM = Math.PI * WHEEL_DIAMETER_MM;
const PULSES_PER_WHEEL_REV = 1320;                     // Encoder module ENCODER_PPR
const STEPS_PER_CELL = Math.round(CELL_MM / WHEEL_CIRC_MM * PULSES_PER_WHEEL_REV);
const STEPS_PER_MM = STEPS_PER_CELL / CELL_MM;
const WHEELBASE_MM = 90.0;                             // chassis track width (sim only)
const BODY_R_MM = 42.0;                                // chassis radius (sim only)
const PWM_STALL = 35;                                  // PWM below which the motors do not turn
const MM_PER_S_PER_PWM = 2.0;                          // sim plant: 150 PWM -> 300 mm/s
const LEFT_MOTOR_TRIM = 0.955;                         // motor asymmetry the P/I loop must cancel

// distance module straight-line controller constants
const TARGET_SPEED = 150;
const YAW_TOLERANCE = 0.5;
const YAW_CORRECTION_GAIN = 2.5;
const ENCODER_CORRECTION_GAIN = 1.0;
const MAX_ENCODER_CORRECTION = 30.0;
const TURN_RECAPTURE_ANGLE = 45.0;
const MAX_YAW_CORRECTION = 50.0;
const YAW_INTEGRAL_GAIN = 0.08;
const MAX_YAW_INTEGRAL = 500.0;
const MIN_POWER = 60, MAX_POWER = 200;
const RAMP = 5.0;                                      // PWM units per loop call (soft start)

// motion module turn constants
const TURN_90_LEFT_MS = 1070, TURN_90_RIGHT_MS = 1285, TURN_180_MS = 1000;
const TURN_TIMEOUT_MS = 3000;
const TURN_HEADING_ALIVE_DEG = 2.0;
const TURN_SLOW_START_DEG = 45.0;
const TURN_MIN_PIVOT_PWM = 65.0;
// The firmware stops a pivot within 5 deg of the target heading; on a 180 mm
// cell that alone leaves ~15 mm of lateral error per cell, which a 16x16 run
// cannot absorb. Tightened to 2 deg so the yaw loop lands on the heading.
const TURN_STOP_TOL_DEG = 2.0;
const TURN_SPEED = 150;

// Wall thresholds (VL6180 module). The front value is the firmware's. The side
// value is raised from 90 mm to 120 mm for the 16x16 geometry: a centred car
// reads 84 mm to a side wall, so 90 mm only tolerates a 6 mm offset, while a
// single-wheel pivot leaves the chassis up to ~45 mm off the corridor centre.
// An open side is never ambiguous — the beam then runs on to the wall two cells
// away (264 mm) and returns "no echo" (999) — so 120 mm cannot false-trigger.
const WALL_FRONT_MM = 130;
const WALL_SIDE_MM = 120;

// Front-abort arming (floodfill module). FRONT_ABORT_ARM_STEPS is the firmware's
// quarter-cell delay. POLL_STOP_REMAIN is widened from the firmware's 60 mm to
// 100 mm: the firmware's margin assumes the car starts every segment exactly on
// the cell centre, while dead-reckoned travel leaves it up to ~25 mm off. With
// 100 mm the poll always ends while the far wall of the destination cell is
// still >160 mm away, so only a genuine unexpected wall (which appears at the
// current cell's boundary, ~90 mm ahead at the start) can trip the abort.
const FRONT_POLL_STOP_MM = 100;

// floodfill module wall-centring trim. The lone-wall standoff is rescaled to the
// 180 mm cell (half corridor) and the clamp is reduced from the firmware's
// 40 PWM: at this plant's 300 mm/s a 40 PWM split slews the chassis ~100 deg/s,
// which over-rotates a one-cell segment. 14 PWM still arrests a 30 mm offset
// inside a single cell without leaving the car crabbing into the next one.
const CENTER_TRIM_GAIN = 0.25;
const CENTER_MAX_TRIM = 8.0;
const CENTER_DEADBAND_MM = 3.0;
const CENTER_ONE_WALL_GAP_MM = 84.0;                   // half corridor of a 180/12 mm cell
const CENTER_MAX_VALID_MM = 190.0;

// Set false to reproduce the physical Grizzly, whose MPU is mounted so the gz
// yaw channel never moves: the motion module then leaves gyroAlive = false and
// runStraight() drives on the encoder equaliser + side-wall trim alone.
let GYRO_ALIVE = true;

const clamp = (v, a, b) => v < a ? a : (v > b ? b : v);

function angleDiff(target, current) {                  // motion module
  let d = target - current;
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}
function norm360(a) { a %= 360; return a < 0 ? a + 360 : a; }

/* ========================= VL6180 module ================================== */
const VL6180 = {
  cast(robot, angleDeg) {                              // readsingleSensor()
    if (!robot.rects) return 999;
    return castSensor(robot.x, robot.y, angleDeg, robot.rects, 1.2);
  },
  readall(robot) {                                     // readallSensor(&l,&m,&r)
    const h = robot.th;
    return {
      l: VL6180.cast(robot, h + 90),                   // lox1  (left)
      m: VL6180.cast(robot, h),                        // lox2  (front)
      r: VL6180.cast(robot, h - 90)                    // lox3  (right)
    };
  },
  isFrontWall(v) { return v < WALL_FRONT_MM; },        // isFrontWall()
  isSideWall(v) { return v < WALL_SIDE_MM; }           // isLeftWall() / isRightWall()
};

/* ========================= mpu6050 module ================================= */
const MPU6050 = {
  readangle(robot) {
    // headingTrueDeg: gyro projected on the gravity axis, unwrapped. The gz-only
    // "yaw" channel of this car stays frozen during rotation (see motion module
    // comments), which is why gyroAlive defaults to false.
    return {
      roll: 0.0,
      pitch: 0.0,
      yaw: robot.yawFrozen,
      headingTrueDeg: robot.gyroHeading + (Math.random() - 0.5) * 0.06
    };
  },
  snapshotAxis(robot) { robot.gyroHeading = norm360(robot.th); }   // snapshotRotationAxis()
};

/* ========================= Encoder module ================================= */
const Encoder = {
  counts(robot) {                                      // L_EN_CNT / R_EN_CNT
    return {
      l: Math.round(robot.wheelDistL / WHEEL_CIRC_MM * PULSES_PER_WHEEL_REV),
      r: Math.round(robot.wheelDistR / WHEEL_CIRC_MM * PULSES_PER_WHEEL_REV)
    };
  },
  reset(robot) { robot.wheelDistL = 0; robot.wheelDistR = 0; },
  speedDiffRPM(robot) {                                // Encoder_SpeedDiffRPM()
    const l = robot.vL / WHEEL_CIRC_MM * 60.0;
    const r = robot.vR / WHEEL_CIRC_MM * 60.0;
    return l - r;                                      // + = left wheel faster
  },
  rpm(robot) {
    return { l: robot.vL / WHEEL_CIRC_MM * 60.0, r: robot.vR / WHEEL_CIRC_MM * 60.0 };
  }
};

/* ======================= MotorControl module ============================== */
const MotorControl = {
  run(robot, left, right) {                            // motorRun(l, r)
    robot.pwmL = clamp(left, -255, 255);
    robot.pwmR = clamp(right, -255, 255);
  },
  brake(robot) { robot.pwmL = 0; robot.pwmR = 0; robot.braking = true; }
};

/* ========================= distance module ================================ */
const Distance = {
  setWallTrim(robot, pwm) { robot.wallTrim = pwm; },   // setStraightWallTrim()
  // runStraight(): one control step. dtMs is the firmware's ~5 ms loop period.
  runStraight(robot, dtMs) {
    const c = robot.ctrl;
    const imu = MPU6050.readangle(robot);
    const yaw = imu.headingTrueDeg;
    robot.yaw = yaw;

    // reference heading capture / re-capture after a turn
    if (!c.refValid || Math.abs(angleDiff(c.refYaw, norm360(robot.th))) > TURN_RECAPTURE_ANGLE) {
      c.refYaw = norm360(robot.th);
      c.refValid = true;
      if (robot.justTurned) {
        robot.justTurned = false;
        c.yawIntegral = 0;
        robot.smoothL = 0; robot.smoothR = 0;
        robot.wallTrim = 0;
      }
    }

    // steering-authority gating: one driver at a time (yaw / encoder / wall trim)
    const encDiff = Encoder.speedDiffRPM(robot);
    let encCorrection = 0;
    if (c.encArmed && Math.abs(robot.wallTrim) > 3.0) {
      c.encArmed = false;
      c.yawIntegral = 0;
      c.refYaw = norm360(robot.th);
    } else if (!c.encArmed && Math.abs(robot.wallTrim) < 1.0) {
      c.encArmed = true;
      c.refYaw = norm360(robot.th);
    }

    let yawError = 0;
    if (robot.gyroAlive && c.encArmed) {
      yawError = angleDiff(yaw, c.refYaw);
    } else {
      c.yawIntegral = 0;
    }

    if (Math.abs(yawError) > YAW_TOLERANCE) {
      c.yawIntegral = clamp(c.yawIntegral + yawError, -MAX_YAW_INTEGRAL, MAX_YAW_INTEGRAL);
    } else {
      c.yawIntegral *= 0.995;
    }

    let correction = clamp(yawError * YAW_CORRECTION_GAIN + c.yawIntegral * YAW_INTEGRAL_GAIN,
      -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION);
    if (c.encArmed) {
      encCorrection = clamp(encDiff * ENCODER_CORRECTION_GAIN, -MAX_ENCODER_CORRECTION, MAX_ENCODER_CORRECTION);
    }

    const lp = clamp(TARGET_SPEED - correction - encCorrection + robot.wallTrim, MIN_POWER, MAX_POWER);
    const rp = clamp(TARGET_SPEED + correction + encCorrection - robot.wallTrim, MIN_POWER, MAX_POWER);

    // soft-start ramp
    const scale = dtMs / 5.0;
    robot.smoothL = clamp(lp, robot.smoothL - RAMP * scale, robot.smoothL + RAMP * scale);
    robot.smoothR = clamp(rp, robot.smoothR - RAMP * scale, robot.smoothR + RAMP * scale);

    MotorControl.run(robot, robot.smoothL, robot.smoothR);

    robot.dbg = {
      yawError, correction, encDiff, encCorrection,
      integral: c.yawIntegral, trim: robot.wallTrim,
      pwm: [robot.pwmL, robot.pwmR]
    };
  }
};

/* ========================== motion module ================================= */
const Motion = {
  // pivotTurn(): shared gyro-closed pivot with the calibrated dead-gyro fallback.
  startPivot(robot, lPwm, rPwm, targetDeg, fallbackMs, kind) {
    MPU6050.readangle(robot);
    MPU6050.snapshotAxis(robot);
    robot.turn = {
      kind, lPwm, rPwm, targetDeg, fallbackMs,
      head0: robot.gyroHeading,
      start: robot.t,
      active: true,
      cmdL: lPwm, cmdR: rPwm
    };
    robot.justTurned = true;
    MotorControl.run(robot, lPwm, rPwm);
  },
  // The firmware's 90 deg turns are SINGLE-WHEEL pivots (turnLeft90 =
  // motorRun(0, turnSpeed)), which swing the chassis about a stationary wheel and
  // leave it ~45 mm off the corridor centre line every time — fine on the 8x8 car
  // with a wall-follow fallback, fatal on a 16x16 grid where the next scan then
  // reads the wrong cell's walls. The simulator drives BOTH wheels (a centre
  // pivot, the same pattern the firmware already uses for turn180PivotLeft) so
  // the turn happens on the spot. Everything else — the gyro closing, the
  // slow-start fade and the dead-gyro fallback times — is unchanged.
  turnLeft90(robot) { Motion.startPivot(robot, -TURN_SPEED, TURN_SPEED, 90, TURN_90_LEFT_MS, 'L90'); },
  turnRight90(robot) { Motion.startPivot(robot, TURN_SPEED, -TURN_SPEED, -90, TURN_90_RIGHT_MS, 'R90'); },
  turn180(robot) { Motion.startPivot(robot, -TURN_SPEED, TURN_SPEED, 180, TURN_180_MS, '180'); },

  // one control step of an in-progress pivot; returns true while still turning
  step(robot) {
    const T = robot.turn;
    if (!T || !T.active) return false;
    const turned = robot.gyroHeading - T.head0;
    const error = T.targetDeg - turned;
    const headingLive = Math.abs(turned) > TURN_HEADING_ALIVE_DEG;

    if (headingLive && Math.abs(error) <= TURN_SLOW_START_DEG) {
      const s = clamp(TURN_SPEED * (Math.abs(error) / TURN_SLOW_START_DEG), TURN_MIN_PIVOT_PWM, TURN_SPEED);
      T.cmdL = T.lPwm ? Math.round(s * Math.sign(T.lPwm)) : 0;
      T.cmdR = T.rPwm ? Math.round(s * Math.sign(T.rPwm)) : 0;
    } else {
      T.cmdL = T.lPwm; T.cmdR = T.rPwm;
    }
    MotorControl.run(robot, T.cmdL, T.cmdR);

    if (headingLive && Math.abs(error) <= TURN_STOP_TOL_DEG) { T.done = 'heading'; }
    else if (!headingLive && robot.t - T.start >= T.fallbackMs) { T.done = 'fallback'; }
    else if (robot.t - T.start > TURN_TIMEOUT_MS) { T.done = 'timeout'; }

    if (T.done) {
      MotorControl.brake(robot);
      T.active = false;
      // The physical Grizzly is mounted so the gz yaw channel stays frozen, and
      // the motion module then leaves gyroAlive = false — runStraight() falls back to
      // the encoder equaliser + side-wall trim only. The simulator can model
      // either case (see the "MPU yaw loop" checkbox): GYRO_ALIVE = true shows
      // the intended P/I heading loop closing on the turns.
      robot.gyroAlive = GYRO_ALIVE && Math.abs(turned) > TURN_HEADING_ALIVE_DEG;
      return false;
    }
    return true;
  }
};

/* ============================== the plant ================================= */
/* Differential-drive chassis: PWM -> wheel speed -> pose. This is the only part
   of the simulator that is not in the firmware (it replaces the real hardware). */
class Robot {
  constructor() { this.reset(null); }

  reset(maze) {
    this.maze = maze;
    this.rects = maze ? wallRects(maze.walls) : [];
    this.t = 0;                                   // simulated milliseconds
    // pose: start cell (0,0) centre, nose North (90 deg CCW from +x)
    this.x = CELL_MM / 2;
    this.y = CELL_MM / 2;
    this.th = DIR_ANGLE[DIR_N];
    this.startTh = this.th;
    this.gyroHeading = norm360(this.th);
    this.yawFrozen = 0;                           // mpu6050 gz channel (frozen on this car)
    this.gyroAlive = GYRO_ALIVE;                  // motion module default: false
    this.justTurned = false;
    this.pwmL = 0; this.pwmR = 0;
    this.smoothL = 0; this.smoothR = 0;
    this.vL = 0; this.vR = 0;
    this.wallTrim = 0;
    this.wheelDistL = 0; this.wheelDistR = 0;
    this.odometer = 0;
    this.ctrl = { refYaw: norm360(this.th), refValid: false, yawIntegral: 0, encArmed: true };
    this.turn = null;
    this.braking = false;
    this.bumps = 0;
    this.stalled = 0;
    this.dbg = { yawError: 0, correction: 0, encDiff: 0, encCorrection: 0, integral: 0, trim: 0, pwm: [0, 0] };
    this.sens = { l: 999, m: 999, r: 999 };
  }

  wheelSpeed(pwm) {
    if (Math.abs(pwm) < PWM_STALL) return 0;
    return Math.sign(pwm) * (Math.abs(pwm) * MM_PER_S_PER_PWM);
  }

  /* integrate one control period (dtMs milliseconds of simulated time) */
  tick(dtMs) {
    const dt = dtMs / 1000.0;
    this.vL = this.wheelSpeed(this.pwmL) * LEFT_MOTOR_TRIM;
    this.vR = this.wheelSpeed(this.pwmR);

    const v = (this.vL + this.vR) / 2.0;                       // mm/s
    const omega = (this.vR - this.vL) / WHEELBASE_MM;          // rad/s, + = CCW = left
    this.th = norm360(this.th + omega * 180 / Math.PI * dt);
    this.gyroHeading += omega * 180 / Math.PI * dt;            // integrates like headingTrueDeg
    this.gyroHeading += 0.02 * dt;                             // small gyro bias drift

    const nx = this.x + v * Math.cos(this.th * Math.PI / 180) * dt;
    const ny = this.y + v * Math.sin(this.th * Math.PI / 180) * dt;

    // Wall contact: the chassis scrapes along the wall (see resolveCollision) and
    // the wheels keep turning, so the odometry is only credited with the distance
    // the car actually covered — that is the wheel slip that makes a real
    // dead-reckoned mouse drift when it rubs a wall.
    const res = resolveCollision(this.x, this.y, nx, ny, BODY_R_MM, this.rects);
    const want = Math.abs(v) * dt;
    const moved = Math.hypot(res.x - this.x, res.y - this.y);
    if (res.bumped) this.bumps++;
    const slipping = want > 0.01 && moved < 0.25 * want;
    if (slipping) {
      if (this.pwmL !== 0 || this.pwmR !== 0) this.stalled++;
    } else {
      this.stalled = 0;
      this.wheelDistL += this.vL * dt * (want > 0 ? moved / want : 1);
      this.wheelDistR += this.vR * dt * (want > 0 ? moved / want : 1);
    }
    this.odometer += moved;
    this.x = res.x; this.y = res.y;

    this.t += dtMs;
    return res.bumped;
  }
}

 