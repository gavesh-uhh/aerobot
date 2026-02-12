#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SH1107_SEEED_128X128_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// =====================================================
// PIN MAP
// =====================================================

const int PIN_FAN = 8;        // Fan Relay
const int PIN_PIR = 2;        // PIR motion sensor digital input
const int PIN_MQ2 = A0;       // MQ-2 analog gas sensor
const int PIN_MQ7 = A1;       // MQ-7 analog gas sensor
const int PIN_MQ135 = A2;     // MQ-135 analog gas sensor
const int PIN_LDR = A3;       // LDR analog input
const int PIN_SERVO = 11;     // Servo
const int PIN_TRIG = 12;      // Ultrasonic trigger
const int PIN_ECHO = 13;      // Ultrasonic echo
const int PIN_LEFT_IN1 = 9;   // L298N left motor input 1 (PWM)
const int PIN_LEFT_IN2 = 10;  // L298N left motor input 2 (PWM)
const int PIN_RIGHT_IN1 = 5;  // L298N right motor input 1 (PWM)
const int PIN_RIGHT_IN2 = 6;  // L298N right motor input 2 (PWM)
const int PIN_BUZZER = 7;     // Buzzer

// =====================================================
// ROBOT NAVIGATION / AVOIDANCE TUNING
// =====================================================

const int PROBE_OFFSET_DEG = 35;              // Servo offset for side probing (± from center)
const unsigned long PROBE_INTERVAL_MS = 145;  // Time between probe attempts while moving forward
const unsigned long PROBE_SETTLE_MS = 40;     // Extra settle time at probe angle (ms)
const unsigned long PROBE_CENTER_MS = 25;     // Small settle when returning to center (ms)
const float TARGET_SIDE = 30.0f;
const float SIDE_SOFT_CM = 35.0f;                  // Side “caution” distance: start steering away
const float SIDE_HARD_CM = 25.0f;                  // Side “hard” distance: force turn away
const unsigned long SIDE_AVOID_COOLDOWN_MS = 250;  // Cooldown after side-avoid turn before probing again

const unsigned long STEER_HOLD_MS = 550;  // Hold steerBias effect for this long (ms)
const int STEER_DELTA_PWM = 22;           // Typical steer strength (PWM delta)

const unsigned long NAV_UPDATE_MS = 25;  // Navigation decision tick interval (ms)
const unsigned long DISPLAY_MS = 500;    // OLED refresh interval (ms)

const unsigned long TURN_MS = 300;          // Normal turn duration (ms)
const unsigned long TURN_SHORT_MS = 225;    // Shorter “nudge” turn duration (ms)
const unsigned long BACKUP_MS = 150;        // Normal backup duration (ms)
const unsigned long BRAKE_MS = 2;           // Brake delay before backing (ms)
const unsigned long BRAKE_CAUTION_MS = 25;  // Brake delay for caution case (ms)
const unsigned long FORWARD_MIN_MS = 250;   // Minimum time to keep moving forward once chosen (ms)

const float AVOID_DIST_CM = 28.0f;    // “Hard obstacle” threshold (front)
const float CAUTION_DIST_CM = 38.0f;  // “Soft obstacle” threshold (front)
const float AVOID_DIST_HYST = 3.5f;   // Hysteresis to avoid oscillating near threshold
const float TURN_BALANCE_CM = 5.5f;   // If L/R scan distances are within this, alternate turns
const int INVALID_DIST_LIMIT = 3;     // How many invalid front reads allowed before treating as blocked

const float STUCK_LOW_DIST_CM = 45.0f;        // If front stays under this, consider “stuck”
const unsigned long STUCK_LOW_DIST_MS = 500;  // How long low distance must persist (ms)
const float STUCK_MIN_DELTA_CM = 6.0f;        // Minimum change in distance that counts as “progress”
const unsigned long STUCK_CHECK_MS = 670;     // No-progress time window (ms)
const unsigned long STUCK_BACKUP_MS = 150;    // Backup duration when stuck recovery triggers (ms)
const unsigned long STUCK_COOLDOWN_MS = 800;  // Cooldown after stuck recovery (ms)

// =====================================================
// MOTOR / DRIVE CONFIG
// =====================================================

const bool INVERT_LEFT_MOTOR = true;   // Flip left motor direction if wiring reversed
const bool INVERT_RIGHT_MOTOR = true;  // Flip right motor direction if wiring reversed
const int LEFT_MOTOR_TRIM = 0;         // Small correction for left PWM
const int RIGHT_MOTOR_TRIM = 0;        // Small correction for right PWM

const int MIN_PWM = 45;    // Minimum effective PWM (below this motors may not move)
const int DRIVE_PWM = 65;  // Forward/back cruising PWM
const int TURN_PWM = 90;   // Turning PWM (spin turns)

// =====================================================
// ULTRASONIC CONFIG
// =====================================================

const unsigned long ULTRASONIC_TIMEOUT_US = 12500;  // pulseIn timeout (us) ~ max ~280cm-ish
const float CM_PER_MICROSECOND = 0.0343f / 2.0f;    // Speed of sound conversion to cm (round-trip /2)

// =====================================================
// TIMING / TELEMETRY
// =====================================================

const unsigned long DIR_DEADTIME_MS = 2;  // Deadtime when reversing motor direction (ms)
const unsigned long TELEMETRY_MS = 100;   // Telemetry frame period (ms)

// =====================================================
// SERVO CONFIG (MANUAL PULSE CONTROL)
// =====================================================

const int SERVO_ANGLE_LEFT = 135;   // Servo angle for left scan
const int SERVO_ANGLE_CENTER = 75;  // Servo center angle
const int SERVO_ANGLE_RIGHT = 35;   // Servo angle for right scan

const unsigned long SERVO_SETTLE_MS = 120;  // Typical settle time for scans (ms)
const int SERVO_PULSE_MIN_US = 600;         // Servo min pulse width (us)
const int SERVO_PULSE_MAX_US = 2400;        // Servo max pulse width (us)
const int SERVO_PERIOD_US = 20000;          // Servo frame period (us)

// =====================================================
// FIRE (FROM ESP32) CONFIG
// =====================================================

const float FIRE_ON_TH = 0.65f;            // Fire turns ON above this confidence
const float FIRE_OFF_TH = 0.45f;           // Fire turns OFF below this confidence
const unsigned long FIRE_STALE_MS = 2500;  // If no new fire frame in this time, treat as stale

// =====================================================
// GAS CONFIG
// =====================================================

const unsigned long GAS_CALIBRATION_MS = 6000;  // Calibration duration at startup (ms)

const float GAS_RISE_SEEK = 12.0f;       // “Rising fast” threshold to enter/keep hunt
const int GAS_ANOMALY_LEVEL = 120;       // Absolute gas level considered “anomaly”
const int GAS_LOCK_MARGIN = 6;           // Within peak-margin + low rise => lock condition
const unsigned long GAS_LOST_MS = 2250;  // If gas not seen for this long, drop to PASSIVE

// =====================================================
// RUNTIME VARIABLES (DO NOT CHANGE SET VALUES)
// =====================================================

// ---- Fire runtime ----
static float fireConfidence = -1.0f;       // Latest fire confidence [0..1], -1 unknown
static unsigned long lastFireReceive = 0;  // Last time fire frame received (ms)
static bool fireActive = false;            // Hysteresis-applied fire state
static bool fanOn = false;                 // Cached fan state
static unsigned long fanHoldUntilMs = 0;   // Fan hold timer (ms)

// ---- Motor runtime ----
static int lastLeftSign = 0;   // Last commanded direction sign for left motor
static int lastRightSign = 0;  // Last commanded direction sign for right motor

// ---- Gas calibration + filtering runtime ----
static int base2 = 0, base7 = 0, base135 = 0;  // Baselines from calibration
static int gasDead = 20;                       // Noise deadband level
static int gasOut = 0;                         // Filtered gas “strength” output
static bool gasActive = false;                 // Hysteresis on/off around deadband
static float gasEma = 0;                       // Slow EMA for rise detection
static float gasFast = 0;                      // Fast EMA for rise detection
static bool gasFilterInit = false;             // Prevent boot spike in rise calc

struct SensorSnapshot {
  float distCenter;
  int gasRaw;
  float gasRise;
  bool pir;
  int ldr;
};

static SensorSnapshot sensors;
static unsigned long lastTelemetryMs = 0;  // Last telemetry send time (ms)

// =====================================================
// NAVIGATION STATE MACHINE
// =====================================================
enum LocomotionState { STOPPED = 0,
                       FORWARD,
                       BACKWARD,
                       TURN_LEFT,
                       TURN_RIGHT };

static unsigned long lastDisplayMs = 0;
static LocomotionState locoState = STOPPED;

static LocomotionState pendingTurn = STOPPED;  // queued turn after backup
static bool pendingTurnValid = false;
static unsigned long pendingTurnDurMs = TURN_MS;

static unsigned long stateUntilMs = 0;     // locomotion state end time
static unsigned long lastNavUpdateMs = 0;  // nav tick timing

static int invalidDistCount = 0;  // consecutive invalid ultrasonic reads

static int steerBias = 0;               // differential steering bias
static unsigned long steerUntilMs = 0;  // when steer bias should decay/clear

static float probeL = -1.0f;    // last left probe reading
static float probeR = -1.0f;    // last right probe reading
static uint8_t probePhase = 0;  // 0 = left next, 1 = right next

static bool lastTurnLeft = false;  // turn alternation memory

static float lastDistForStuck = -1.0f;        // last distance used for progress detection
static unsigned long lastDistChangeMs = 0;    // last time distance changed enough
static unsigned long lowDistSinceMs = 0;      // when we entered low-distance band
static unsigned long lastStuckRecoverMs = 0;  // stuck recovery cooldown marker
static unsigned long lastBackupEndMs = 0;     // last backup end time marker

static unsigned long lastScanMs = 0;        // last time we did L/R scan
static unsigned long lastCommitMs = 0;      // last time we committed a turn direction
static uint8_t failStreak = 0;              // repeated fail escalator
static unsigned long lastWanderTurnMs = 0;  // periodic wander timer
static unsigned long lastProbeMs = 0;       // last side probe time
static unsigned long sideAvoidUntilMs = 0;  // cooldown window after side avoid

// =====================================================
// GAS HUNT STATE MACHINE
// =====================================================
enum GasHuntState { PASSIVE = 0,
                    HUNT,
                    LOCK };
static GasHuntState gasState = PASSIVE;
static unsigned long gasStateSinceMs = 0;
static unsigned long lastGasSeenMs = 0;
static unsigned long lastPeakMs = 0;
static int gasPeak = 0;

void setLocomotionState(LocomotionState s, unsigned long durationMs) {
  locoState = s;
  stateUntilMs = millis() + durationMs;
}

void debug(const char *tag, const char *msg) {
  Serial.print('#');
  Serial.print(tag);
  Serial.print(',');
  Serial.println(msg);
}

void applyLocomotion() {
  switch (locoState) {
    case STOPPED:
      steerBias = 0;
      stopAllMotors();
      break;

    case FORWARD:
      {
        int b = steerBias;
        if (b > 35)
          b = 35;
        if (b < -35)
          b = -35;

        int base = DRIVE_PWM;
        if (sensors.distCenter > 0) {
          if (sensors.distCenter < 55.0f) base = DRIVE_PWM - 10;
          else if (sensors.distCenter > 120.0f) base = DRIVE_PWM + 10;
        }
        int lp = base + b;
        int rp = base - b;
        driveMotorsDifferential(lp, true, rp, true);
      }
      break;

    case BACKWARD:
      steerBias = 0;
      driveMotorsDifferential(DRIVE_PWM, false, DRIVE_PWM, false);
      break;

    case TURN_LEFT:
      steerBias = 0;
      driveMotorsDifferential(TURN_PWM, false, TURN_PWM, true);
      break;

    case TURN_RIGHT:
      steerBias = 0;
      driveMotorsDifferential(TURN_PWM, true, TURN_PWM, false);
      break;
  }
}

bool obstacleTooClose() {
  if (sensors.distCenter < 0)
    return invalidDistCount >= INVALID_DIST_LIMIT;
  return sensors.distCenter <= AVOID_DIST_CM;
}

bool obstacleClear() {
  if (sensors.distCenter < 0)
    return true;
  return sensors.distCenter >= (AVOID_DIST_CM + AVOID_DIST_HYST);
}

void servoPulseUs(int us) {
  digitalWrite(PIN_SERVO, HIGH);
  delayMicroseconds(us);
  digitalWrite(PIN_SERVO, LOW);
  delayMicroseconds(SERVO_PERIOD_US - us);
}

void servoWriteAngleBlocking(int angleDeg, unsigned long settleMs) {
  if (angleDeg < 0)
    angleDeg = 0;
  if (angleDeg > 180)
    angleDeg = 180;
  int us = map(angleDeg, 0, 180, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  unsigned long start = millis();
  while (millis() - start < settleMs) {
    servoPulseUs(us);
  }
}

unsigned long computeBackupMs(float d, unsigned long baseMs) {
  if (d <= 0)
    return baseMs;

  if (d < 18.0f)
    return baseMs + 80;
  if (d < 28.0f)
    return baseMs + 40;
  if (d < 40.0f)
    return baseMs;
  return baseMs - 80;
}

float readDistanceAtAngle(int angleDeg) {
  servoWriteAngleBlocking(angleDeg, SERVO_SETTLE_MS);
  return readUltrasonicMedian();
}

void updateStuckMonitor() {
  unsigned long now = millis();

  if (sensors.distCenter <= 0)
    return;

  if (lastDistForStuck < 0) {
    lastDistForStuck = sensors.distCenter;
    lastDistChangeMs = now;
  }

  float diff = sensors.distCenter - lastDistForStuck;
  if (diff < 0)
    diff = -diff;

  if (diff >= STUCK_MIN_DELTA_CM) {
    lastDistForStuck = sensors.distCenter;
    lastDistChangeMs = now;
  }

  if (sensors.distCenter < STUCK_LOW_DIST_CM) {
    if (lowDistSinceMs == 0)
      lowDistSinceMs = now;
  } else {
    lowDistSinceMs = 0;
  }
}

bool isStuck(unsigned long now) {
  if (now - lastStuckRecoverMs < STUCK_COOLDOWN_MS)
    return false;
  if (sensors.distCenter <= 0)
    return false;
  if (locoState != FORWARD && locoState != TURN_LEFT && locoState != TURN_RIGHT)
    return false;
  bool noDelta = (now - lastDistChangeMs) > STUCK_CHECK_MS;
  bool lowDistPersist =
    (lowDistSinceMs != 0) && ((now - lowDistSinceMs) > STUCK_LOW_DIST_MS);
  return noDelta || lowDistPersist;
}

LocomotionState pickTurnFromScan(float left, float right) {
  bool leftValid = left > 0;
  bool rightValid = right > 0;
  LocomotionState turn;
  if (leftValid && rightValid) {
    float diff = left - right;
    if (diff < 0)
      diff = -diff;
    if (diff <= TURN_BALANCE_CM) {
      turn = lastTurnLeft ? TURN_RIGHT : TURN_LEFT;
    } else {
      turn = (left >= right) ? TURN_LEFT : TURN_RIGHT;
    }
  } else if (leftValid) {
    turn = TURN_LEFT;
  } else if (rightValid) {
    turn = TURN_RIGHT;
  } else {
    turn = lastTurnLeft ? TURN_RIGHT : TURN_LEFT;
  }
  lastTurnLeft = (turn == TURN_LEFT);
  return turn;
}

void updateGasHuntState(unsigned long now) {
  bool gasRising = (sensors.gasRise >= GAS_RISE_SEEK);
  bool gasAnomaly = (sensors.gasRaw >= GAS_ANOMALY_LEVEL) && (sensors.gasRise >= GAS_RISE_SEEK);

  if (gasAnomaly)
    lastGasSeenMs = now;

  if (sensors.gasRaw > gasPeak + 2) {
    gasPeak = sensors.gasRaw;
    lastPeakMs = now;
  }

  switch (gasState) {
    case PASSIVE:
      if (gasAnomaly) {
        gasState = HUNT;
        gasStateSinceMs = now;
      }
      break;
    case HUNT:
      if (now - lastGasSeenMs > GAS_LOST_MS) {
        gasState = PASSIVE;
        gasStateSinceMs = now;
        gasPeak = 0;
      } else if ((now - gasStateSinceMs) > 1000 && sensors.gasRaw >= (gasPeak - GAS_LOCK_MARGIN) && sensors.gasRise < 2.0f) {
        gasState = LOCK;
        gasStateSinceMs = now;
      }
      break;
    case LOCK:
      if (now - lastGasSeenMs > GAS_LOST_MS) {
        gasState = PASSIVE;
        gasStateSinceMs = now;
        gasPeak = 0;
      } else if (gasRising && sensors.gasRaw > gasPeak + GAS_LOCK_MARGIN) {
        gasState = HUNT;
        gasStateSinceMs = now;
      }
      break;
  }
}

void navigationUpdate() {
  unsigned long now = millis();

  // PATCH: Direction Decay
  if (steerBias != 0 && now >= steerUntilMs) {
    steerBias = (steerBias * 7) / 10;
    if (steerBias > -2 && steerBias < 2) steerBias = 0;
    steerUntilMs = now + 60;
  }

  if (locoState == BACKWARD && sensors.distCenter > (CAUTION_DIST_CM + 12)) {
    stateUntilMs = millis();
  }

  if (now - lastNavUpdateMs < NAV_UPDATE_MS)
    return;
  lastNavUpdateMs = now;

  updateGasHuntState(now);
  if (gasState == HUNT && sensors.gasRaw > gasPeak - 6) lastCommitMs = now;

  bool distValid = (sensors.distCenter > 0);
  float d = sensors.distCenter;
  bool invalidBlocked = (!distValid && invalidDistCount >= INVALID_DIST_LIMIT && locoState == FORWARD);
  bool hardObstacle = distValid ? (d <= AVOID_DIST_CM) : invalidBlocked;
  bool softObstacle = distValid ? (d <= CAUTION_DIST_CM) : invalidBlocked;
  bool clearEnough = distValid ? (d >= (CAUTION_DIST_CM + AVOID_DIST_HYST)) : !invalidBlocked;

  // PATCH : 90deg and 45deg corners
  bool cornerTrap =
    (softObstacle && !hardObstacle) && (probeL > 0 && probeL < 32.0f) && (probeR > 0 && probeR < 32.0f);

  if (cornerTrap && failStreak >= 2) {
    pendingTurnValid = false;
    stopAllMotors();
    delay(BRAKE_CAUTION_MS);
    pendingTurn = lastTurnLeft ? TURN_RIGHT : TURN_LEFT;
    lastTurnLeft = !lastTurnLeft;
    pendingTurnDurMs = TURN_MS + 220;
    pendingTurnValid = true;
    setLocomotionState(BACKWARD, BACKUP_MS + 220);
    return;
  }

  if (gasState == LOCK && !softObstacle) {
    pendingTurnValid = false;
    servoWriteAngleBlocking(SERVO_ANGLE_CENTER, SERVO_SETTLE_MS);
    setLocomotionState(STOPPED, 500);
    return;
  }

  if (now < stateUntilMs) {
    if (locoState == BACKWARD)
      return;
    if (locoState != FORWARD)
      return;

    if (!hardObstacle && !softObstacle && now >= sideAvoidUntilMs && (now - lastProbeMs) >= PROBE_INTERVAL_MS && (now - lastScanMs) >= 250) {
      bool skipProbe = distValid && (d > 150.0f) && (probeL > 0 && probeL > 70.0f) && (probeR > 0 && probeR > 70.0f);

      if (!skipProbe) {
        lastProbeMs = now;

        if (steerBias > 12) probePhase = 0;        // right
        else if (steerBias < -12) probePhase = 1;  // left

        int ang = SERVO_ANGLE_CENTER + (probePhase ? +PROBE_OFFSET_DEG : -PROBE_OFFSET_DEG);

        servoWriteAngleBlocking(ang, PROBE_SETTLE_MS);
        float pd = readUltrasonicCm();
        servoWriteAngleBlocking(SERVO_ANGLE_CENTER, PROBE_CENTER_MS);

        if (ang > SERVO_ANGLE_CENTER) probeL = pd;
        else probeR = pd;

        probePhase ^= 1;

        int b = steerBias;

        if (probeL > 0 && probeR > 0) {
          float diff = probeR - probeL;  // + => left closer => steer right
          b = (int)(diff * 1.6f);
          if (b > 35) b = 35;
          if (b < -35) b = -35;
        } else if (probeL > 0 && probeL < TARGET_SIDE) {
          b = +32;  // was +18 (stronger push away)
        } else if (probeR > 0 && probeR < TARGET_SIDE) {
          b = -32;  // was -18
        } else {
          b = (b * 7) / 10;
        }

        steerBias = b;
        steerUntilMs = now + STEER_HOLD_MS;

        bool leftSoft = (probeL > 0 && probeL <= SIDE_SOFT_CM);
        bool rightSoft = (probeR > 0 && probeR <= SIDE_SOFT_CM);
        bool leftHard = (probeL > 0 && probeL <= SIDE_HARD_CM);
        bool rightHard = (probeR > 0 && probeR <= SIDE_HARD_CM);

        if (rightHard || (rightSoft && !leftSoft)) {
          pendingTurnValid = false;
          lastTurnLeft = true;
          sideAvoidUntilMs = now + SIDE_AVOID_COOLDOWN_MS;
          setLocomotionState(TURN_LEFT, TURN_SHORT_MS);
          return;
        } else if (leftHard || (leftSoft && !rightSoft)) {
          pendingTurnValid = false;
          lastTurnLeft = false;
          sideAvoidUntilMs = now + SIDE_AVOID_COOLDOWN_MS;
          setLocomotionState(TURN_RIGHT, TURN_SHORT_MS);
          return;
        } else if (leftSoft && rightSoft) {
          pendingTurnValid = false;
          sideAvoidUntilMs = now + (SIDE_AVOID_COOLDOWN_MS + 120);
          setLocomotionState(lastTurnLeft ? TURN_LEFT : TURN_RIGHT, TURN_SHORT_MS);
          return;
        }
      }
    }

    if (!(hardObstacle || softObstacle))
      return;
  }

  if (locoState == BACKWARD) {
    lastBackupEndMs = now;
  }

  if (pendingTurnValid) {
    setLocomotionState(pendingTurn, pendingTurnDurMs);
    pendingTurnValid = false;
    pendingTurnDurMs = TURN_MS;
    return;
  }

  if (isStuck(now)) {
    bool canScan = (now - lastScanMs) >= 450;
    LocomotionState turn;

    if (canScan) {
      lastScanMs = now;
      float left = readDistanceAtAngle(SERVO_ANGLE_LEFT);
      float right = readDistanceAtAngle(SERVO_ANGLE_RIGHT);
      servoWriteAngleBlocking(SERVO_ANGLE_CENTER, SERVO_SETTLE_MS);
      turn = pickTurnFromScan(left, right);
    } else {
      turn = lastTurnLeft ? TURN_RIGHT : TURN_LEFT;
      lastTurnLeft = !lastTurnLeft;
    }

    pendingTurn = turn;
    pendingTurnDurMs = TURN_MS + 120;
    pendingTurnValid = true;

    lastStuckRecoverMs = now;
    failStreak = 0;
    unsigned long backMs = computeBackupMs(sensors.distCenter, STUCK_BACKUP_MS);
    setLocomotionState(BACKWARD, backMs);
    return;
  }

  if (hardObstacle) {
    bool rightAfterBackup = (now - lastBackupEndMs) < 1100;
    bool noProgress = (now - lastDistChangeMs) > 900;

    if (rightAfterBackup || noProgress) {
      if (failStreak < 6)
        failStreak++;
    } else if (clearEnough) {
      failStreak = 0;
    }

    stopAllMotors();
    delay(BRAKE_MS);

    bool canScan = (now - lastScanMs) >= 450;
    LocomotionState turn;

    if (rightAfterBackup && failStreak >= 2) {
      turn = lastTurnLeft ? TURN_RIGHT : TURN_LEFT;
      lastTurnLeft = !lastTurnLeft;
      lastCommitMs = now;
    } else if (canScan) {
      lastScanMs = now;
      float left = readDistanceAtAngle(SERVO_ANGLE_LEFT);
      float right = readDistanceAtAngle(SERVO_ANGLE_RIGHT);
      servoWriteAngleBlocking(SERVO_ANGLE_CENTER, PROBE_CENTER_MS);
      turn = pickTurnFromScan(left, right);
      lastCommitMs = now;
    } else {
      if (now - lastCommitMs > 1200) {
        turn = lastTurnLeft ? TURN_RIGHT : TURN_LEFT;
        lastTurnLeft = !lastTurnLeft;
        lastCommitMs = now;
      } else {
        turn = lastTurnLeft ? TURN_LEFT : TURN_RIGHT;
      }
    }

    pendingTurn = turn;
    pendingTurnValid = true;

    unsigned long backupDur = BACKUP_MS + (unsigned long)failStreak * 70;
    if (backupDur > BACKUP_MS + 280)
      backupDur = BACKUP_MS + 280;

    unsigned long turnDur = TURN_MS + (unsigned long)failStreak * 50;
    if (turnDur > TURN_MS + 250)
      turnDur = TURN_MS + 250;

    pendingTurnDurMs = turnDur;
    setLocomotionState(BACKWARD, backupDur);
    return;
  }

  if (clearEnough && failStreak > 0)
    failStreak--;

  if (softObstacle) {
    bool canScan = (now - lastScanMs) >= 450;
    LocomotionState turn;

    if (canScan) {
      lastScanMs = now;

      stopAllMotors();
      delay(BRAKE_CAUTION_MS);

      float left = readDistanceAtAngle(SERVO_ANGLE_LEFT);
      float right = readDistanceAtAngle(SERVO_ANGLE_RIGHT);
      servoWriteAngleBlocking(SERVO_ANGLE_CENTER, SERVO_SETTLE_MS);

      turn = pickTurnFromScan(left, right);
      lastCommitMs = now;
    } else {
      turn = lastTurnLeft ? TURN_LEFT : TURN_RIGHT;
    }

    setLocomotionState(turn, TURN_SHORT_MS);
    return;
  }

  if (gasState == HUNT) {
    bool gasRising = (sensors.gasRise >= GAS_RISE_SEEK);

    if (gasRising || sensors.gasRaw > gasPeak - 8 || (now - lastPeakMs) < 1200) {
      setLocomotionState(FORWARD, FORWARD_MIN_MS);
    } else {
      if (now - lastCommitMs > 1600) {
        lastTurnLeft = !lastTurnLeft;
        lastCommitMs = now;
      }
      setLocomotionState(lastTurnLeft ? TURN_LEFT : TURN_RIGHT, TURN_SHORT_MS);
    }
    return;
  }

  if (now - lastWanderTurnMs > 7500) {
    lastWanderTurnMs = now;
    lastTurnLeft = !lastTurnLeft;
    setLocomotionState(lastTurnLeft ? TURN_LEFT : TURN_RIGHT, TURN_SHORT_MS);
    return;
  }

  setLocomotionState(FORWARD, FORWARD_MIN_MS);
}

void setFan(bool on) {
  fanOn = on;
  digitalWrite(PIN_FAN, on ? LOW : HIGH);
}

void playCalibrationTickBeep() {
  static unsigned long lastBeepMs = 0;
  unsigned long now = millis();
  if (now - lastBeepMs >= 300) {
    lastBeepMs = now;
    tone(PIN_BUZZER, 1100, 18);
  }
}

void playStartupBeep() {
  static unsigned long lastBeepMs = 0;
  unsigned long now = millis();
  if (now - lastBeepMs >= 300) {
    lastBeepMs = now;
    tone(PIN_BUZZER, 1000, 30);
    delay(35);
    tone(PIN_BUZZER, 1300, 35);
  }
}

void playCalibrationDoneBeep() {
  static unsigned long lastBeepMs = 0;
  unsigned long now = millis();
  if (now - lastBeepMs >= 300) {
    lastBeepMs = now;
    tone(PIN_BUZZER, 1200, 40);
    delay(45);
    tone(PIN_BUZZER, 900, 60);
    delay(70);
  }
}

int clampPwm(int pwm) {
  if (pwm < 0)
    pwm = 0;
  if (pwm > 255)
    pwm = 255;
  if (pwm > 0 && pwm < MIN_PWM)
    pwm = MIN_PWM;
  return pwm;
}

void stopAllMotors() {
  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}

void driveMotorPair(int in1, int in2, int pwm, bool forward, bool invert,
                    int &lastSign) {
  pwm = clampPwm(pwm);
  bool dir = forward ^ invert;

  int sign = (pwm == 0) ? 0 : (forward ? +1 : -1);
  if (sign != 0 && lastSign != 0 && sign != lastSign) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    delay(DIR_DEADTIME_MS);
  }
  lastSign = sign;

  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }

  if (dir) {
    digitalWrite(in2, LOW);
    analogWrite(in1, pwm);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, pwm);
  }
}

void driveMotorsDifferential(int leftPwm, bool leftFwd, int rightPwm,
                             bool rightFwd) {
  driveMotorPair(PIN_LEFT_IN1, PIN_LEFT_IN2, leftPwm + LEFT_MOTOR_TRIM, leftFwd,
                 INVERT_LEFT_MOTOR, lastLeftSign);
  driveMotorPair(PIN_RIGHT_IN1, PIN_RIGHT_IN2, rightPwm + RIGHT_MOTOR_TRIM,
                 rightFwd, INVERT_RIGHT_MOTOR, lastRightSign);
}

float readUltrasonicCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0)
    return -1.0f;

  float dist = duration * CM_PER_MICROSECOND;
  if (dist < 2.0f) return 1.5f;
  if (dist > 350.0f) return -1.0f;
  return dist;
}

float readUltrasonicMedian() {
  float a = readUltrasonicCm();
  delay(2);
  float b = readUltrasonicCm();
  delay(2);
  float c = readUltrasonicCm();

  int valid = 0;
  if (a > 0)
    valid++;
  if (b > 0)
    valid++;
  if (c > 0)
    valid++;
  if (valid < 2)
    return -1.0f;

  if (a < 0)
    a = 999.0f;
  if (b < 0)
    b = 999.0f;
  if (c < 0)
    c = 999.0f;

  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }

  return b;
}

int readAnalogStable(int pin) {
  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);
  int d = analogRead(pin);
  int e = analogRead(pin);

  int x1 = a, x2 = b, x3 = c, x4 = d, x5 = e;

  if (x1 > x2) {
    int t = x1;
    x1 = x2;
    x2 = t;
  }
  if (x4 > x5) {
    int t = x4;
    x4 = x5;
    x5 = t;
  }
  if (x1 > x3) {
    int t = x1;
    x1 = x3;
    x3 = t;
  }
  if (x2 > x3) {
    int t = x2;
    x2 = x3;
    x3 = t;
  }
  if (x4 > x3) {
    int t = x4;
    x4 = x3;
    x3 = t;
  }
  if (x5 > x3) {
    int t = x5;
    x5 = x3;
    x3 = t;
  }

  return x3;
}

int integerSqrt(long x) {
  if (x <= 0)
    return 0;
  long r = 0, bit = 1L << 30;
  while (bit > x)
    bit >>= 2;
  while (bit != 0) {
    if (x >= r + bit) {
      x -= r + bit;
      r = (r >> 1) + bit;
    } else
      r >>= 1;
    bit >>= 2;
  }
  return (int)r;
}

void calibrateGasSensors(unsigned long ms) {
  long s2 = 0, s7 = 0, s135 = 0;
  long s2sq = 0, s7sq = 0, s135sq = 0;
  int n = 0;

  unsigned long start = millis();
  while (millis() - start < ms) {
    playCalibrationTickBeep();

    int mq2 = readAnalogStable(PIN_MQ2);
    int mq7 = readAnalogStable(PIN_MQ7);
    int mq135 = readAnalogStable(PIN_MQ135);

    s2 += mq2;
    s7 += mq7;
    s135 += mq135;
    s2sq += (long)mq2 * mq2;
    s7sq += (long)mq7 * mq7;
    s135sq += (long)mq135 * mq135;
    n++;

    delay(30);
  }

  if (n < 5)
    n = 5;

  base2 = (int)(s2 / n);
  base7 = (int)(s7 / n);
  base135 = (int)(s135 / n);

  long m2 = base2, m7 = base7, m135 = base135;
  long v2 = (s2sq / n) - (m2 * m2);
  long v7 = (s7sq / n) - (m7 * m7);
  long v135 = (s135sq / n) - (m135 * m135);
  if (v2 < 0)
    v2 = 0;
  if (v7 < 0)
    v7 = 0;
  if (v135 < 0)
    v135 = 0;

  int sd2 = integerSqrt(v2);
  int sd7 = integerSqrt(v7);
  int sd135 = integerSqrt(v135);

  int sdMax = sd2;
  if (sd7 > sdMax)
    sdMax = sd7;
  if (sd135 > sdMax)
    sdMax = sd135;

  gasDead = sdMax * 3;
  if (gasDead < 12)
    gasDead = 12;
  if (gasDead > 80)
    gasDead = 80;

  gasOut = 0;
  gasActive = false;
  gasFilterInit = false;

  playCalibrationDoneBeep();
}

int readCalibratedGasStrength() {
  static int md1 = 0, md2 = 0, md3 = 0;
  static bool mdInit = false;

  int mq2 = readAnalogStable(PIN_MQ2);
  int mq7 = readAnalogStable(PIN_MQ7);
  int mq135 = readAnalogStable(PIN_MQ135);

  int d2 = mq2 - base2;
  if (d2 < 0)
    d2 = 0;
  int d7 = mq7 - base7;
  if (d7 < 0)
    d7 = 0;
  int d135 = mq135 - base135;
  if (d135 < 0)
    d135 = 0;

  int maxDelta = d2;
  if (d7 > maxDelta)
    maxDelta = d7;
  if (d135 > maxDelta)
    maxDelta = d135;

  if (!mdInit) {
    md1 = md2 = md3 = maxDelta;
    mdInit = true;
  } else {
    md1 = md2;
    md2 = md3;
    md3 = maxDelta;
  }

  int a = md1, b = md2, c = md3;
  if (a > b) {
    int t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    int t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    int t = a;
    a = b;
    b = t;
  }
  maxDelta = b;

  int onTh = gasDead + 30;
  int offTh = gasDead + 14;

  if (!gasActive) {
    if (maxDelta >= onTh)
      gasActive = true;
  } else {
    if (maxDelta <= offTh)
      gasActive = false;
  }

  int target = 0;
  if (gasActive) {
    target = maxDelta - offTh;
    if (target < 0)
      target = 0;
  }

  int maxJump = gasDead * 3;
  if (maxJump < 35)
    maxJump = 35;
  if (maxJump > 160)
    maxJump = 160;

  int diff = target - gasOut;
  if (diff > maxJump)
    target = gasOut + maxJump;
  if (diff < -maxJump)
    target = gasOut - maxJump;

  if (!gasActive && gasOut > 0) {
    gasOut = (gasOut * 9) / 10;
    if (gasOut < 2)
      gasOut = 0;
    return gasOut;
  }

  gasOut = target;
  return gasOut;
}

void updateSensorSnapshot() {
  static unsigned long lastDistMs = 0;
  unsigned long now = millis();

  if (now - lastDistMs >= 45) {
    lastDistMs = now;
    sensors.distCenter = readUltrasonicCm();
    if (sensors.distCenter < 0) {
      if (invalidDistCount < 1000)
        invalidDistCount++;
    } else {
      invalidDistCount = 0;
    }
    updateStuckMonitor();
  }

  sensors.pir = (digitalRead(PIN_PIR) == HIGH);
  sensors.ldr = analogRead(PIN_LDR);
  sensors.gasRaw = readCalibratedGasStrength();
  if (!gasFilterInit) {
    gasEma = (float)sensors.gasRaw;
    gasFast = (float)sensors.gasRaw;
    sensors.gasRise = 0.0f;
    gasFilterInit = true;
  } else {
    gasEma = gasEma * 0.96f + sensors.gasRaw * 0.04f;
    gasFast = gasFast * 0.93f + sensors.gasRaw * 0.07f;
    sensors.gasRise = gasFast - gasEma;
  }
}

void readTelemetryFrame() {
  static char line[48];
  static uint8_t len = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (len == 0)
        continue;
      line[len] = '\0';
      len = 0;

      const char *pfx = "FIRE32,confidence=";
      const size_t pfxLen = 17;

      if (strncmp(line, pfx, pfxLen) == 0) {
        const char *v = line + pfxLen;
        char *endp = nullptr;
        float val = (float)strtod(v, &endp);
        if (endp != v) {
          fireConfidence = val;
          lastFireReceive = millis();
        }
      }
      continue;
    }

    if (len < sizeof(line) - 1)
      line[len++] = c;
    else
      len = 0;
  }
}

void sendTelemetryFrame() {
  unsigned long now = millis();
  if (now - lastTelemetryMs < TELEMETRY_MS)
    return;
  lastTelemetryMs = now;

  int mq2 = readAnalogStable(PIN_MQ2);
  int mq7 = readAnalogStable(PIN_MQ7);
  int mq135 = readAnalogStable(PIN_MQ135);

  int lvl = sensors.gasRaw;
  int rise = (int)(sensors.gasRise);
  int pir = sensors.pir ? 1 : 0;
  int ldr = sensors.ldr;

  Serial.print("UNO,1,");
  Serial.print((int)locoState);
  Serial.print(",");
  Serial.print((int)gasState);
  Serial.print(",");
  Serial.print(mq2);
  Serial.print(",");
  Serial.print(mq7);
  Serial.print(",");
  Serial.print(mq135);
  Serial.print(",");
  Serial.print(lvl);
  Serial.print(",");
  Serial.print(rise);
  Serial.print(",");
  Serial.print(pir);
  Serial.print(",");
  if (sensors.distCenter > 0)
    Serial.print(sensors.distCenter, 2);
  else
    Serial.print(-1);
  Serial.print(",");
  Serial.print(ldr);
  Serial.print("\n");
}

static int clampi(int v, int lo, int hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static int mapClamp(int x, int inMin, int inMax, int outMin, int outMax) {
  if (inMax == inMin)
    return outMin;
  long v =
    (long)(x - inMin) * (outMax - outMin) / (long)(inMax - inMin) + outMin;
  if (outMin < outMax)
    return clampi((int)v, outMin, outMax);
  return clampi((int)v, outMax, outMin);
}

static void drawBar(U8G2 &d, int x, int y, int w, int h, int fill) {
  fill = clampi(fill, 0, w);
  d.drawFrame(x, y, w, h);
  if (fill > 0)
    d.drawBox(x + 1, y + 1, fill - 2 >= 0 ? fill - 2 : 0, h - 2);
}

static void drawPill(U8G2 &d, int x, int y, const char *text) {
  int tw = d.getStrWidth(text);
  int w = tw + 10;
  int h = 10;
  d.drawRFrame(x, y, w, h, 3);
  d.setCursor(x + 5, y + 8);
  d.print(text);
}

void displayTick(U8G2 &display) {
  display.firstPage();
  do {
    display.setFont(u8g2_font_5x8_tr);

    const char *locoLabel = "STOP";
    switch (locoState) {
      case FORWARD:
        locoLabel = "FWD";
        break;
      case BACKWARD:
        locoLabel = "BACK";
        break;
      case TURN_LEFT:
        locoLabel = "LEFT";
        break;
      case TURN_RIGHT:
        locoLabel = "RIGHT";
        break;
      default:
        break;
    }

    const char *gasLabel = "PASS";
    switch (gasState) {
      case HUNT:
        gasLabel = "HUNT";
        break;
      case LOCK:
        gasLabel = "LOCK";
        break;
      default:
        break;
    }

    bool distValid = (sensors.distCenter > 0);
    float dcm = sensors.distCenter;

    bool fireValid = (millis() - lastFireReceive) <= FIRE_STALE_MS;
    int firePct = (int)(fireConfidence * 100.0f + 0.5f);
    firePct = clampi(firePct, 0, 100);

    const char *status = "OK";
    if (fireValid && fireActive)
      status = "FIRE";
    else if (gasState == LOCK)
      status = "LOCK";
    else if (distValid && dcm <= AVOID_DIST_CM)
      status = "AVOID";
    else if (gasState == HUNT)
      status = "HUNT";

    display.setCursor(0, 8);
    display.print("AEROBOT ");
    display.print(status);

    drawPill(display, 0, 12, locoLabel);
    drawPill(display, 48, 12, gasLabel);

    display.setCursor(92, 20);
    display.print(sensors.pir ? "P" : "-");
    display.print(fanOn ? "F" : "-");
    display.print(fireValid ? "V" : "S");

    int distFill = 0;
    if (distValid) {
      int dInt = (int)(dcm + 0.5f);
      distFill = mapClamp(dInt, 80, 10, 0, 100);
    }

    display.setCursor(0, 34);
    display.print("DIST");
    if (distValid) {
      display.setCursor(34, 34);
      display.print((int)(dcm + 0.5f));
      display.print("cm");
    } else {
      display.setCursor(34, 34);
      display.print("n/a");
    }

    display.drawFrame(0, 36, 128, 10);
    int fillPx = mapClamp(distFill, 0, 100, 0, 128);
    if (fillPx > 0)
      display.drawBox(0, 36, fillPx, 10);

    int gasFill = mapClamp(sensors.gasRaw, 0, 250, 0, 100);
    display.setCursor(0, 58);
    display.print("GAS ");
    display.print(sensors.gasRaw);

    display.drawFrame(0, 60, 128, 8);
    int gasPx = mapClamp(gasFill, 0, 100, 0, 128);
    if (gasPx > 0)
      display.drawBox(0, 60, gasPx, 8);

    float r = sensors.gasRise;
    if (r > 20)
      r = 20;
    if (r < -20)
      r = -20;
    int cx = 64;
    int needle = cx + (int)((r / 20.0f) * 60.0f);

    display.setCursor(0, 78);
    display.print("RISE ");
    display.print(sensors.gasRise, 1);

    display.drawHLine(0, 84, 128);
    display.drawVLine(cx, 80, 9);
    display.drawVLine(needle, 80, 9);

    display.setCursor(0, 98);
    display.print("FIRE ");
    if (!fireValid) {
      display.print("stale");
    } else {
      display.print(fireActive ? "ON " : "off ");
      display.print(firePct);
      display.print("%");
    }

    display.drawFrame(0, 100, 128, 8);
    int firePx = mapClamp(firePct, 0, 100, 0, 128);
    if (fireValid && firePx > 0)
      display.drawBox(0, 100, firePx, 8);

    display.setCursor(0, 122);
    display.print("LDR ");
    display.print(sensors.ldr);

  } while (display.nextPage());
}

void soundTick() {
  unsigned long now = millis();
  static GasHuntState prevGas = PASSIVE;
  static bool prevFire = false;
  static unsigned long oneShotUntilMs = 0;
  static unsigned long oneShotNextMs = 0;
  static uint8_t oneShotStep = 0;
  static uint8_t oneShotType = 0;
  static unsigned long huntNextMs = 0;

  bool fireValid = (now - lastFireReceive) <= FIRE_STALE_MS;
  if (!fireValid) {
    fireActive = false;
  } else {
    if (!fireActive && fireConfidence >= FIRE_ON_TH)
      fireActive = true;
    if (fireActive && fireConfidence <= FIRE_OFF_TH)
      fireActive = false;
  }

  if (gasState != prevGas) {
    if (gasState == LOCK) {
      oneShotType = 1;
      oneShotStep = 0;
      oneShotNextMs = 0;
      oneShotUntilMs = now + 600;
    }
    prevGas = gasState;
  }

  if (fireActive != prevFire) {
    if (fireActive) {
      oneShotType = 2;
      oneShotStep = 0;
      oneShotNextMs = 0;
      oneShotUntilMs = now + 700;
    } else {
      oneShotType = 3;
      oneShotStep = 0;
      oneShotNextMs = 0;
      oneShotUntilMs = now + 450;
    }
    prevFire = fireActive;
  }

  if (now < oneShotUntilMs) {
    if (oneShotNextMs == 0 || now >= oneShotNextMs) {
      if (oneShotType == 1) {
        if (oneShotStep == 0) {
          tone(PIN_BUZZER, 700, 120);
          oneShotNextMs = now + 180;
          oneShotStep = 1;
        } else {
          tone(PIN_BUZZER, 700, 120);
          oneShotNextMs = now + 260;
          oneShotStep = 0;
        }
      } else if (oneShotType == 2) {
        if (oneShotStep == 0) {
          tone(PIN_BUZZER, 1050, 90);
          oneShotNextMs = now + 140;
          oneShotStep = 1;
        } else if (oneShotStep == 1) {
          tone(PIN_BUZZER, 1050, 90);
          oneShotNextMs = now + 140;
          oneShotStep = 2;
        } else {
          tone(PIN_BUZZER, 1050, 120);
          oneShotNextMs = now + 250;
          oneShotStep = 0;
        }
      } else {
        tone(PIN_BUZZER, 520, 320);
        oneShotNextMs = now + 360;
        oneShotStep = 0;
      }
    }
    return;
  }

  if (fireActive) {
    static unsigned long fireNextMs = 0;
    static bool fireOn = false;
    static bool fireHi = false;

    if (now >= fireNextMs) {
      fireOn = !fireOn;
      if (fireOn) {
        fireHi = !fireHi;
        tone(PIN_BUZZER, fireHi ? 1200 : 900);
        fireNextMs = now + 100;
      } else {
        noTone(PIN_BUZZER);
        fireNextMs = now + 80;
      }
    }
    return;
  }

  if (gasState == LOCK) {
    static unsigned long lockNextMs = 0;
    static uint8_t lockStep = 0;

    if (now >= lockNextMs) {
      if (lockStep == 0) {
        tone(PIN_BUZZER, 780, 160);
        lockNextMs = now + 220;
        lockStep = 1;
      } else if (lockStep == 1) {
        tone(PIN_BUZZER, 780, 160);
        lockNextMs = now + 520;
        lockStep = 0;
      }
    }
    return;
  }

  if (gasState == HUNT) {
    if (now >= huntNextMs) {
      tone(PIN_BUZZER, 650, 35);
      huntNextMs = now + 1200;
    }
    return;
  }

  noTone(PIN_BUZZER);
}

void updateFanLogic() {
  unsigned long now = millis();
  if (fireActive || gasState == LOCK) {
    fanHoldUntilMs = now + 3500;
  }
  bool shouldFan = now < fanHoldUntilMs;
  setFan(shouldFan);
}

void setup() {
  playStartupBeep();
  Serial.begin(57600);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  digitalWrite(PIN_FAN, LOW);
  display.begin();
  display.setBusClock(400000);
  display.clearBuffer();
  display.sendBuffer();
  setFan(false);
  stopAllMotors();
  servoWriteAngleBlocking(SERVO_ANGLE_CENTER, SERVO_SETTLE_MS);
  calibrateGasSensors(GAS_CALIBRATION_MS);
}

void loop() {
  unsigned long now = millis();
  readTelemetryFrame();
  updateSensorSnapshot();
  navigationUpdate();
  updateFanLogic();
  applyLocomotion();
  soundTick();
  if ((now - lastDisplayMs >= DISPLAY_MS)) {
    lastDisplayMs = now;
    displayTick(display);
  }
  sendTelemetryFrame();
}
