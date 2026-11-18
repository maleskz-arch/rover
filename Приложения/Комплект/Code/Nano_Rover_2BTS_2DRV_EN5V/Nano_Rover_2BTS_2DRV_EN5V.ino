// Rover Nano / 2 IBT-2 + 2 DRV8871. EN IBT-2 -> +5V. A0/A1 free.
// Nano ATmega328P, FlySky FS-iA6B, SERVO iBUS S -> D0/RX.
// M1, M2: ход. M3, M4: вертикаль и горизонталь левого стика.
// VCC, R_EN, L_EN каждого IBT-2 -> общая шина +5 В. R_IS/L_IS свободны.
// Для DRV8871: IN1/IN2/GND. VM -> питание по номиналу мотора, отдельного VCC нет.
// STOP на IBT-2 с EN=5V тормозит. STOP 00 на DRV8871 даёт свободный выбег.
// D2 и D4 — обычные цифровые выходы. writeAux регулирует скорость одним PWM в обе стороны.
// По направлениям DRV8871 используются разные режимы спада тока, реакция может отличаться.

#include <Arduino.h>
#include <stdio.h>
#include <util/atomic.h>

#if !defined(__AVR_ATmega328P__) && !defined(ROVER_HOST_TEST)
#error "Select the classic Nano with ATmega328P."
#endif

enum RoverProfile : byte { TWO_IBT, FOUR_IBT, TWO_IBT_TWO_DRV };
constexpr RoverProfile PROFILE = TWO_IBT_TWO_DRV;
constexpr bool HAS_AUX = PROFILE != TWO_IBT;
constexpr bool USB_CHECK = true; // true: моторные команды только в мониторе. EN IBT-2 остаются на +5 В.
constexpr bool AUX_SWITCH_REQUIRED = true;
constexpr byte MASTER_CHANNEL = 5;   // CH5: общее разрешение.
constexpr byte AUX_ENABLE_CHANNEL = 6; // CH6: разрешение М3 и М4.

struct Axis {
  byte channel;
  uint16_t center;
  bool invert;
};

constexpr Axis axes[4] = {
  {1, 1500, false}, // 0: правый стик, поворот.
  {2, 1500, false}, // 1: правый стик, газ.
  {3, 1500, false}, // 2: левый стик, вертикаль -> М3.
  {4, 1500, false}  // 3: левый стик, горизонталь -> М4.
};

constexpr bool invertMotor[4] = {false, false, false, false};
constexpr int driveLimit = 110;
constexpr int auxLimit = 110;         // Заполнение ШИМ, без измерения тока.
constexpr int neutralBand = 60;
constexpr uint32_t linkTimeoutMs = 250;
constexpr uint32_t neutralHoldMs = 500;
constexpr uint32_t rampPeriodMs = 20;
constexpr uint32_t reversePauseMs = 400;
constexpr int rampUpStep = 4;
constexpr int rampDownStep = 8;

constexpr byte forwardPin[2] = {5, 10};
constexpr byte reversePin[2] = {11, 9};
constexpr byte auxPwm[2] = {3, 6};
constexpr byte auxSecond[2] = {2, 4};
// PD3/PD2 и PD6/PD4 меняются попарно одной записью в PORTD.
constexpr uint8_t auxMask[2] = {uint8_t(_BV(PD3) | _BV(PD2)),
                                uint8_t(_BV(PD6) | _BV(PD4))};

static_assert(driveLimit > 0 && driveLimit <= 255 && auxLimit > 0 && auxLimit <= 255,
              "PWM limits must be in 1..255");
static_assert(MASTER_CHANNEL >= 1 && MASTER_CHANNEL <= 6 &&
              AUX_ENABLE_CHANNEL >= 1 && AUX_ENABLE_CHANNEL <= 6,
              "Switch channels must be in 1..6");
static_assert(axes[0].channel >= 1 && axes[0].channel <= 6 &&
              axes[1].channel >= 1 && axes[1].channel <= 6 &&
              axes[2].channel >= 1 && axes[2].channel <= 6 &&
              axes[3].channel >= 1 && axes[3].channel <= 6,
              "Axis channels must be in 1..6");
static_assert(axes[0].center > 1000 + neutralBand && axes[0].center < 2000 - neutralBand &&
              axes[1].center > 1000 + neutralBand && axes[1].center < 2000 - neutralBand &&
              axes[2].center > 1000 + neutralBand && axes[2].center < 2000 - neutralBand &&
              axes[3].center > 1000 + neutralBand && axes[3].center < 2000 - neutralBand,
              "Joystick centers must leave room for the deadband");

struct Motor {
  int target;
  int pwm;
  int8_t lastDirection;
  uint32_t stoppedAt;
};

Motor motors[4] = {};
int8_t auxElectricalDirection[2] = {};
uint16_t channels[6] = {};
byte frame[32];
byte frameLength = 0;
bool linkValid = false;
bool masterSeenOff = false;
bool driveArmed = false;
bool auxArmed = false;
bool driveNeutralPending = false;
bool auxNeutralPending = false;
uint32_t driveNeutralSince = 0;
uint32_t auxNeutralSince = 0;
uint32_t lastFrameAt = 0;
uint32_t lastRampAt = 0;
uint32_t lastLogAt = 0;

int directionOf(int value) {
  return (value > 0) - (value < 0);
}

void writeAux(byte index, int pwm) {
  const int8_t direction = int8_t(directionOf(pwm));
  const int8_t previous = auxElectricalDirection[index];
  if (direction != previous) {
    // Снять ШИМ на уровне, равном второму входу. Разность напряжений исчезает.
    analogWrite(auxPwm[index], previous < 0 ? 255 : 0);
    // Перевести оба входа в новый исходный уровень без одиночного фронта.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (direction < 0) PORTD = uint8_t(PORTD | auxMask[index]);
      else PORTD = uint8_t(PORTD & uint8_t(~auxMask[index]));
    }
    auxElectricalDirection[index] = direction;
  }
  if (direction > 0) analogWrite(auxPwm[index], pwm);
  if (direction < 0) analogWrite(auxPwm[index], 255 - abs(pwm));
}

void writeMotor(byte index, int pwm) {
  if (USB_CHECK) pwm = 0;
  if (invertMotor[index]) pwm = -pwm;
  if (index >= 2) {
    writeAux(index - 2, pwm);
    return;
  }
  if (pwm >= 0) {
    analogWrite(reversePin[index], 0);
    analogWrite(forwardPin[index], pwm);
  } else {
    analogWrite(forwardPin[index], 0);
    analogWrite(reversePin[index], -pwm);
  }
}

void stopMotor(byte index, uint32_t now) {
  Motor& motor = motors[index];
  if (motor.pwm != 0) motor.stoppedAt = now;
  motor.target = 0;
  motor.pwm = 0;
  writeMotor(index, 0);
}

void stopAux(uint32_t now) {
  if (!HAS_AUX) return;
  stopMotor(2, now);
  stopMotor(3, now);
  auxArmed = false;
  auxNeutralPending = false;
}

void stopAll(uint32_t now) {
  stopAux(now);
  stopMotor(0, now);
  stopMotor(1, now);
  driveArmed = false;
  driveNeutralPending = false;
  digitalWrite(LED_BUILTIN, LOW);
}

void loseLink(uint32_t now) {
  stopAll(now);
  linkValid = false;
  masterSeenOff = false;
}

void checkTimeout(uint32_t now) {
  if (linkValid && uint32_t(now - lastFrameAt) >= linkTimeoutMs) {
    loseLink(now);
    frameLength = 0;
  }
}

bool neutral(byte index) {
  const Axis& axis = axes[index];
  return abs(int(channels[axis.channel - 1]) - int(axis.center)) <= neutralBand;
}

int axisCommand(byte index, int limit) {
  const Axis& axis = axes[index];
  const int offset = int(channels[axis.channel - 1]) - int(axis.center);
  const int magnitude = abs(offset);
  if (magnitude <= neutralBand) return 0;
  const int span = offset > 0 ? 2000 - axis.center : axis.center - 1000;
  const int pwm = int(map(min(magnitude, span), neutralBand, span, 0, limit));
  const bool negative = (offset < 0) != axis.invert;
  return negative ? -pwm : pwm;
}

bool heldNeutral(bool centered, uint32_t now, bool& pending, uint32_t& since) {
  if (!centered) {
    pending = false;
    return false;
  }
  if (!pending) {
    pending = true;
    since = now;
  }
  return uint32_t(now - since) >= neutralHoldMs;
}

void setDriveTargets() {
  const int throttle = axisCommand(1, driveLimit);
  const int turn = axisCommand(0, driveLimit);
  int left = throttle + turn;
  int right = throttle - turn;
  const int peak = max(abs(left), abs(right));
  if (peak > driveLimit) {
    left = int(long(left) * driveLimit / peak);
    right = int(long(right) * driveLimit / peak);
  }
  motors[0].target = left;
  motors[1].target = right;
}

bool validChannel(byte number) {
  const uint16_t value = channels[number - 1];
  return value >= 900 && value <= 2100;
}

void acceptFrame(uint32_t now) {
  for (byte i = 0; i < 6; ++i) {
    const byte pos = 2 + 2 * i;
    channels[i] = uint16_t(frame[pos]) | (uint16_t(frame[pos + 1]) << 8);
  }
  for (byte i = 0; i < (HAS_AUX ? 4 : 2); ++i) {
    if (!validChannel(axes[i].channel)) {
      loseLink(now);
      return;
    }
  }
  if (!validChannel(MASTER_CHANNEL) ||
      (HAS_AUX && AUX_SWITCH_REQUIRED && !validChannel(AUX_ENABLE_CHANNEL))) {
    loseLink(now);
    return;
  }
  linkValid = true;
  lastFrameAt = now;
  const uint16_t master = channels[MASTER_CHANNEL - 1];
  if (master < 1700) {
    stopAll(now);
    if (master <= 1300) masterSeenOff = true;
    return;
  }
  if (!driveArmed) {
    if (!masterSeenOff || !heldNeutral(neutral(0) && neutral(1), now,
                                     driveNeutralPending, driveNeutralSince)) return;
    driveArmed = true;
    masterSeenOff = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  setDriveTargets();
  if (!HAS_AUX) return;

  if (AUX_SWITCH_REQUIRED && channels[AUX_ENABLE_CHANNEL - 1] < 1700) {
    stopAux(now);
    return;
  }
  if (!auxArmed) {
    if (!heldNeutral(neutral(2) && neutral(3), now,
                     auxNeutralPending, auxNeutralSince)) return;
    auxArmed = true;
  }
  motors[2].target = axisCommand(2, auxLimit);
  motors[3].target = axisCommand(3, auxLimit);
}

void receiveByte(byte value) {
  frame[frameLength++] = value;
  if (frameLength < sizeof(frame)) return;

  // 32 байта SERVO iBUS: заголовок 20 40, 14 каналов, контрольная сумма.
  if (frame[0] == 0x20 && frame[1] == 0x40) {
    uint16_t checksum = 0xFFFF;
    for (byte i = 0; i < 30; ++i) checksum -= frame[i];
    const uint16_t expected = uint16_t(frame[30]) | (uint16_t(frame[31]) << 8);
    if (checksum == expected) {
      frameLength = 0;
      acceptFrame(millis());
      return;
    }
  }
  for (byte i = 1; i < sizeof(frame); ++i) frame[i - 1] = frame[i];
  frameLength = sizeof(frame) - 1;
}

void rampMotor(byte index, uint32_t now) {
  Motor& motor = motors[index];
  int goal = motor.target;
  const int before = motor.pwm;
  const int wanted = directionOf(goal);
  if (before != 0 && wanted != 0 && directionOf(before) != wanted) goal = 0;
  if (before == 0 && wanted != 0 && motor.lastDirection != 0 &&
      motor.lastDirection != wanted && uint32_t(now - motor.stoppedAt) < reversePauseMs) return;

  const int step = abs(goal) > abs(before) ? rampUpStep : rampDownStep;
  int next = before;
  if (goal > before) next = min(before + step, goal);
  if (goal < before) next = max(before - step, goal);
  if (next == before) return;
  if (next == 0) motor.stoppedAt = now;
  else motor.lastDirection = int8_t(directionOf(next));
  motor.pwm = next;
  writeMotor(index, next);
}

void reportStatus(uint32_t now) {
  if (uint32_t(now - lastLogAt) < 250) return;
  char line[64];
  const int n = snprintf(line, sizeof(line), "C=%u,%u,%u,%u,%u,%u P=%d,%d,%d,%d %c%c%c\n",
      unsigned(channels[0]), unsigned(channels[1]), unsigned(channels[2]),
      unsigned(channels[3]), unsigned(channels[4]), unsigned(channels[5]),
      motors[0].pwm, motors[1].pwm, motors[2].pwm, motors[3].pwm,
      linkValid ? 'L' : '-', driveArmed ? 'D' : '-', auxArmed ? 'A' : '-');
  if (n > 0 && n < int(sizeof(line)) && Serial.availableForWrite() >= n) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), size_t(n));
    lastLogAt = now;
  }
}

void setup() {
  for (byte i = 0; i < 2; ++i) {
    digitalWrite(forwardPin[i], LOW);
    digitalWrite(reversePin[i], LOW);
    pinMode(forwardPin[i], OUTPUT);
    pinMode(reversePin[i], OUTPUT);
    if (HAS_AUX) {
      digitalWrite(auxPwm[i], LOW);
      digitalWrite(auxSecond[i], LOW);
      pinMode(auxPwm[i], OUTPUT);
      pinMode(auxSecond[i], OUTPUT);
    }
  }
  pinMode(LED_BUILTIN, OUTPUT);
  stopAll(millis());
  Serial.begin(115200);
}

void loop() {
  checkTimeout(millis());
  for (byte n = 0; n < 64 && Serial.available() > 0; ++n) {
    const int value = Serial.read();
    if (value >= 0) receiveByte(byte(value));
  }
  const uint32_t now = millis();
  checkTimeout(now);
  if (uint32_t(now - lastRampAt) >= rampPeriodMs) {
    lastRampAt = now;
    if (driveArmed) {
      rampMotor(0, now);
      rampMotor(1, now);
    }
    if (auxArmed) {
      rampMotor(2, now);
      rampMotor(3, now);
    }
  }
  reportStatus(now);
}
