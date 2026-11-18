// Ровер для учебной сборки. Классическая Nano ATmega328P, два IBT-2.
// Д1: RPWM D5, LPWM D11. Д2: RPWM D10, LPWM D9.
// На каждом IBT-2 VCC, R_EN и L_EN подключены к общей шине +5 В.
// R_IS/L_IS, A0 и A1 свободны. SERVO iBUS S -> D0/RX.
// STOP задаёт оба PWM в LOW и электрически тормозит мотор.
// Полностью снять питание позволяет общий силовой выключатель S0.

#include <Arduino.h>
#include <stdio.h>

#if !defined(__AVR_ATmega328P__) && !defined(ROVER_HOST_TEST)
#error "Select the classic Nano with ATmega328P."
#endif

constexpr bool USB_CHECK = true;  // После проверки логики заменить на false.
constexpr byte forwardPin[2] = {5, 10};
constexpr byte reversePin[2] = {11, 9};
constexpr bool invertMotor[2] = {false, false};
constexpr uint16_t axisCenter[2] = {1500, 1500};  // CH1, CH2.
constexpr bool invertAxis[2] = {false, false};
constexpr int driveLimit = 110;  // Заполнение ШИМ, не ограничение тока.
constexpr int neutralBand = 60;
constexpr uint32_t linkTimeoutMs = 250;
constexpr uint32_t neutralHoldMs = 500;
constexpr uint32_t rampPeriodMs = 20;
constexpr uint32_t reversePauseMs = 400;
constexpr int rampUpStep = 4;
constexpr int rampDownStep = 8;

struct Motor {
  int target;
  int pwm;
  int8_t lastDirection;
  uint32_t stoppedAt;
};

Motor motors[2] = {};
uint16_t channels[6] = {};
byte frame[32];
byte frameLength = 0;
bool linkValid = false;
bool masterSeenOff = false;
bool driveArmed = false;
bool neutralPending = false;
uint32_t neutralSince = 0;
uint32_t lastFrameAt = 0;
uint32_t lastRampAt = 0;
uint32_t lastLogAt = 0;

int directionOf(int value) {
  return (value > 0) - (value < 0);
}

void writeMotor(byte index, int pwm) {
  if (USB_CHECK) pwm = 0;
  if (invertMotor[index]) pwm = -pwm;
  // Сначала обнуляется противоположный вход.
  if (pwm >= 0) {
    analogWrite(reversePin[index], 0);
    analogWrite(forwardPin[index], pwm);
  } else {
    analogWrite(forwardPin[index], 0);
    analogWrite(reversePin[index], -pwm);
  }
}

void stopAll(uint32_t now) {
  for (byte i = 0; i < 2; ++i) {
    if (motors[i].pwm != 0) motors[i].stoppedAt = now;
    motors[i].target = 0;
    motors[i].pwm = 0;
    writeMotor(i, 0);
  }
  driveArmed = false;
  neutralPending = false;
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

bool neutral(byte axis) {
  return abs(int(channels[axis]) - int(axisCenter[axis])) <= neutralBand;
}

int axisCommand(byte axis) {
  const int offset = int(channels[axis]) - int(axisCenter[axis]);
  const int magnitude = abs(offset);
  if (magnitude <= neutralBand) return 0;
  const int span = offset > 0 ? 2000 - axisCenter[axis] : axisCenter[axis] - 1000;
  const int pwm = int(map(min(magnitude, span), neutralBand, span, 0, driveLimit));
  return ((offset < 0) != invertAxis[axis]) ? -pwm : pwm;
}

void setDriveTargets() {
  const int turn = axisCommand(0);
  const int throttle = axisCommand(1);
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

bool validChannel(byte index) {
  return channels[index] >= 900 && channels[index] <= 2100;
}

void acceptFrame(uint32_t now) {
  for (byte i = 0; i < 6; ++i) {
    const byte pos = 2 + 2 * i;
    channels[i] = uint16_t(frame[pos]) | (uint16_t(frame[pos + 1]) << 8);
  }
  // Для двух ходовых используются только CH1, CH2 и CH5.
  if (!validChannel(0) || !validChannel(1) || !validChannel(4)) {
    loseLink(now);
    return;
  }
  linkValid = true;
  lastFrameAt = now;
  if (channels[4] < 1700) {
    stopAll(now);
    if (channels[4] <= 1300) masterSeenOff = true;
    return;
  }
  if (!driveArmed) {
    if (!masterSeenOff) return;
    if (!neutral(0) || !neutral(1)) {
      neutralPending = false;
      return;
    }
    if (!neutralPending) {
      neutralPending = true;
      neutralSince = now;
    }
    if (uint32_t(now - neutralSince) < neutralHoldMs) return;
    driveArmed = true;
    masterSeenOff = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  setDriveTargets();
}

void receiveByte(byte value) {
  frame[frameLength++] = value;
  if (frameLength < sizeof(frame)) return;
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
  // Скользящее окно восстанавливает синхронизацию после пропущенного байта.
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
  const int n = snprintf(line, sizeof(line), "C=%u,%u,%u,%u,%u,%u P=%d,%d %c%c\n",
      unsigned(channels[0]), unsigned(channels[1]), unsigned(channels[2]),
      unsigned(channels[3]), unsigned(channels[4]), unsigned(channels[5]),
      motors[0].pwm, motors[1].pwm, linkValid ? 'L' : '-', driveArmed ? 'D' : '-');
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
  }
  reportStatus(now);
}
