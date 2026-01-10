#include "TLx493D_inc.hpp"
#include <EEPROM.h>
#include <avr/wdt.h>

using namespace ifx::tlx493d;

// EEPROM
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_DATA_ADDR = 4;
const uint32_t EEPROM_MAGIC = 0xCAFE493D;

// Calibration data
struct CalibData {
  double maxLeft, maxRight;
  double maxBack, maxForward;
  double maxDown;
};

TLx493D_A1B6 sensor(Wire, TLx493D_IIC_ADDR_A0_e);

// Baseline (center position)
double idleX = 0, idleY = 0, idleZ = 0;

// Direction thresholds
CalibData calib = {0, 0, 0, 0, 0};

// Config
const double NOISE_MARGIN = 0.5;
const int BTN_1 = 4;
const int BTN_2 = 5;

// State
bool logEnabled = true;
int8_t lastDir = 0;  // 1-5 for directions, 0 for idle
uint8_t lastMag = 0;

// Direction constants
enum Dir { IDLE = 0, LEFT = 1, RIGHT = 2, BACK = 3, FORWARD = 4, DOWN = 5 };

double getAxisDelta(int axis) {
  double idle[] = {idleX, idleY, idleZ};
  double sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    double x, y, z;
    if (sensor.getMagneticField(&x, &y, &z)) {
      double vals[] = {x, y, z};
      sum += vals[axis];
      count++;
    }
    delay(50);
  }
  return count > 0 ? (sum / count) - idle[axis] : 0;
}

void calibrateDirection(const __FlashStringHelper* name, double* maxVal, int axis, bool negative) {
  Serial.print(F("Move to "));
  Serial.print(name);
  Serial.println(F(" max, hold 2s..."));

  // Wait for movement
  while (true) {
    double delta = getAxisDelta(axis);
    if (negative) delta = -delta;
    if (delta > NOISE_MARGIN) {
      Serial.println(F("Movement detected, hold..."));
      break;
    }
    delay(50);
  }

  // Sample for 2 seconds
  double peak = 0;
  for (int i = 0; i < 20; i++) {
    double delta = getAxisDelta(axis);
    if (negative) delta = -delta;
    if (delta > peak) peak = delta;
    Serial.print(F("  "));
    Serial.println(delta, 2);
    delay(100);
  }

  *maxVal = peak;
  Serial.print(name);
  Serial.print(F(" max: "));
  Serial.println(peak, 2);
  Serial.println();
}

void saveToEEPROM() {
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_DATA_ADDR, calib);
  Serial.println(F("Saved to EEPROM"));
}

bool loadFromEEPROM() {
  uint32_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC) return false;
  EEPROM.get(EEPROM_DATA_ADDR, calib);
  return true;
}

void calibrateBaseline() {
  Serial.println(F("Calibrating baseline... hold still"));
  delay(1000);

  double sumX = 0, sumY = 0, sumZ = 0;
  int count = 0;

  for (int i = 0; i < 20; i++) {
    double x, y, z;
    if (sensor.getMagneticField(&x, &y, &z)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      count++;
    }
    delay(100);
  }

  if (count > 0) {
    idleX = sumX / count;
    idleY = sumY / count;
    idleZ = sumZ / count;
    Serial.print(F("Baseline: "));
    Serial.print(idleX, 2);
    Serial.print(F(", "));
    Serial.print(idleY, 2);
    Serial.print(F(", "));
    Serial.println(idleZ, 2);
  } else {
    Serial.println(F("Baseline failed!"));
  }
}

void calibrateDirections() {
  Serial.println(F("\n=== DIRECTION CALIBRATION ==="));
  calibrateDirection(F("LEFT"), &calib.maxLeft, 0, true);
  calibrateDirection(F("RIGHT"), &calib.maxRight, 0, false);
  calibrateDirection(F("BACK"), &calib.maxBack, 1, true);
  calibrateDirection(F("FORWARD"), &calib.maxForward, 1, false);
  calibrateDirection(F("DOWN"), &calib.maxDown, 2, true);
  saveToEEPROM();
  Serial.println(F("=== COMPLETE ==="));
}

void calibrate() {
  calibrateBaseline();
  calibrateDirections();
  Serial.println(F("Ready!"));
}

void showEEPROM() {
  if (!loadFromEEPROM()) {
    Serial.println(F("EEPROM: empty"));
    return;
  }
  Serial.println(F("EEPROM:"));
  Serial.print(F("  L:")); Serial.print(calib.maxLeft, 1);
  Serial.print(F(" R:")); Serial.print(calib.maxRight, 1);
  Serial.print(F(" B:")); Serial.print(calib.maxBack, 1);
  Serial.print(F(" F:")); Serial.print(calib.maxForward, 1);
  Serial.print(F(" D:")); Serial.println(calib.maxDown, 1);
}

uint8_t getMagnitude(double delta, double maxVal) {
  if (maxVal <= NOISE_MARGIN) return 0;
  double ratio = abs(delta) / maxVal;
  if (ratio >= 0.66) return 3;
  if (ratio >= 0.33) return 2;
  if (abs(delta) >= NOISE_MARGIN) return 1;
  return 0;
}

const __FlashStringHelper* magStr(uint8_t m) {
  switch (m) {
    case 3: return F("STRONG");
    case 2: return F("MEDIUM");
    default: return F("WEAK");
  }
}

const __FlashStringHelper* dirStr(int8_t d) {
  switch (d) {
    case LEFT: return F("LEFT");
    case RIGHT: return F("RIGHT");
    case BACK: return F("BACK");
    case FORWARD: return F("FORWARD");
    case DOWN: return F("DOWN");
    default: return F("");
  }
}

void processEvent(double dX, double dY, double dZ) {
  double absX = abs(dX);
  double absY = abs(dY);
  double absZ = dZ < 0 ? abs(dZ) : 0;  // Only DOWN

  // Find dominant direction
  int8_t dir = IDLE;
  double maxDelta = 0;
  double maxVal = 0;

  if (absX >= absY && absX >= absZ && absX > NOISE_MARGIN) {
    dir = dX > 0 ? RIGHT : LEFT;
    maxDelta = absX;
    maxVal = dX > 0 ? calib.maxRight : calib.maxLeft;
  } else if (absY >= absX && absY >= absZ && absY > NOISE_MARGIN) {
    dir = dY > 0 ? FORWARD : BACK;
    maxDelta = absY;
    maxVal = dY > 0 ? calib.maxForward : calib.maxBack;
  } else if (absZ > NOISE_MARGIN) {
    dir = DOWN;
    maxDelta = absZ;
    maxVal = calib.maxDown;
  }

  uint8_t mag = getMagnitude(maxDelta, maxVal);

  // Only print on change
  if (dir != lastDir || mag != lastMag) {
    lastDir = dir;
    lastMag = mag;
    if (mag > 0 && logEnabled) {
      Serial.print(F("EVENT: "));
      Serial.print(dirStr(dir));
      Serial.print(F(" "));
      Serial.println(magStr(mag));
    }
  }
}

void checkButtons() {
  static bool last1 = HIGH, last2 = HIGH;
  static unsigned long lastTime = 0;

  if (millis() - lastTime < 200) return;

  bool btn1 = digitalRead(BTN_1);
  bool btn2 = digitalRead(BTN_2);

  if (btn1 == LOW && last1 == HIGH) {
    Serial.println(F("BTN1"));
    lastTime = millis();
  }
  if (btn2 == LOW && last2 == HIGH) {
    Serial.println(F("BTN2"));
    lastTime = millis();
  }

  last1 = btn1;
  last2 = btn2;
}

void checkSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  while (Serial.available()) Serial.read();

  switch (cmd) {
    case 'c': case 'C': calibrate(); break;
    case 'e': case 'E': showEEPROM(); break;
    case 'l': case 'L':
      logEnabled = !logEnabled;
      Serial.print(F("Log: "));
      Serial.println(logEnabled ? F("ON") : F("OFF"));
      break;
    case 'r': case 'R':
      Serial.println(F("Rebooting..."));
      delay(100);
      wdt_enable(WDTO_15MS);
      while (1) {}
    case 'h': case 'H': case '?':
      Serial.println(F("C=calibrate E=EEPROM L=log R=reboot"));
      break;
  }
}

void setup() {
  wdt_disable();
  Serial.begin(9600);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);

  Serial.println(F("TLV493D 3-Axis Monitor"));

  if (!sensor.begin()) {
    Serial.println(F("Sensor init failed!"));
    while (1) delay(1000);
  }

  calibrateBaseline();

  if (loadFromEEPROM()) {
    Serial.print(F("EEPROM: L:"));
    Serial.print(calib.maxLeft, 1);
    Serial.print(F(" R:"));
    Serial.print(calib.maxRight, 1);
    Serial.print(F(" B:"));
    Serial.print(calib.maxBack, 1);
    Serial.print(F(" F:"));
    Serial.print(calib.maxForward, 1);
    Serial.print(F(" D:"));
    Serial.println(calib.maxDown, 1);
    Serial.println(F("Ready! (C=recalibrate)"));
  } else {
    Serial.println(F("No EEPROM data, calibrating..."));
    calibrateDirections();
    Serial.println(F("Ready!"));
  }
}

void loop() {
  checkSerial();
  checkButtons();

  double sumX = 0, sumY = 0, sumZ = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    double x, y, z;
    if (sensor.getMagneticField(&x, &y, &z)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      count++;
    }
    for (int j = 0; j < 5; j++) {
      checkButtons();
      delay(10);
    }
  }

  if (count > 0) {
    double dX = (sumX / count) - idleX;
    double dY = (sumY / count) - idleY;
    double dZ = (sumZ / count) - idleZ;

    double maxDelta = max(max(abs(dX), abs(dY)), dZ < 0 ? abs(dZ) : 0);
    if (maxDelta >= NOISE_MARGIN) {
      processEvent(dX, dY, dZ);
    }
  }
}
