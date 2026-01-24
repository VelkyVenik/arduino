#include "TLx493D_inc.hpp"
#include <EEPROM.h>
#include <avr/wdt.h>
#include <MIDIUSB.h>
#include <LedControl.h>

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

// LED Matrix pins
const int LED_DIN = 7;
const int LED_CS = 8;
const int LED_CLK = 9;
LedControl lc = LedControl(LED_DIN, LED_CLK, LED_CS, 1);

// MIDI config
const uint8_t MIDI_CHANNEL = 0;  // Channel 1
const uint8_t MIDI_CC_LEFT = 1;
const uint8_t MIDI_CC_RIGHT = 2;
const uint8_t MIDI_CC_BACK = 3;
const uint8_t MIDI_CC_FORWARD = 4;
const uint8_t MIDI_CC_DOWN = 5;
const uint8_t MIDI_NOTE_BTN1 = 60;  // C4
const uint8_t MIDI_NOTE_BTN2 = 62;  // D4
const uint8_t MIDI_VELOCITY = 127;

// State
bool logEnabled = true;
bool midiEnabled = true;
bool ledEnabled = true;
bool btn1State = false;  // Toggle state for button 1
bool btn2State = false;  // Toggle state for button 2
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

void printCalibData(const __FlashStringHelper* prefix) {
  Serial.print(prefix);
  Serial.print(F("L:")); Serial.print(calib.maxLeft, 1);
  Serial.print(F(" R:")); Serial.print(calib.maxRight, 1);
  Serial.print(F(" B:")); Serial.print(calib.maxBack, 1);
  Serial.print(F(" F:")); Serial.print(calib.maxForward, 1);
  Serial.print(F(" D:")); Serial.println(calib.maxDown, 1);
}

void showEEPROM() {
  if (!loadFromEEPROM()) {
    Serial.println(F("EEPROM: empty"));
    return;
  }
  printCalibData(F("EEPROM: "));
}

uint8_t getMagnitude(double delta, double maxVal) {
  if (maxVal <= NOISE_MARGIN || abs(delta) < NOISE_MARGIN) return 0;
  double ratio = abs(delta) / maxVal;
  // Thresholds: 80%=5, 60%=4, 40%=3, 20%=2, else=1
  static const double thresholds[] = {0.80, 0.60, 0.40, 0.20};
  for (uint8_t i = 0; i < 4; i++) {
    if (ratio >= thresholds[i]) return 5 - i;
  }
  return 1;
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

uint8_t dirToCC(int8_t d) {
  // Map direction enum (1-5) to MIDI CC (1-5)
  static const uint8_t ccMap[] = {0, MIDI_CC_LEFT, MIDI_CC_RIGHT, MIDI_CC_BACK, MIDI_CC_FORWARD, MIDI_CC_DOWN};
  return (d >= 1 && d <= 5) ? ccMap[d] : 0;
}

uint8_t magToValue(uint8_t mag) {
  // Map magnitude 0-5 to MIDI values: 0, 25, 51, 76, 102, 127
  static const uint8_t values[] = {0, 25, 51, 76, 102, 127};
  return mag <= 5 ? values[mag] : 127;
}

void sendMidiCC(uint8_t cc, uint8_t value) {
  midiEventPacket_t event = {0x0B, 0xB0 | MIDI_CHANNEL, cc, value};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}

void sendMidiNoteOn(uint8_t note) {
  midiEventPacket_t event = {0x09, 0x90 | MIDI_CHANNEL, note, MIDI_VELOCITY};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}

void sendMidiNoteOff(uint8_t note) {
  midiEventPacket_t event = {0x08, 0x80 | MIDI_CHANNEL, note, 0};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}

// LED patterns (8x8 bitmaps)
const byte PATTERN_LEFT[8] = {
  B00010000,
  B00110000,
  B01111111,
  B11111111,
  B01111111,
  B00110000,
  B00010000,
  B00000000
};

const byte PATTERN_RIGHT[8] = {
  B00001000,
  B00001100,
  B11111110,
  B11111111,
  B11111110,
  B00001100,
  B00001000,
  B00000000
};

const byte PATTERN_FORWARD[8] = {
  B00011000,
  B00111100,
  B01111110,
  B11111111,
  B00011000,
  B00011000,
  B00011000,
  B00011000
};

const byte PATTERN_BACK[8] = {
  B00011000,
  B00011000,
  B00011000,
  B00011000,
  B11111111,
  B01111110,
  B00111100,
  B00011000
};

const byte PATTERN_DOWN[8] = {
  B00000000,
  B00011000,
  B00111100,
  B00111100,
  B00011000,
  B00000000,
  B00000000,
  B00000000
};

const byte PATTERN_IDLE[8] = {
  B00111100,
  B01000010,
  B10100101,
  B10000001,
  B10100101,
  B10011001,
  B01000010,
  B00111100
};

void updateLED(int8_t dir, uint8_t mag) {
  if (!ledEnabled) return;

  // Set intensity based on magnitude (mag 1-5 -> intensity 3-15)
  uint8_t intensity = mag > 0 ? 3 + (mag * 2) : 8;
  lc.setIntensity(0, intensity);

  // Select pattern
  const byte* pattern;
  switch (dir) {
    case LEFT: pattern = PATTERN_LEFT; break;
    case RIGHT: pattern = PATTERN_RIGHT; break;
    case FORWARD: pattern = PATTERN_FORWARD; break;
    case BACK: pattern = PATTERN_BACK; break;
    case DOWN: pattern = PATTERN_DOWN; break;
    default: pattern = PATTERN_IDLE; break;
  }

  // Display pattern with magnitude indicator and button indicators
  for (int row = 0; row < 7; row++) {
    byte rowData = pattern[row];

    // Add button indicators in top corners (row 0)
    if (row == 0) {
      if (btn1State) rowData |= B00000001;  // BTN1 = right corner (bit 0)
      if (btn2State) rowData |= B10000000;  // BTN2 = left corner (bit 7)
    }

    lc.setRow(0, row, rowData);
  }

  // Add magnitude indicator on bottom row (dots expand from center)
  if (dir != IDLE && mag > 0) {
    static const byte magBars[] = {B00000000, B00011000, B00111100, B01111110, B11111111, B11111111};
    lc.setRow(0, 7, magBars[mag]);
  } else {
    byte bottomRow = pattern[7];
    // Add button indicators even on idle bottom row if space available
    if (dir == IDLE) {
      if (btn1State) bottomRow |= B00000001;  // BTN1 indicator
      if (btn2State) bottomRow |= B10000000;  // BTN2 indicator
    }
    lc.setRow(0, 7, bottomRow);
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

  // Only act on change
  if (dir != lastDir || mag != lastMag) {
    // Send MIDI CC
    if (midiEnabled) {
      bool returningToIdle = (dir == IDLE && mag == 0 && lastMag > 0 && lastDir != IDLE);
      if (returningToIdle) {
        // Send intermediate steps when jumping to idle for smooth release
        uint8_t cc = dirToCC(lastDir);
        for (int8_t m = lastMag - 1; m >= 1; m--) {
          sendMidiCC(cc, magToValue(m));
        }
        sendMidiCC(cc, 0);
      } else {
        if (dir != IDLE) {
          sendMidiCC(dirToCC(dir), magToValue(mag));
        }
        if (lastDir != IDLE && lastDir != dir) {
          sendMidiCC(dirToCC(lastDir), 0);
        }
      }
    }

    // Update LED display
    updateLED(dir, mag);

    // Logging
    if (logEnabled) {
      // Show intermediate step when returning to idle
      if (dir == IDLE && mag == 0 && lastMag > 0 && lastDir != IDLE) {
        Serial.print(F("EVENT: "));
        Serial.print(dirStr(lastDir));
        Serial.println(F(" 1"));
      } else if (mag > 0) {
        Serial.print(F("EVENT: "));
        Serial.print(dirStr(dir));
        Serial.print(F(" "));
        Serial.println(mag);
      }
    }

    lastDir = dir;
    lastMag = mag;
  }
}

void handleButtonToggle(bool& state, uint8_t note, const __FlashStringHelper* name) {
  state = !state;
  if (logEnabled) {
    Serial.print(name);
    Serial.println(state ? F(" ON") : F(" OFF"));
  }
  if (midiEnabled) {
    if (state) sendMidiNoteOn(note);
    else sendMidiNoteOff(note);
  }
  updateLED(lastDir, lastMag);
}

void checkButtons() {
  static bool last1 = HIGH, last2 = HIGH;
  bool btn1 = digitalRead(BTN_1);
  bool btn2 = digitalRead(BTN_2);

  if (btn1 == LOW && last1 == HIGH) handleButtonToggle(btn1State, MIDI_NOTE_BTN1, F("BTN1"));
  if (btn2 == LOW && last2 == HIGH) handleButtonToggle(btn2State, MIDI_NOTE_BTN2, F("BTN2"));

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
    case 'm': case 'M':
      midiEnabled = !midiEnabled;
      Serial.print(F("MIDI: "));
      Serial.println(midiEnabled ? F("ON") : F("OFF"));
      break;
    case 'd': case 'D':
      ledEnabled = !ledEnabled;
      Serial.print(F("LED: "));
      Serial.println(ledEnabled ? F("ON") : F("OFF"));
      if (!ledEnabled) lc.clearDisplay(0);
      break;
    case 'r': case 'R':
      Serial.println(F("Rebooting..."));
      delay(100);
      wdt_enable(WDTO_15MS);
      while (1) {}
    case 'h': case 'H': case '?':
      Serial.println(F("C=calibrate D=display E=EEPROM L=log M=midi R=reboot"));
      break;
  }
}

void setup() {
  wdt_disable();
  Serial.begin(9600);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);

  Serial.println(F("TLV493D MIDI Controller"));
  Serial.println(F("Initializing LED matrix..."));

  // Initialize LED matrix
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  // Test pattern - all LEDs on
  Serial.println(F("LED test: all ON"));
  for (int row = 0; row < 8; row++) {
    lc.setRow(0, row, B11111111);
  }
  delay(1000);
  lc.clearDisplay(0);
  Serial.println(F("LED test: cleared"));

  // Initialize buttons to OFF state
  sendMidiNoteOff(MIDI_NOTE_BTN1);
  sendMidiNoteOff(MIDI_NOTE_BTN2);

  if (!sensor.begin()) {
    Serial.println(F("Sensor init failed!"));
    while (1) delay(1000);
  }

  calibrateBaseline();

  if (loadFromEEPROM()) {
    printCalibData(F("EEPROM: "));
    Serial.println(F("Ready! (C=recalibrate)"));
  } else {
    Serial.println(F("No EEPROM data, calibrating..."));
    calibrateDirections();
    Serial.println(F("Ready!"));
  }

  // Show idle pattern (smiley)
  Serial.println(F("Showing idle pattern"));
  updateLED(IDLE, 5);
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
    // Process event if movement detected OR returning to idle from active position
    if (maxDelta >= NOISE_MARGIN || lastDir != IDLE) {
      processEvent(dX, dY, dZ);
    }
  }
}
