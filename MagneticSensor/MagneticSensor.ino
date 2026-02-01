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
uint8_t ledBrightness = 8;  // LED matrix brightness (0-15)
bool btn1State = false;  // Toggle state for button 1
bool btn2State = false;  // Toggle state for button 2
int8_t lastDir = 0;  // 1-5 for directions, 0 for idle (for display only)
uint8_t lastMag = 0;  // for display only
// Track magnitude for each direction independently
uint8_t dirMag[6] = {0, 0, 0, 0, 0, 0};  // [IDLE, LEFT, RIGHT, BACK, FORWARD, DOWN]

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

// Forward declaration
const byte* getPatternForDir(int8_t dir);

void calibrateDirection(const __FlashStringHelper* name, double* maxVal, int axis, bool negative, int8_t dirEnum) {
  Serial.print(F("Move to "));
  Serial.print(name);
  Serial.println(F(" max, hold 2s..."));

  // Show direction arrow animation - pattern will be retrieved later
  const byte* pattern = NULL;

  // Flash direction arrow while waiting for movement
  int flashCount = 0;
  while (true) {
    // Get pattern (done here after patterns are defined)
    if (pattern == NULL) {
      pattern = getPatternForDir(dirEnum);
    }

    // Show arrow (flashing)
    if (ledEnabled) {
      if ((flashCount / 5) % 2 == 0) {  // Flash every 0.5s
        for (int row = 0; row < 8; row++) {
          lc.setRow(0, row, pattern[row]);
        }
      } else {
        lc.clearDisplay(0);
      }
    }

    double delta = getAxisDelta(axis);
    if (negative) delta = -delta;
    if (delta > NOISE_MARGIN) {
      Serial.println(F("Movement detected, hold..."));
      break;
    }
    delay(50);
    flashCount++;
  }

  // Keep arrow solid during sampling
  if (ledEnabled) {
    for (int row = 0; row < 8; row++) {
      lc.setRow(0, row, pattern[row]);
    }
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

  // Show checkmark on success
  if (ledEnabled) {
    byte checkmark[8] = {
      B00000000,
      B00000001,
      B00000011,
      B10000110,
      B11001100,
      B01111000,
      B00110000,
      B00000000
    };
    for (int row = 0; row < 8; row++) {
      lc.setRow(0, row, checkmark[row]);
    }
    delay(500);
    lc.clearDisplay(0);
    delay(300);
  }
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

  // Spinning animation during calibration
  byte spinFrames[8][8] = {
    {B00011000, B00111100, B01111110, B11111111, B00000000, B00000000, B00000000, B00000000},
    {B00000000, B00011000, B00111100, B01111110, B11111111, B00000000, B00000000, B00000000},
    {B00000000, B00000000, B00011000, B00111100, B01111110, B11111111, B00000000, B00000000},
    {B00000000, B00000000, B00000000, B00011000, B00111100, B01111110, B11111111, B00000000},
    {B00000000, B00000000, B00000000, B00000000, B00011000, B00111100, B01111110, B11111111},
    {B00000000, B00000000, B00000000, B00000000, B11111111, B01111110, B00111100, B00011000},
    {B00000000, B00000000, B00000000, B11111111, B01111110, B00111100, B00011000, B00000000},
    {B00000000, B00000000, B11111111, B01111110, B00111100, B00011000, B00000000, B00000000}
  };

  double sumX = 0, sumY = 0, sumZ = 0;
  int count = 0;

  for (int i = 0; i < 20; i++) {
    // Show animation frame
    if (ledEnabled) {
      int frame = i % 8;
      for (int row = 0; row < 8; row++) {
        lc.setRow(0, row, spinFrames[frame][row]);
      }
    }

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

    // Show checkmark on success
    if (ledEnabled) {
      byte checkmark[8] = {
        B00000000,
        B00000001,
        B00000011,
        B10000110,
        B11001100,
        B01111000,
        B00110000,
        B00000000
      };
      for (int row = 0; row < 8; row++) {
        lc.setRow(0, row, checkmark[row]);
      }
      delay(800);
      lc.clearDisplay(0);
    }
  } else {
    Serial.println(F("Baseline failed!"));

    // Show X on failure
    if (ledEnabled) {
      byte crossX[8] = {
        B10000001,
        B01000010,
        B00100100,
        B00011000,
        B00011000,
        B00100100,
        B01000010,
        B10000001
      };
      for (int row = 0; row < 8; row++) {
        lc.setRow(0, row, crossX[row]);
      }
      delay(800);
      lc.clearDisplay(0);
    }
  }
}

void calibrateDirections() {
  Serial.println(F("\n=== DIRECTION CALIBRATION ==="));

  // Show "CAL" text animation
  if (ledEnabled) {
    byte calText[8] = {
      B11111110,
      B11000000,
      B11000000,
      B11000000,
      B11000000,
      B11000000,
      B11111110,
      B00000000
    };
    for (int flash = 0; flash < 4; flash++) {
      for (int row = 0; row < 8; row++) {
        lc.setRow(0, row, calText[row]);
      }
      delay(200);
      lc.clearDisplay(0);
      delay(200);
    }
  }

  calibrateDirection(F("LEFT"), &calib.maxLeft, 0, false, LEFT);
  calibrateDirection(F("RIGHT"), &calib.maxRight, 0, true, RIGHT);
  calibrateDirection(F("BACK"), &calib.maxBack, 1, false, BACK);
  calibrateDirection(F("FORWARD"), &calib.maxForward, 1, true, FORWARD);
  calibrateDirection(F("DOWN"), &calib.maxDown, 2, true, DOWN);
  saveToEEPROM();

  // Show "OK" on complete
  if (ledEnabled) {
    byte okText[8] = {
      B01111110,
      B11000011,
      B11000011,
      B11000011,
      B11000011,
      B11000011,
      B01111110,
      B00000000
    };
    for (int row = 0; row < 8; row++) {
      lc.setRow(0, row, okText[row]);
    }
    delay(1000);
    lc.clearDisplay(0);
  }

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
  // Don't flush here - will flush after all messages sent
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

const byte* getPatternForDir(int8_t dir) {
  switch (dir) {
    case LEFT: return PATTERN_LEFT;
    case RIGHT: return PATTERN_RIGHT;
    case FORWARD: return PATTERN_FORWARD;
    case BACK: return PATTERN_BACK;
    case DOWN: return PATTERN_DOWN;
    default: return PATTERN_IDLE;
  }
}

void updateLED(int8_t dir, uint8_t mag) {
  if (!ledEnabled) return;

  // Use brightness 0 for IDLE, ledBrightness for direction modes
  uint8_t brightness = (dir == IDLE) ? 0 : ledBrightness;
  lc.setIntensity(0, brightness);

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

void sendAllMidiState() {
  if (!midiEnabled) return;

  // Send all 5 direction CC values as a burst
  sendMidiCC(MIDI_CC_LEFT, magToValue(dirMag[LEFT]));
  sendMidiCC(MIDI_CC_RIGHT, magToValue(dirMag[RIGHT]));
  sendMidiCC(MIDI_CC_BACK, magToValue(dirMag[BACK]));
  sendMidiCC(MIDI_CC_FORWARD, magToValue(dirMag[FORWARD]));
  sendMidiCC(MIDI_CC_DOWN, magToValue(dirMag[DOWN]));

  // Send button states as notes
  if (btn1State) {
    midiEventPacket_t noteOn = {0x09, 0x90 | MIDI_CHANNEL, MIDI_NOTE_BTN1, MIDI_VELOCITY};
    MidiUSB.sendMIDI(noteOn);
  } else {
    midiEventPacket_t noteOff = {0x08, 0x80 | MIDI_CHANNEL, MIDI_NOTE_BTN1, 0};
    MidiUSB.sendMIDI(noteOff);
  }

  if (btn2State) {
    midiEventPacket_t noteOn = {0x09, 0x90 | MIDI_CHANNEL, MIDI_NOTE_BTN2, MIDI_VELOCITY};
    MidiUSB.sendMIDI(noteOn);
  } else {
    midiEventPacket_t noteOff = {0x08, 0x80 | MIDI_CHANNEL, MIDI_NOTE_BTN2, 0};
    MidiUSB.sendMIDI(noteOff);
  }

  // Flush all messages at once
  MidiUSB.flush();
}

void processEvent(double dX, double dY, double dZ) {
  double absX = abs(dX);
  double absY = abs(dY);
  double absZ = dZ < 0 ? abs(dZ) : 0;  // Only DOWN

  // Calculate magnitude for each direction independently
  uint8_t newMag[6];
  newMag[IDLE] = 0;
  newMag[LEFT] = (dX > 0) ? getMagnitude(absX, calib.maxLeft) : 0;
  newMag[RIGHT] = (dX < 0) ? getMagnitude(absX, calib.maxRight) : 0;
  newMag[BACK] = (dY > 0) ? getMagnitude(absY, calib.maxBack) : 0;
  newMag[FORWARD] = (dY < 0) ? getMagnitude(absY, calib.maxForward) : 0;
  newMag[DOWN] = getMagnitude(absZ, calib.maxDown);

  // Check if anything changed
  bool changed = false;
  int8_t dominantDir = IDLE;
  uint8_t dominantMag = 0;

  for (int8_t d = LEFT; d <= DOWN; d++) {
    if (newMag[d] != dirMag[d]) {
      changed = true;

      // Logging
      if (logEnabled && (newMag[d] > 0 || dirMag[d] > 0)) {
        Serial.print(F("EVENT: "));
        Serial.print(dirStr(d));
        Serial.print(F(" "));
        Serial.println(newMag[d]);
      }

      dirMag[d] = newMag[d];
    }

    // Track dominant direction for display
    if (newMag[d] > dominantMag) {
      dominantMag = newMag[d];
      dominantDir = d;
    }
  }

  // If anything changed, send complete MIDI state as burst
  if (changed) {
    sendAllMidiState();

    // Update LED display with dominant direction
    if (dominantDir != lastDir || dominantMag != lastMag) {
      updateLED(dominantDir, dominantMag);
      lastDir = dominantDir;
      lastMag = dominantMag;
    }
  }
}

void handleButtonToggle(bool& state, uint8_t note, const __FlashStringHelper* name) {
  state = !state;
  if (logEnabled) {
    Serial.print(name);
    Serial.println(state ? F(" ON") : F(" OFF"));
  }
  // Send complete MIDI state as burst (includes button states)
  sendAllMidiState();
  updateLED(lastDir, lastMag);
}

void checkButtons() {
  static bool last1 = HIGH, last2 = HIGH;
  static unsigned long bothPressedStart = 0;
  static bool calibTriggered = false;

  bool btn1 = digitalRead(BTN_1);
  bool btn2 = digitalRead(BTN_2);

  // Check if both buttons pressed
  if (btn1 == LOW && btn2 == LOW) {
    if (bothPressedStart == 0) {
      bothPressedStart = millis();
      calibTriggered = false;
    }

    // Check if held for 5 seconds
    unsigned long holdTime = millis() - bothPressedStart;
    if (holdTime >= 5000 && !calibTriggered) {
      Serial.println(F("Both buttons held - starting calibration!"));

      // Show progress animation during hold
      if (ledEnabled && holdTime < 5500) {
        byte progressBar[8] = {
          B11111111,
          B11111111,
          B00000000,
          B00000000,
          B00000000,
          B00000000,
          B11111111,
          B11111111
        };
        for (int row = 0; row < 8; row++) {
          lc.setRow(0, row, progressBar[row]);
        }
      }

      calibTriggered = true;
      calibrateBaseline();
      calibrateDirections();

      // Return to idle display
      updateLED(IDLE, 0);
    }
  } else {
    // Buttons released
    if (bothPressedStart > 0 && !calibTriggered) {
      // Released before 5 seconds - process as normal button presses
      if (btn1 == HIGH && last1 == LOW && btn2 == LOW) {
        // BTN1 released first
        handleButtonToggle(btn1State, MIDI_NOTE_BTN1, F("BTN1"));
      }
      if (btn2 == HIGH && last2 == LOW && btn1 == LOW) {
        // BTN2 released first
        handleButtonToggle(btn2State, MIDI_NOTE_BTN2, F("BTN2"));
      }
    }
    bothPressedStart = 0;
    calibTriggered = false;

    // Normal single button press detection
    if (!calibTriggered) {
      if (btn1 == LOW && last1 == HIGH && btn2 == HIGH) {
        handleButtonToggle(btn1State, MIDI_NOTE_BTN1, F("BTN1"));
      }
      if (btn2 == LOW && last2 == HIGH && btn1 == HIGH) {
        handleButtonToggle(btn2State, MIDI_NOTE_BTN2, F("BTN2"));
      }
    }
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
    case 'b': case 'B':
      ledBrightness = (ledBrightness + 3) % 16;  // Cycle: 0, 3, 6, 9, 12, 15, 0...
      Serial.print(F("Brightness: "));
      Serial.println(ledBrightness);

      // Show brightness preview pattern
      if (ledEnabled) {
        lc.setIntensity(0, ledBrightness);
        // Show a brightness preview pattern - vertical bar with horizontal line
        byte brightnessPattern[8] = {
          B00011000,
          B00011000,
          B00011000,
          B11111111,
          B00011000,
          B00011000,
          B00011000,
          B00000000
        };
        for (int row = 0; row < 8; row++) {
          lc.setRow(0, row, brightnessPattern[row]);
        }
        delay(1000);  // Show preview for 1 second
      }

      updateLED(lastDir, lastMag);  // Return to normal display
      break;
    case 'r': case 'R':
      Serial.println(F("Rebooting..."));
      delay(100);
      wdt_enable(WDTO_15MS);
      while (1) {}
    case 'h': case 'H': case '?':
      Serial.println(F("B=brightness C=calibrate D=display E=EEPROM L=log M=midi R=reboot"));
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

  // Show idle pattern
  updateLED(IDLE, 0);

  // Send initial MIDI state (all zeros + button states)
  sendAllMidiState();
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
