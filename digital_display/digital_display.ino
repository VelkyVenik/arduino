#include <EEPROM.h>
#include <OneWire.h>

#define pinClock      8
#define pinData       9
#define buttonStart   3
#define buttonMode    2

#define buttonTimeout     300
#define buttonLongPress   2000

volatile unsigned long buttonTime;
volatile int buttonPreviousState = 1;
volatile int buttonState = 0;


volatile bool stopWatch = false;
volatile unsigned int stopWatchStart = 0;
volatile unsigned int stopWatchValue = 0;


/* Mode
 *  0 - stopwatch
 *  1 - thermometer
 *  2 - countdown
 */

#define modeStopWatch     0
#define modeTemperature   1
#define modeCountDown     2

#define modeMax 2

volatile int mode = 0;


#define countDownDays     3
#define countDownHours    13
unsigned int countDownStart;
int countDownHoursSaved = 0;

byte numbersData[]  = {
  B11101101,    // 0
  B00101000,    // 1
  B11001110,    // 2
  B01101110,    // 3
  B00101011,    // 4
  B01100111,    // 5
  B11100111,    // 6
  B00101100,    // 7
  B11101111,    // 8
  B01101111     // 9
};
byte point = B00010000;  // .

byte animationData1[][3] =
{
  {B00000100, B00000000, B00000000},
  {B00000100, B00000100, B00000000},
  {B00000000, B00000100, B00000100},
  {B00000000, B00000000, B00001100},
  {B00000000, B00000000, B00101000},
  {B00000000, B00000000, B01100000},
  {B00000000, B01000000, B01000000},
  {B01000000, B01000000, B00000000},
  {B11000000, B00000000, B00000000},
  {B10000001, B00000000, B00000000},
  {B00000001, B00000000, B00000000},
  {B00000000, B00000000, B00000000},
};

byte animationData2[][3] =
{
  {B00000000, B00000000, B00000000},
  {B10100000, B10100000, B10100000},
  {B10101001, B10101001, B10101001},
  {B10101101, B10101101, B10101101},
  {B11101101, B11101101, B11101101},
};

byte animationData3[][3] =
{
  {B00000000, B00000000, B00000000},
  {B10000000, B00000000, B00000000},
  {B10000001, B00000000, B00000000},
  {B10100001, B00000000, B00000000},
  {B10101001, B00000000, B00000000},
  {B10101001, B10000000, B00000000},
  {B10101001, B10000001, B00000000},
  {B10101001, B10100001, B00000000},
  {B10101001, B10101001, B00000000},
  {B10101001, B10101001, B10000000},
  {B10101001, B10101001, B10000001},
  {B10101001, B10101001, B10100001},
  {B10101001, B10101001, B10101001},
  {B10101101, B10101001, B10101001},
  {B10101101, B10101101, B10101001},
  {B10101101, B10101101, B10101101},
  {B11101101, B10101101, B10101101},
  {B11101101, B11101101, B10101101},
  {B11101101, B11101101, B11101101},
};

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)

void startAnimation()
{
  for (int ii = 0; ii < 2; ii++)
  {
    for (int i = 0; i < 11; i++)
    {
      noInterrupts();
      shiftOut(pinData, pinClock, MSBFIRST, animationData1[i][2]);
      shiftOut(pinData, pinClock, MSBFIRST, animationData1[i][1]);
      shiftOut(pinData, pinClock, MSBFIRST, animationData1[i][0]);
      interrupts();
      delay(80);
    }
  }

  for (int i = 0; i < 19; i++)
  {
    noInterrupts();
    shiftOut(pinData, pinClock, MSBFIRST, animationData3[i][2]);
    shiftOut(pinData, pinClock, MSBFIRST, animationData3[i][1]);
    shiftOut(pinData, pinClock, MSBFIRST, animationData3[i][0]);
    interrupts();
    delay(80);
  }
}

void setup() {
  pinMode(pinClock, OUTPUT);
  pinMode(pinData, OUTPUT);

  pinMode(buttonStart, INPUT_PULLUP);
  pinMode(buttonMode, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(buttonStart), buttonStartChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonMode), buttonModePressed, LOW);

  countDownStart = millis();

  Serial.begin(9600);

  countDownLoad();

  startAnimation();
}


int readTemperature()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;

  ds.reset_search();
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return 0;
  }

  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return 0;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return 0;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.println(" Celsius, ");

  return (int) (celsius * 10);
}

int countDown()
{
  unsigned int now = millis();
  int days, hours, hoursSinceStart;

  hoursSinceStart = (now - countDownStart) / 1000 / 60 / 60;

  hours = countDownHours - hoursSinceStart - countDownHoursSaved;
  days = countDownDays - (hoursSinceStart - countDownHoursSaved) / 24;

  countDownSave();

  return days * 100 + hours;
}

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void countDownSave()
{
  unsigned int now = millis();
  int hours;

  hours = (now - countDownStart) / 1000 / 60 / 60 + countDownHoursSaved;
  EEPROMWriteInt(0, hours);
}

void countDownLoad()
{
  int hours = EEPROMReadInt(0);
  Serial.print("Countdown EEPROM data: ");
  Serial.println(hours);

  countDownHoursSaved = hours;
}

void buttonStartChange ()
{
  int state = digitalRead(buttonStart);
  unsigned long now = millis();

  if (state == buttonPreviousState)
    return;

  Serial.print("Start Button state changed: ");
  Serial.println(state);

  if (state == 1)
  {
    stopWatchStart = now;
    if ((now - buttonTime) < buttonLongPress)
    {
      stopWatch = !stopWatch;
    } else
    {
      stopWatchValue = 0;
    }
  }
  else
    buttonTime = millis();

  buttonPreviousState = state;
}


void buttonModePressed ()
{
  unsigned long now = millis();

  if ((now - buttonTime) < buttonTimeout)
    return;

  mode = mode + 1;
  if (mode > modeMax)
    mode = 0;
  buttonTime = millis();
  return;
}

void displayValue(int v, int p)
{
  noInterrupts();
  shiftOut(pinData, pinClock, MSBFIRST, numbersData[v - v / 10 * 10] | (p == 0)*point);
  shiftOut(pinData, pinClock, MSBFIRST, numbersData[(v - v / 100 * 100) / 10] | (p == 1)*point);
  shiftOut(pinData, pinClock, MSBFIRST, numbersData[v / 100] | (p == 2)*point);
  interrupts();
}

void loop() {
  if (stopWatch)
  {
    stopWatchValue = stopWatchValue + ((millis() - stopWatchStart) / 100);
    stopWatchStart = millis();
    if (stopWatchValue > 999)
    {
      stopWatchValue = 0;
    }
  }

  switch (mode)
  {
    case modeStopWatch:
      displayValue(stopWatchValue, 1);
      break;

    case modeTemperature:
      displayValue(readTemperature(), 1);
      break;

    case modeCountDown:
      displayValue(countDown(), 2);
      break;
  }

  if (stopWatch)
    delay(100);
  else
    delay(500);
}
