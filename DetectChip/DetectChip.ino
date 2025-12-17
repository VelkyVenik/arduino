void setup() {
  Serial.begin(9600);
  delay(2000);

  Serial.println(F("=== Arduino Chip Detection ==="));
  Serial.println();

  // Compile-time detection
  Serial.print(F("Detected: "));
  #if defined(__AVR_ATmega328P__)
    Serial.println(F("ATmega328P"));
  #elif defined(__AVR_ATmega328__)
    Serial.println(F("ATmega328"));
  #elif defined(__AVR_ATmega168P__)
    Serial.println(F("ATmega168P"));
  #elif defined(__AVR_ATmega168__)
    Serial.println(F("ATmega168"));
  #elif defined(__AVR_ATmega88P__)
    Serial.println(F("ATmega88P"));
  #elif defined(__AVR_ATmega88__)
    Serial.println(F("ATmega88"));
  #else
    Serial.println(F("Unknown AVR"));
  #endif

  Serial.println();
  Serial.print(F("F_CPU: "));
  Serial.print(F_CPU / 1000000L);
  Serial.println(F(" MHz"));

  Serial.print(F("SRAM: "));
  Serial.print(RAMEND + 1);
  Serial.println(F(" bytes"));

  Serial.print(F("Flash: "));
  Serial.print((long)FLASHEND + 1);
  Serial.println(F(" bytes"));

  Serial.println();
  Serial.println(F("Detection complete!"));
}

void loop() {
  // Nothing
}
