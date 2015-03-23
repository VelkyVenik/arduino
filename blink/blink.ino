
#define ledsCount 5
#define playLeds(n) _playLeds (n, sizeof(n)/sizeof(int)/ledsCount)

const int firstLed = 2;

const int ledsOn[ledsCount] = {1, 1, 1, 1, 1};
const int ledsOff[ledsCount] = {0, 0, 0, 0, 0};

const int leds1[][ledsCount] = {
  {1, 0, 0, 0, 0},
  {0, 1, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {0, 1, 0, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {0, 1, 0, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 0, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {0, 1, 0, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 0, 1, 0},
  {0, 0, 0, 0, 1},
  {0, 0, 0, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0}
};

const int leds2[][ledsCount] = {
  {0, 1, 1, 1, 1},
  {1, 0, 1, 1, 1},
  {1, 1, 0, 1, 1},
  {1, 1, 1, 0, 1},
  {1, 1, 1, 1, 0},
  {1, 1, 1, 0, 1},
  {1, 1, 0, 1, 1},
  {1, 0, 1, 1, 1}
};

const int leds3[][ledsCount] = {
  {1, 0, 0, 0, 0},
  {0, 1, 0, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 0, 1, 0},
  {0, 0, 0, 0, 1},
  {0, 0, 0, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0}
};

const int leds4[][ledsCount] = {
  {1, 0, 0, 0, 0},
  {1, 1, 0, 0, 0},
  {0, 1, 1, 0, 0},
  {0, 0, 1, 1, 0},
  {0, 0, 0, 1, 1},
  {0, 0, 0, 0, 1},
  {0, 0, 0, 1, 1},
  {0, 0, 1, 1, 0},
  {0, 1, 1, 0, 0},
  {1, 1, 0, 0, 0}
};

const int leds5[][ledsCount] = {
  {1, 1, 1, 0, 0},
  {0, 1, 1, 1, 0},
  {0, 0, 1, 1, 1},
  {0, 1, 1, 1, 0},
};


void setup() {
  for (int i = firstLed; i < firstLed + ledsCount; i++) {
    pinMode(i, OUTPUT);
  }
  
  pinMode(7, INPUT_PULLUP);
  
  Serial.begin(9600);
}

boolean checkButton (void) {
  return digitalRead(7) == 0; 
}

void setLed(int l, int v) {
  digitalWrite (l, v == 1 ? HIGH : LOW);
}

void setLeds(const int leds[]) {
  for (int i = 0; i < ledsCount; i++) {
    setLed(firstLed + i, leds[i]);
  }

}

void _playLeds(const int leds[][5], int len) {
  for (int i = 0; i < len; i++) {
    setLeds (leds[i]);
    if (checkButton()) return;
    delay (100);
  }
  
}

int s = 0; 

void loop() {
  int next = 0;
  
  while (digitalRead(7) == 0)
  {
    next = 1;
    setLeds(ledsOn);
    delay (100);
    setLeds(ledsOff);
    delay (100);
  }
  
  if (next == 1) {
    s++;
    if (s > 5) s = 0;
    next = 0;
  }

    
  switch (s) {
    case 0:
      playLeds (leds1);
    break;
    case 1:
      playLeds (leds2);
    break;
    case 2:
      playLeds (leds3);
    break;
    case 3:
      playLeds (leds4);
    break;
    case 4:
      playLeds (leds5);
    break;
  }
}
