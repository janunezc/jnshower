#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "rgb_lcd.h"

#define pin_led           13
#define pin_buzzer         2
#define pin_button_B       4
#define pin_button_A       5
#define pin_SSR_01         8
#define pin_SSR_02         9
#define pin_FAN           10

#define BUTTON_DELAY_MS    500
#define HEATING_TIME_MS    65000
#define SHOWER_TIME_MS     240000
#define MICRO_PAUSE_INI_MS 160000
#define MICRO_PAUSE_DUR_MS 5000

#define PAUSE_TIME_MS      120000
#define ARMED_TIME_MS      3600000 /*1hr*/


rgb_lcd lcd_r;
Adafruit_ADS1115 ads;

const float AMP_FACTOR =    0.0009090909;
const int16_t AMP_DIVIDER = 2500;

int16_t amps_0_1;
int16_t amps_2_3;

enum armStates {
  ST_DISARMED,
  ST_ARMED,
  ST_HEATING,
  ST_SHOWER,
  ST_PAUSE
};

armStates myState = ST_ARMED;

void setup()
{
  Serial.begin(9600);
  Serial.println("SETUP");

  setup_adc();
  setup_pins();
  setup_lcd();

  Serial.println("SETUP COMPLETE!");
}

void loop()
{
  for (int i = 1000; i <= 9999; i++) { //HEARTBEAT
    readInputs();
    stateProcessor();
    lcdOutput(i);
  }
}

/*SETUP FUNCTIONS*/
void setup_adc() {
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
  ads.begin(0x48);
}

void setup_pins() {
  pinMode(pin_button_B, INPUT_PULLUP);
  pinMode(pin_button_A, INPUT_PULLUP );

  pinMode(pin_buzzer, OUTPUT);   digitalWrite(pin_buzzer, LOW);
  pinMode(pin_led, OUTPUT);    digitalWrite(pin_led, LOW);
  pinMode(pin_SSR_01, OUTPUT);   digitalWrite(pin_SSR_01, LOW);
  pinMode(pin_SSR_02, OUTPUT);  digitalWrite(pin_SSR_02, LOW);
  pinMode(pin_FAN, OUTPUT); digitalWrite(pin_FAN, LOW);
}

void setup_lcd() {
  lcd_r.begin(16/*COLS*/, 2/*ROWS*/);
  lcd_r.setRGB(128, 255, 128);
  lcd_r.setCursor(0, 1);
  lcd_r.print("WELCOME! S-SHOWER");
}

/*LOOP FUNCTIONS*/
void readInputs() {
  amps_0_1 = readAmpsMax_01();
  amps_2_3 = readAmpsAverage_23();
  Serial.println(amps_0_1);

  buttonProcessor();
}

void lcdOutput(int i) {
  activeSignalLCD(i);
  statusToLCD();
}

int16_t readDiffOneShot() {
  int16_t measure = ads.readADC_Differential_0_1();
  int16_t absmeasure = abs(measure);
  return absmeasure;
}

int16_t readMaxDiffSampled() {
  int16_t maxMeasure = 0;
  for (int i = 0; i < 10; i++) {
    int16_t measure = ads.readADC_Differential_0_1();
    int16_t absmeasure = abs(measure);

    if (absmeasure > maxMeasure) {
      maxMeasure = absmeasure;
    }
  }

  return maxMeasure;
}
int16_t readAmpsMax_01() {
  return readMaxDiffSampled() / AMP_DIVIDER;
}
int16_t readAmpsAverage_01() {
  int32_t acc = 0;
  int16_t diff;

  for (int i = 0; i < 10; i++) {
    diff = ads.readADC_Differential_0_1();
    diff = abs(diff);
    acc += diff;
  }

  return (acc / 10) * AMP_FACTOR;
}

int16_t readAmpsAverage_23() {
  int32_t acc = 0;
  int16_t diff;

  for (int i = 0; i < 10; i++) {
    diff = ads.readADC_Differential_2_3();
    diff = abs(diff);
    acc += diff;
  }

  return (acc / 10) * AMP_FACTOR;
}
void stateProcessor() {
  switch (myState) {
    case ST_DISARMED:
      handleState_DISARMED();
      break;
    case ST_ARMED:
      handleState_ARMED();
      break;
    case ST_HEATING:
      handleState_HEATING();
      break;
    case ST_SHOWER:
      handleState_SHOWER();
      break;
    case ST_PAUSE:
      handleState_PAUSE();
  }
}

//JUST WAIT TO BE ARMED
void handleState_DISARMED() {
  digitalWrite(pin_SSR_01, LOW);
  digitalWrite(pin_SSR_02, LOW);
  digitalWrite(pin_FAN, LOW);
}

//JUST WAIT TO DETECT INITIATION OF SHOWERING CYCLE
unsigned long endOfTimeInArmed = 0;
void handleState_ARMED() {
  digitalWrite(pin_SSR_01, HIGH);
  digitalWrite(pin_SSR_02, HIGH);
  digitalWrite(pin_FAN, LOW);

  if (amps_0_1 > 5 || amps_2_3 > 5) {
    switchToState(ST_HEATING);
  }
}

//STAY HEATING FOR 1 MINUTE THEN SWITCH TO SHOWER AUTOMATICALLY
unsigned long endOfTimeInHeating = 0;
unsigned long startOfMicroPause = 0;
unsigned long endOfMicroPause = 0;

void handleState_HEATING() {
  digitalWrite(pin_SSR_01, HIGH);
  digitalWrite(pin_SSR_02, HIGH);
  digitalWrite(pin_FAN, HIGH);

  if (millis() > startOfMicroPause && millis() < endOfMicroPause) {
    digitalWrite(pin_SSR_01, LOW);
    digitalWrite(pin_SSR_02, LOW);
  } else {
    digitalWrite(pin_SSR_01, HIGH);
    digitalWrite(pin_SSR_02, HIGH);
  }

  if (millis() > endOfTimeInHeating) {
    switchToState(ST_SHOWER);
  }
}

//STAY SHOWER FOR 5 MINUTES. THEN SWITCH TO PAUSE AUTOMATICALLY
unsigned long endOfTimeInShower = 0;
void handleState_SHOWER() {
  digitalWrite(pin_SSR_01, HIGH);
  digitalWrite(pin_SSR_02, HIGH);
  digitalWrite(pin_FAN, HIGH);

  if (millis() > endOfTimeInShower) {
    switchToState(ST_PAUSE);
  }
}

//TURN OFF HEATER AND STAY IN PAUSE FOR 1 MINUTE. THEN SWITCH TO ARMED.
unsigned long endOfTimeInPause = 0;
void handleState_PAUSE() {
  digitalWrite(pin_FAN, HIGH);
  digitalWrite(pin_SSR_01, LOW);
  digitalWrite(pin_SSR_02, LOW);
  if (millis() > endOfTimeInPause) {
    switchToState(ST_ARMED);
  }

  buzz(200, 100, 1);
}

void switchToState(armStates targetState) {
  switch (targetState) {
    case ST_DISARMED:
      Serial.println("SWITCHING TO DISARMED!");
      myState = targetState;
      buzz(10, 200, 1);
      break;
    case ST_ARMED:
      Serial.println("SWITCHING TO ARMED!");
      endOfTimeInArmed = millis() + ARMED_TIME_MS;
      myState = targetState;
      buzz(10, 200, 1);
      break;
    case ST_HEATING:
      Serial.println("SWITCHING TO HEATING!");
      endOfTimeInHeating = millis() + HEATING_TIME_MS;
      myState = targetState;
      buzz(100, 0, 1);
      break;
    case ST_SHOWER:
      Serial.println("SWITCHING TO SHOWER!");
      endOfTimeInShower = millis() + SHOWER_TIME_MS;
      startOfMicroPause = millis() + MICRO_PAUSE_INI_MS;
      endOfMicroPause = millis() + MICRO_PAUSE_INI_MS + MICRO_PAUSE_DUR_MS;
      myState = targetState;
      buzz(200, 50, 2);
      break;
    case ST_PAUSE:
      Serial.println("SWITCHING TO PAUSE!");
      endOfTimeInPause = millis() + PAUSE_TIME_MS;
      myState = targetState;
      break;
  }
}

unsigned long buttonActiveMs = millis();
void buttonProcessor() {
  if (millis() > buttonActiveMs || buttonActiveMs < 0) {
    int button_right = digitalRead(4);
    int button_left = digitalRead(5);

    if (button_left == 0/*ARM*/ && myState == ST_DISARMED) {
      Serial.println("ARM BUTTON! SWITCH TO ARMED!");
      buttonActiveMs = (unsigned long) millis() + BUTTON_DELAY_MS;
      switchToState(ST_ARMED);
    } else if (button_left == 0/*ARM*/ && myState == ST_ARMED /*ARMED*/) { //THEN FAKE DETECT
      Serial.println("ARM BUTTON FAKE SWITCH TO HEATING!");
      switchToState(ST_HEATING);
      buttonActiveMs = (unsigned long) millis() + BUTTON_DELAY_MS;
    }

    if (button_right == 0 /*DISARM*/ && myState != ST_DISARMED ) {
      Serial.println("DISARM BUTTON! SWITCH TO DISARMED!");
      buttonActiveMs = (unsigned long) millis() + BUTTON_DELAY_MS;
      switchToState(ST_DISARMED);
    }
  }
}

void buzz(int buzzTimeMs, int delayTimeMs, int repetitions) {
  for (int i = 0; i < repetitions; i++) {
    digitalWrite(pin_buzzer, HIGH);
    delay(buzzTimeMs);
    digitalWrite(pin_buzzer, LOW);
    delay(delayTimeMs);
  }
}

void activeSignalLCD(int i) {
  lcd_r.setCursor(0, 1);

  switch (myState) {
    case ST_DISARMED:
      lcd_r.print("DISARMED (");
      break;
    case ST_ARMED:
      lcd_r.print("ARMED (");
      break;
    case ST_HEATING:
      lcd_r.print("HEATING (");
      break;
    case ST_SHOWER:
      lcd_r.print("SHOWER (");
      break;
    case ST_PAUSE:
      lcd_r.print("PAUSE (");
      break;
  }


  lcd_r.print(i);
  lcd_r.print(")  ");
}

void statusToLCD() {
  unsigned long remainingMinutes = 0;
  unsigned long remainingSeconds = 0;
  String messageForLCD = "";
  String ampsMessage = "";

  ampsMessage += String(amps_0_1) + "A/";
  ampsMessage += String(amps_2_3) + "A ";
  switch (myState) {
    case ST_DISARMED:
      messageInLCD("OFF " + ampsMessage + "-", 0, 0, 255, 0);
      break;
    case ST_ARMED:
      messageForLCD = "IDLE " + ampsMessage + "-" + String (millis()) + "ms";
      messageInLCD (messageForLCD, 0, 255, 255, 0);
      break;

    case ST_HEATING:
      remainingMinutes = (unsigned long) (endOfTimeInHeating - millis()) / 1000 / 60;
      remainingSeconds = (unsigned long) (endOfTimeInHeating - millis()) / 1000;
      if (remainingMinutes >= 1) {
        messageForLCD = String(remainingMinutes + 1);
        messageForLCD = "HEAT " + ampsMessage + "-" + messageForLCD + "m'";
      } else {
        messageForLCD = String(remainingSeconds + 1);
        messageForLCD = "HEAT " + ampsMessage + "-" + messageForLCD + "s'";
      }

      messageInLCD (messageForLCD, 0, 255, 0, 0);
      break;
    case ST_SHOWER:
      remainingMinutes = (unsigned long) (endOfTimeInShower - millis()) / 1000 / 60;
      remainingSeconds = (unsigned long) (endOfTimeInShower - millis()) / 1000;
      if (remainingMinutes >= 1) {
        messageForLCD = String(remainingMinutes + 1);
        messageForLCD = "SHWR " + ampsMessage + "-" + messageForLCD + "m'";
      } else {
        messageForLCD = String(remainingSeconds + 1);
        messageForLCD = "SHWR " + ampsMessage + "-" + messageForLCD + "s'";
      }

      if (millis() > startOfMicroPause && millis() < endOfMicroPause) {
        messageForLCD = "P: " + messageForLCD;
      }

      messageInLCD (messageForLCD, 0, 255, 0, 128);
      break;
    case ST_PAUSE:
      remainingMinutes = (unsigned long) (endOfTimeInPause - millis()) / 1000 / 60;
      remainingSeconds = (unsigned long) (endOfTimeInPause - millis()) / 1000;
      if (remainingMinutes >= 1) {
        messageForLCD = String(remainingMinutes + 1);
        messageForLCD = "STOP " + ampsMessage + "-" + messageForLCD + "m'";
      } else {
        messageForLCD = String(remainingSeconds + 1);
        messageForLCD = "STOP " + ampsMessage + "-" + messageForLCD + "s'+";
      }

      messageInLCD (messageForLCD, 0, 0, 0, 255);
      break;
  }
}

void messageInLCD (String message, int row, int R, int G, int B) {
  lcd_r.setCursor(0, row);
  lcd_r.setRGB(R, G, B);
  lcd_r.print(message);
  lcd_r.print("                  ");
}

void blinkEffect(int times, int duration) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(pin_led, HIGH);
    delay(duration);
    digitalWrite(pin_led, LOW);
    delay(duration);
  }
}
