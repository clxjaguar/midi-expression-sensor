/* Arduino sketch for the midi-expression-sensor
 * https://github.com/clxjaguar/midi-expression-sensor
 * Please select "Tools> Board[...]> Arduino AVR Boards> Arduino Uno" */

#include <EEPROM.h>

const int ledPin       = 2; // PD2
const int segGPin      = 5; // PD5
const int selDigit1Pin = 6; // PD6
const int selDigit0Pin = 7; // PD7
const int sw3Pin      = 16; // PC2
const int sw2Pin      = 17; // PC3
const int sw1Pin      = 18; // PC4
const int analogInPin = A1; // Analog input pin that the potentiometer is attached to

#define delay_ms delay

#define NOTE_ON         0x90
#define NOTE_OFF        0x80
#define MIDI_COMMAND_PITCHWHEEL 0xE0
#define MIDI_COMMAND_CC         0xB0

#define CC_MODULATION  1
#define CC_BREATH      2
#define CC_VOLUME      7
#define CC_EXPRESSION 11

#define MIDI_CH1	0
#define MIDI_CH2	1
#define MIDI_CH16	15

void midi_send(uint8_t type, uint8_t channel, uint8_t key, uint8_t arg) {
  Serial.write(type|(channel&0x0F));
  Serial.write(key & 0x7f);
  Serial.write(arg & 0x7f);
  delay_ms(5);
}

void midi_pitchwheel(uint8_t channel, int16_t offset) {
  uint8_t val1, val2;
  offset+=0x2000;
  if (offset<0) {
    offset=0;
  }
  else if (offset>=0x4000) {
    offset = 0x3fff;
  }
  val1 = (uint8_t)(offset & 0x007f);
  val2 = (uint8_t)(offset>>7 & 0x007f);

  Serial.write(MIDI_COMMAND_PITCHWHEEL|channel);
  Serial.write(val1);
  Serial.write(val2);
  delay_ms(3);
}

void midi_cc(uint8_t channel, uint8_t cc, uint8_t value) {
  Serial.write(MIDI_COMMAND_CC|channel);
  Serial.write(cc & 0x7f);
  Serial.write(value & 0x7f);
  delay_ms(3);
}

// variables used by this program
int16_t ref, pressure, ccVal, ccValOld, pwVal, pwValOld, sign;
uint8_t out[2] = {0xff, 0xff};
uint8_t confChanged = 0;
uint8_t debounce_counter = 0;
uint8_t mode_changed_counter = 0;
int i;
unsigned long currentMillis = 0, previousMillis;

typedef struct t_conf {
  uint8_t midi_channel;
  uint8_t midi_cc;
  uint8_t bend;
  uint8_t filter;
  uint8_t twoflows;
} t_conf;
t_conf conf;

void reset_conf(void) {
  conf.midi_channel = MIDI_CH1;
  conf.midi_cc = CC_EXPRESSION;
  conf.bend = 0;
  conf.filter = 3;
  conf.twoflows = 0;
}

void load_conf(void) {
  int i, d, checksum=0x55;
  uint8_t *p = (uint8_t *)&conf;
  for (i=0; i<sizeof(t_conf); i++) {
    d = EEPROM.read(i);
    p[i] = d;
    checksum^=d;
  }
  if (checksum != EEPROM.read(i)) {
    reset_conf();
    save_conf();
  }
}

void save_conf(void) {
  int i, checksum=0x55;
  uint8_t *p = (uint8_t *)&conf;
  out[1] = out[0] = 0x86;
  for (i=0; i<sizeof(t_conf); i++) {
    EEPROM.write(i, p[i]);
    checksum^=p[i];
  }
  EEPROM.write(i, checksum);
  confChanged = 0;
  delay_ms(1000);
}

// the setup routine runs once at reset
void setup() {
  PORTB = 0x7F;
  pinMode(selDigit0Pin, OUTPUT);
  pinMode(selDigit1Pin, OUTPUT);
  pinMode(sw1Pin, INPUT_PULLUP);
  pinMode(sw2Pin, INPUT_PULLUP);
  pinMode(sw3Pin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(segGPin, OUTPUT);
  DDRB = B00111111;  // pins PB[5..0] as outputs for seven-segments display (F to A)
  Serial.begin(31250);
  digitalWrite(ledPin, HIGH);
  delay_ms(500);
  ref = analogRead(analogInPin);

  cli();                                   // disable all interrupts
  TCCR2A = (1<<WGM21)|(0<<WGM20);          // Mode CTC
  TIMSK2 = (1<<OCIE2A);                    // Local interruption OCIE2A
  TCCR2B = (0<<WGM22)|(1<<CS22)|(1<<CS21); // Frequency 16Mhz/ 256 = 62500
  OCR2A = 250;                             //250*125 = 31250 = 16Mhz/256/2
  sei();                                   // enable interrupts

  // press plus and minus buttons at startup to reset default parameters values
  if (!digitalRead(sw1Pin) && digitalRead(sw2Pin) && !digitalRead(sw3Pin)) {
    reset_conf();
    save_conf();
  }
  else {
    load_conf();
  }

  midi_pitchwheel(conf.midi_channel, 0);
}

const uint8_t nibbleToSegments[] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, 0x88, 0x83, 0xc6, 0xa1, 0x86, 0x8e};
const uint8_t barGraph[][2] = {{0xff, 0xff}, {0xef, 0xff}, {0xcf, 0xff}, {0xce, 0xff}, {0xce, 0xfe}, {0xce, 0xfc}, {0xce, 0xf8}};
const uint8_t segments_CH[] = {0xc6, 0x8b};
const uint8_t segments_CC[] = {0xc6, 0xc6};
const uint8_t segments_bend[] = {0x83, 0xab};
const uint8_t segments_filter[] = {0x8e, 0xe9};
const uint8_t segments_twoflows[] = {0xa4, 0x8e};
int currentDigit = 0;

typedef enum {
  MENU_BARGRAPH = 0,
  MENU_CHANNEL,
  MENU_CONTINUOUS_CONTROL,
  MENU_BEND,
  MENU_FILTER,
  MENU_TWOFLOWS
} t_menu_state;
t_menu_state menu_state;

// timer interrupt service routine
ISR(TIMER2_COMPA_vect) {
  PORTB = 0x7F;
  digitalWrite(segGPin, HIGH);
  digitalWrite(selDigit0Pin, (currentDigit == 0)?LOW:HIGH);
  digitalWrite(selDigit1Pin, (currentDigit == 1)?LOW:HIGH);
  PORTB = out[currentDigit];

  digitalWrite(segGPin, (PORTB & 0x40)?HIGH:LOW);

  currentDigit++;
  if (currentDigit>1) {
    currentDigit = 0;
  }
  if (debounce_counter) { debounce_counter--; }
  if (mode_changed_counter) { mode_changed_counter--; }
}

int paramButtonPressed(void) {
  uint8_t ret;
  if (!digitalRead(sw2Pin)) {
    currentMillis = millis();
    if (!debounce_counter) {
      ret = 1;
      mode_changed_counter = 255;
    }
    else {
      ret = 0;
    }
    debounce_counter = 10;
    return ret;
  }
  return 0;
}

int plusButtonPressed(void) {
  uint8_t ret;
  if (!digitalRead(sw1Pin)) {
    currentMillis = millis();
    if (!debounce_counter) {
      ret = 1;
      previousMillis = currentMillis;
    }
    else {
      ret = 0;
      if (currentMillis - previousMillis > 1000) {
        delay_ms(50);
        ret = 2;
      }
    }
    debounce_counter = 10;
    return ret;
  }
  return 0;
}

int minusButtonPressed(void) {
  uint8_t ret;
  if (!digitalRead(sw3Pin)) {
    currentMillis = millis();
    if (!debounce_counter) {
      ret = 1;
      previousMillis = currentMillis;
    }
    else {
      ret = 0;
      if (currentMillis - previousMillis > 1000) {
        delay_ms(50);
        ret = 2;
      }
    }
    debounce_counter = 10;
    return ret;
  }
  return 0;
}

int rawPressure;
float fPressure;

// the main loop routine runs over and over again forever
void loop() {
  rawPressure = analogRead(analogInPin) - ref;

  switch (conf.filter) {
    case 0:
      pressure = rawPressure;
      fPressure = rawPressure;
      break;

    case 1:
      fPressure = ((float)rawPressure + fPressure) / 2.0;
      pressure = fPressure;
      break;

    case 2:
      fPressure = ((float)rawPressure + fPressure*3) / 4.0;
      pressure = fPressure;
      break;

    case 3:
      fPressure = ((float)rawPressure + fPressure*7) / 8.0;
      pressure = fPressure;
      break;

    case 4:
      fPressure = ((float)rawPressure + fPressure*15) / 16.0;
      pressure = fPressure;
      break;

    case 5:
      fPressure = ((float)rawPressure + fPressure*31) / 32.0;
      pressure = fPressure;
      break;

    case 6:
      fPressure = ((float)rawPressure + fPressure*63) / 64.0;
      pressure = fPressure;
      break;

  }

  if (pressure > 10) {
    ccVal = pressure - 10;
    sign=1;

  }
  else if (pressure < -70 && conf.twoflows) {
    ccVal = -pressure - 70;
    sign=-1;
  }
  else {
    ccVal = 0;
    digitalWrite(ledPin, LOW);
  }

  if (ccVal) {
    digitalWrite(ledPin, HIGH);
    if (ccVal>127) {
      if (ccVal > 150) {
        if (conf.bend) {
          pwVal = -(ccVal-150)*32*sign;
        }
      }
      else {
        pwVal = 0;
      }
      ccVal = 127;
    }
  }
  else {
    pwVal = 0;
  }

  if (ccVal != ccValOld) {
    midi_cc(conf.midi_channel, conf.midi_cc, ccVal);
    ccValOld = ccVal;
  }
  if (pwVal != pwValOld) {
      midi_pitchwheel(conf.midi_channel, pwVal);
    pwValOld = pwVal;
  }

  // menu / update display
  switch (menu_state) {
    case MENU_BARGRAPH:
      if      (ccVal >= 127) { i = 6; }
      else if (ccVal >= 102) { i = 5; }
      else if (ccVal >=  77) { i = 4; }
      else if (ccVal >=  51) { i = 3; }
      else if (ccVal >=  26) { i = 2; }
      else if (ccVal >=   1) { i = 1; }
      else { i = 0; }

      out[1] = barGraph[i][0];
      out[0] = barGraph[i][1];

      if (confChanged && !mode_changed_counter) {
        save_conf();
      }

      if (paramButtonPressed()) {
        menu_state = MENU_CHANNEL;
      }
      if (plusButtonPressed()) {
        ref = analogRead(analogInPin);
      }
      break;

    case MENU_CHANNEL:
      if (mode_changed_counter) {
        out[1] = segments_CH[0];
        out[0] = segments_CH[1];
      }
      else {
        i = conf.midi_channel + 1;
        out[1] = nibbleToSegments[i / 10];
        out[0] = nibbleToSegments[i % 10];

        if (plusButtonPressed() && conf.midi_channel < MIDI_CH16) {
          midi_cc(conf.midi_channel, conf.midi_cc, 127);
          conf.midi_channel++;
          midi_cc(conf.midi_channel, conf.midi_cc, ccVal);
          confChanged = 1;
        }
        if (minusButtonPressed() && conf.midi_channel > MIDI_CH1) {
          midi_cc(conf.midi_channel, conf.midi_cc, 127);
          conf.midi_channel--;
          midi_cc(conf.midi_channel, conf.midi_cc, ccVal);
          confChanged = 1;
        }
      }

      if (paramButtonPressed()) {
        menu_state = MENU_CONTINUOUS_CONTROL;
      }
      break;

    case MENU_CONTINUOUS_CONTROL:
      if (mode_changed_counter) {
        out[1] = segments_CC[0];
        out[0] = segments_CC[1];
      }
      else {
        out[1] = nibbleToSegments[conf.midi_cc / 10];
        out[0] = nibbleToSegments[conf.midi_cc % 10];

        if (plusButtonPressed()  && conf.midi_cc < 127) {
          conf.midi_cc++;
          confChanged = 1;
        }
        if (minusButtonPressed() && conf.midi_cc >= 1) {  // todo: 1 -> 0
          conf.midi_cc--;
          confChanged = 1;
        }
      }

      if (paramButtonPressed()) {
        menu_state = MENU_BEND;
      }
      break;

    case MENU_BEND:
      if (mode_changed_counter) {
        out[1] = segments_bend[0];
        out[0] = segments_bend[1];
      }
      else {
        out[1] = 0xff;
        out[0] = nibbleToSegments[conf.bend];

        if (plusButtonPressed()  && conf.bend < 1) {
          conf.bend++;
          confChanged = 1;
        }
        if (minusButtonPressed() && conf.bend) {
          conf.bend--;
          confChanged = 1;
        }
      }
      if (paramButtonPressed()) {
        menu_state = MENU_FILTER;
      }
      break;

    case MENU_FILTER:
      if (mode_changed_counter) {
        out[1] = segments_filter[0];
        out[0] = segments_filter[1];
      }
      else {
        out[1] = 0xff;
        out[0] = nibbleToSegments[conf.filter];

        if (plusButtonPressed()  && conf.filter < 6) {
          conf.filter++;
          confChanged = 1;
        }
        if (minusButtonPressed() && conf.filter) {
          conf.filter--;
          confChanged = 1;
        }
      }

      if (paramButtonPressed()) {
        menu_state = MENU_TWOFLOWS;
      }
      break;

    case MENU_TWOFLOWS:
      if (mode_changed_counter) {
        out[1] = segments_twoflows[0];
        out[0] = segments_twoflows[1];
      }
      else {
        out[1] = 0xff;
        out[0] = nibbleToSegments[conf.twoflows];

        if (plusButtonPressed()  && conf.twoflows < 1) {
          conf.twoflows++;
          confChanged = 1;
        }
        if (minusButtonPressed() && conf.twoflows) {
          conf.twoflows--;
          confChanged = 1;
        }
      }

      if (paramButtonPressed()) {
        menu_state = MENU_BARGRAPH;
      }
      break;
  }

  // you can also play with
  //midi_send(NOTE_OFF, MIDI_CH1, noteNumber, 0);
  //midi_send(NOTE_ON,  MIDI_CH1, noteNumber, noteVelocity);
}
