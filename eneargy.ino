
#include <EddystoneBeacon.h>
#include <RN487x_CONST.h>
#include <RN487x_BLE.h>
#include <iBeacon.h>
#include <Led.h>

#include <Sodaq_wdt.h>
#include <Switchable_Device.h>
#include <LpwaOrange.h>
#include <SodaqExt_RN2483.h>
#include <Utils.h>
#include <StringLiterals.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include "Arduino.h"

#define PIN_PIXELS      6
#define NUMPIXELS      59

#define LORA_PORT     5

#define neighbourSerial Serial
#define debugSerial SerialUSB
#define loraSerial  Serial2
#define FRM_PAYLOAD_MAX_LENGTH  255
#define FIXED_VOLTAGE 230

//LED
#define DIODE0 2u
#define DIODE1 3u
#define DIODE2 4u
#define DIODE3 5u
#define DIODE4 6u

//inputs
#define NATIONAL 7u
#define NEIGHBOUR 8u
#define ALT 9u

#define NET 0
#define ALT 1
#define NEI 2

#define PLUS_BUTTON 10u
#define MINUS_BUTTON 11u

// OTAA PRODUCTION Keys - Use your own KEYS!

const uint8_t devEUI[8]  = {0x21, 0x5F, 0xD5, 0x69, 0x16, 0x7F, 0x5D, 0x9A} ;
const uint8_t appEUI[8]  = {0xA0, 0xAC, 0xB2, 0x67, 0x0D, 0xA9, 0xB1, 0xAE} ;
const uint8_t appKey[16] = {0x0A, 0x6E, 0x2C, 0x57, 0x0D, 0x3B, 0xEF, 0x60, 0x13, 0xD1, 0x14, 0xCB, 0x13, 0xD8, 0x7B, 0xDA};

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN_PIXELS, NEO_GRB + NEO_KHZ800);

//initialisation des variables
long timestamp = 0;
long energyTs = 0;
int minusButton = 0;
int plusButton = 0;
uint8_t frameReceived[FRM_PAYLOAD_MAX_LENGTH];

int powerSrc = NET;
int currentNeighbourCons = 0;
int maxCons[] = {0, 0, 0};
bool changed[] = {0, 0, 0};
int powerThreshold[] = {0, 0, 0};
double output[2] = {0, 0};
double input[3] = {0, 0, 0};
bool is_input[3] = {false, false, false};
int nb_led = 0;

void initPin() {
  //Initialize the LEDs and turn them all off
  pinMode(LED_RED, OUTPUT) ;
  pinMode(LED_GREEN, OUTPUT) ;
  pinMode(LED_BLUE, OUTPUT) ;

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  //Set LED as output
  pinMode(DIODE0, OUTPUT);
  pinMode(DIODE1, OUTPUT);
  pinMode(DIODE2, OUTPUT);
  pinMode(DIODE3, OUTPUT);
  pinMode(DIODE4, OUTPUT);

  //set power source as input
  pinMode(NATIONAL, INPUT);
  pinMode(NEIGHBOUR, INPUT);
  pinMode(ALT, INPUT);

  pinMode(PLUS_BUTTON, INPUT_PULLUP);
  pinMode(MINUS_BUTTON, INPUT_PULLUP);

  //Set the energy sensor pin as input
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  //pinMode(BUTTON, INPUT_PULLUP);

  //Set ADC resolution to 12 bits
  analogReadResolution(12) ;

  // Init Serial connection with neighbour
  neighbourSerial.begin(9600);

  // Init power src pin
  initPowerSrc(NATIONAL, NET);
  initPowerSrc(NEIGHBOUR, ALT);
}

void initPowerSrc(int pin, int srcIndex) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  is_input[srcIndex] = (digitalRead(pin) == HIGH);
}

void initConsumer() {
  nb_led = 5;
  maxCons[0] = 7;
  maxCons[1] = 5;
  maxCons[2] = 0;
  is_input[0] = true;
  is_input[1] = true;
  is_input[2] = false;
  powerThreshold[0] = 1;
  powerThreshold[1] = 3;
  powerThreshold[2] = 5;
}

void initPoducer() {
  nb_led = 4;
  maxCons[0] = 7;
  maxCons[1] = 0;
  maxCons[2] = 0;
  is_input[0] = 1;
  is_input[1] = 0;
  is_input[2] = 0;
  powerThreshold[0] = 3;
  powerThreshold[1] = 4;
  powerThreshold[2] = 5;
}

void red() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void green() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void blue() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void white() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

void orange() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void setup() {
  debugSerial.begin(57600);
  loraSerial.begin(LpwaOrange.getDefaultBaudRate());

  initPin();
  pixels.begin(); // This initializes the NeoPixel library.
  for(int i=0;i<NUMPIXELS;i++){
    pixels.setPixelColor(i, pixels.Color(200,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
  }

  orange();
  delay(10000) ;

  LpwaOrange.flush();
  LpwaOrange.setDiag(debugSerial) ; // optional

  bool res = 0;
  int tentative = 0;
  do {
    blue();
    debugSerial.println("OTAA Join Request") ;
    res = LpwaOrange.joinLoRaNetwork(loraSerial, devEUI, appEUI, appKey, true, 4);
    debugSerial.println(res ? "OTAA Join Accepted." : "OTAA Join Failed! Trying again. Waiting 10 seconds.") ;
    if (!res) {
      red();
      tentative++;
      delay(10000);
    }
    if (tentative == 3) {
      while (1) {
        red();
        delay(1000);
        orange();
        delay(1000);
        white();
        delay(1000);
      }
    }
  } while (res == 0);

  debugSerial.println("Sleeping for 5 seconds before starting sending out test packets.");
  sleep(5);
}

void sleep(unsigned short count) {
  for (uint8_t i = count; i > 0; i--) {
    debugSerial.println(i) ;
    delay(1000) ;
  }
}

void debugFrame(const char* frame, int len) {
  if (frame != NULL) {
    debugSerial.print("Payload: ");
    int i = 0;
    for (i = 0; i < len; i++) {
      unsigned char value = (unsigned char)frame[i];
      debugSerial.print(value, HEX); debugSerial.print(" ");
    }
    debugSerial.print("\r\n");
  }
}

float getPower(int pin) {
  double Irms = nb_led * 46;
  double power = Irms * FIXED_VOLTAGE;
  return power;
}

double getEnergy(double power, float period) { //power in watt, period in second.
  return power * period / 3600; //energy in Wh
}

void led_up(int led) {
  digitalWrite(led, HIGH);
}

void led_down(int led) {
  digitalWrite(led, LOW);

}
void update_led(int nb) {
  if (nb < 1) {
    led_down(DIODE0);
  }
  if (nb < 2) {
    led_down(DIODE1);
  } else {
    led_up(DIODE1);
  }
  if (nb < 3) {
    led_down(DIODE2);
  } else {
    led_up(DIODE2);
  }
  if (nb < 4) {
    led_down(DIODE3);
  } else {
    led_up(DIODE3);
  }
  if (nb >= 4) {
    led_up(DIODE4);
  }

}

void loop() {
  //mise à jour compte LED
  if (digitalRead(PLUS_BUTTON) == LOW)
    plusButton = 1;

  if (digitalRead(PLUS_BUTTON == HIGH) && plusButton == 1) {
    nb_led++;
    minusButton = 0;
  }

  if (digitalRead(MINUS_BUTTON == LOW))
    minusButton = 1;

  if (digitalRead(MINUS_BUTTON == HIGH) && minusButton == 1) {
    nb_led--;
    minusButton = 0;
  }
  //mise à jour LED
  update_led(nb_led);
  // compteur de mise à jour du calcul de l'énergie (en ms) 15000 = 15 secondes
  if (millis() - energyTs > 15000) {
    output[0] += getEnergy(getPower(0), 15); //energie consomé en Wh durant 15 secondes
    output[1] += getEnergy(getPower(1), 15);
    if (is_input[0])
      input[0] += getEnergy(getPower(2), 15);
    if (is_input[1])
      input[1] += getEnergy(getPower(3), 15);
    if (is_input[2])
      input[2] += getEnergy(getPower(4), 15);
    if (is_input[3])
      input[3] += getEnergy(getPower(5), 15);
  }
  // compteur pour le timing d'envoi des messages (en ms) 300000 = 5 minutes
  if (millis() - timestamp > 300000) {

    int8_t inputs = 0;

    for (int i = 0; i < 4; i++) {
      if (is_input[i])
        inputs = inputs & (1 << i + 4);
    }

    LpwaOrange.flush();
    LpwaOrange.addByte(inputs);
    for (int i = 0; i < sizeof(input) / sizeof(*input); i++) {
      LpwaOrange.addShort((short)input[i]);
    }
    for (int i = 0; i < sizeof(output) / sizeof(*output); i++) {
      LpwaOrange.addShort((short)output[i]);
    }
    int len;
    const char* frame = LpwaOrange.getFramePayload(&len);
    debugFrame(frame, len);

    switch (LpwaOrange.send(LORA_PORT, (const uint8_t*)frame, len)) {
    case NoError:
      debugSerial.println("Successful transmission.");
      digitalWrite(LED_BUILTIN, LOW);
      green();
      delay(2000);
      white();
      break;

    case NoResponse:
      debugSerial.println("There was no response from the device.");
      red();
      break ;

    case Timeout:
      debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
      red();
      delay(20000) ;
      break ;

    case PayloadSizeError:
      debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
      red();
      break ;

    case InternalError:
      debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe program will now halt.");
      red();
      while (1) {} ;
      break ;

    case Busy:
      debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
      red();
      delay(10000) ;
      break ;

    case NetworkFatalError:
      debugSerial.println("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe program will now halt.");
      red();
      while (1) {} ;
      break ;

    case NotConnected:
      debugSerial.println("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe program will now halt.");
      red();
      while (1) {} ;
      break ;

    case NoAcknowledgment:
      debugSerial.println("There was no acknowledgment sent back!");
      red();
      break ;

    default:
      break ;
    }


    int frameReceivedSize = LpwaOrange.receive(frameReceived, FRM_PAYLOAD_MAX_LENGTH);

    debugSerial.print("received : ");
    for (int i = 0; i < frameReceivedSize; i++) {
      debugSerial.print(frameReceived[i], HEX);
      debugSerial.print(" ");
    }
    debugSerial.println("");

    timestamp = millis();
    energyTs = millis();

    for (int i = 0; i < sizeof(output) / sizeof(*output); i++)
      output[i] = 0;

    for (int i = 0; i < sizeof(input) / sizeof(*input); i++)
      input[i] = 0;
  }
}
