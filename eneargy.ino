
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

#define SIZE_INPUT 2
#define SIZE_OUTPUT 3

#define PIN_PIXELS      6
#define NUMPIXELS      59

#define LORA_PORT     5

#define neighbourSerial Serial
#define debugSerial SerialUSB
#define loraSerial  Serial2
#define FRM_PAYLOAD_MAX_LENGTH  255
#define FIXED_VOLTAGE 230

//inputs
#define NATIONAL 7u
#define NEIGHBOUR 8u
#define ALT 9u

#define NET_INDEX 0
#define ALT_INDEX 1

#define PLUS_BUTTON 10u
#define MINUS_BUTTON 11u

#define SERIAL_NEW_CONSUMPTION_COMMAND 'c' // consumption
#define SERIAL_NEED_POWER_COMMAND 'b'      // buy
#define SERIAL_POWER_AVAILABLE_COMMAND 's' // sell

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
int maxButton = 0;
uint8_t frameReceived[FRM_PAYLOAD_MAX_LENGTH];

int powerSrc = NET_INDEX;
int currentNeighbourCons = 0;
int powerThreshold[SIZE_OUTPUT] = {0, 0, 0};
double output[SIZE_OUTPUT] = {0, 0, 0};
double input[SIZE_INPUT] = {0, 0};
bool is_input[SIZE_INPUT] = {false, false};
int nb_led = 0;
int max_led = 6;

void initPin() {
  //Initialize the LEDs and turn them all off
  pinMode(LED_RED, OUTPUT) ;
  pinMode(LED_GREEN, OUTPUT) ;
  pinMode(LED_BLUE, OUTPUT) ;

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
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

  //Set ADC resolution to 12 bits
  analogReadResolution(12) ;

  // Init Serial connection with neighbour
  neighbourSerial.begin(9600);

  // Init power src pin
  initPowerSrc(NATIONAL, NET_INDEX);
  initPowerSrc(ALT, ALT_INDEX);

  initPoducer();
}

void initPowerSrc(int pin, int srcIndex) {
  pinMode(pin, INPUT);
  is_input[srcIndex] = (digitalRead(pin) == HIGH);
}

void initConsumer() {
  nb_led = 5;
  is_input[0] = true;
  is_input[1] = false;
  input[ALT_INDEX] = 0;
  powerThreshold[0] = 1;
  powerThreshold[1] = 3;
  powerThreshold[2] = 5;
}

void initPoducer() {
  nb_led = 8;
  is_input[0] = true;
  is_input[1] = true;
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

float getPower(int load) {
  double Irms = load * 46;
  double power = Irms * FIXED_VOLTAGE;
  return power;
}

double getEnergy(double power, float period) { //power in watt, period in second.
  return power * period / 3600; //energy in Wh
}

void sendNewConsumption(int consumption) {
  char value = SERIAL_NEW_CONSUMPTION_COMMAND;
  neighbourSerial.write(value);
  value = consumption;
  neighbourSerial.write(value);
}

void processNeighbourRequest() {
  char command;
  char data;
  command = neighbourSerial.read();
  data = neighbourSerial.read();

  switch(command) {
  case SERIAL_NEW_CONSUMPTION_COMMAND:
    currentNeighbourCons = data;
    break;
  case SERIAL_NEED_POWER_COMMAND:
    break;
  case SERIAL_POWER_AVAILABLE_COMMAND:
    break;
  }
}

void loop() {
  //mise à jour compte LED
  int old_led_nr = nb_led;
  if (digitalRead(PLUS_BUTTON) == LOW)
    plusButton = 1;

  if (digitalRead(PLUS_BUTTON) == HIGH && plusButton == 1) {
    nb_led++;
	debugSerial.println("add led : ");
	debugSerial.println(nb_led);
    plusButton = 0;
  }

  if (digitalRead(MINUS_BUTTON) == LOW)
    minusButton = 1;

  if (digitalRead(MINUS_BUTTON) == HIGH && minusButton == 1) {
    nb_led--;
	debugSerial.println("rm led : ");
	debugSerial.println(nb_led);
    minusButton = 0;
  }

  if (digitalRead(BUTTON) == LOW)
	maxButton = 1;

  if (digitalRead(BUTTON) == HIGH && maxButton == 1){
	if(max_led == 9){
		max_led = 6;
	}
	else{
		max_led = 9;
	}
	maxButton = 0;
  }

  //mise à jour LED
  for(int i=0; i< 9; i++){
		if(i < 3){
			if(i < min(nb_led, max_led)){
    			pixels.setPixelColor(i, pixels.Color(0,255,0)); // green
			}else{
				pixels.setPixelColor(i, pixels.Color(0, 0, 0)); //off			
			}
		}else if(i < 6){
			if (i < min(nb_led, max_led)){
				pixels.setPixelColor(i, pixels.Color(255, 150, 0)); // orange
			}else{
				pixels.setPixelColor(i, pixels.Color(0, 0, 0)); //off
			}
		}else{
			if(i < min(nb_led, max_led)){
				pixels.setPixelColor(i, pixels.Color(255, 0, 0)); // red
			}else{
				pixels.setPixelColor(i, pixels.Color(0, 0, 0)); //off
			}
		}
    	pixels.show(); // This sends the updated pixel color to the hardware.
  }

  if(nb_led != old_led_nr) {
    sendNewConsumption(nb_led);
  }
  // compteur de mise à jour du calcul de l'énergie (en ms) 15000 = 15 secondes
  if (millis() - energyTs > 15000) {
    //energie consomé en Wh durant 15 secondes
    int load = (nb_led > max_led) ? max_led : nb_led;
    debugSerial.println("DATAS0 :");
    debugSerial.println(nb_led);
    debugSerial.println(max_led);
    debugSerial.println(load);

    if(max_led >= 3) {
      if(load > 3)
        output[0] += getEnergy(getPower(3), 15);
      else if(load > 0)
        output[0] += getEnergy(getPower(load), 15);
      else
        output[0] += getEnergy(0, 15);
      load -= 3;
    }
    
    if(max_led >= 6) {
      if(load > 3)
        output[1] += getEnergy(getPower(3), 15);
      else if(load > 0)
        output[1] += getEnergy(getPower(load), 15);
      else
        output[1] += getEnergy(0, 15);
      load -= 3;
    }
    
    if(max_led >= 9) {
      if(load > 3)
        output[2] += getEnergy(getPower(3), 15);
      else if(load > 0)
        output[2] += getEnergy(getPower(load), 15);
      else
        output[2] += getEnergy(0, 15);
      load -= 3;
    }
    
    if(is_input[ALT_INDEX])
      input[ALT_INDEX] += getEnergy(getPower(2), 15);

    debugSerial.println("DATAS :");
    debugSerial.println(output[0]);
    debugSerial.println(output[1]);
    debugSerial.println(output[2]);
    debugSerial.println(input[ALT_INDEX]);

    energyTs = millis();
  }
  // compteur pour le timing d'envoi des messages (en ms) 300000 = 5 minutes
  if (millis() - timestamp > 60000) {

    short state = 0;
    if(nb_led < max_led) state = 7;
    else if(max_led > 3) state = 3;
    else if(max_led == 0) state = 0;
    else state = 1;

    LpwaOrange.flush();
    LpwaOrange.addByte(state);
    LpwaOrange.addShort(output[0]);
    LpwaOrange.addShort(output[1]);
    LpwaOrange.addShort(output[2]);

    short total_value = output[0] + output[1] + output[2];
    LpwaOrange.addShort(total_value);

    LpwaOrange.addShort(input[ALT_INDEX]);

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

    for (int i = 0; i < SIZE_OUTPUT; i++)
      output[i] = 0;

    for (int i = 0; i < SIZE_INPUT; i++)
      input[i] = 0;
  }

  if(neighbourSerial.available()) {
    processNeighbourRequest();
  }

}
