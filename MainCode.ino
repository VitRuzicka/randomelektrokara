// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           300         // [ms] Sending time interval
#define SPEED_MAX_TEST      400         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(3, 2);        // RX, TX

#define KEY_TIMEOUT_OFF 5000
#define KEY_TIMEOUT_POWER 20000
#define USART_COM_TIMEOUT 30000

#define CONTACTOR_PIN 12
#define ESC_PIN 11
#define KEY_SWITCH1 6
#define KEY_SWITCH2 7
#define ANALOG_ACCELERATOR A5
#define ANALOG_BRAKE A7

#include <FastLED.h>

#define LED_PIN_FRONT 10
#define LED_PIN_REAR 4

bool ESCEnabled = 0;
uint32_t KeyPMill;
uint8_t KeyPrev;
int16_t SpeedGradationValue;
uint32_t SpeedGradationPMill;
uint32_t USARTTimeoutPMill;
bool USARTLost;
uint32_t FirePMill;
uint32_t RearFlamePMill;
uint8_t RearFlameStarted;

CRGB leds[16];
CRGB ledsBack[8];

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
uint32_t dataSendPMill;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);

  FastLED.addLeds<WS2812B, LED_PIN_FRONT, GRB>(leds, 16).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<WS2812B, LED_PIN_REAR, GRB>(ledsBack, 8).setCorrection( TypicalLEDStrip );
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ESC_PIN, OUTPUT);
  pinMode(CONTACTOR_PIN, OUTPUT);
  pinMode(KEY_SWITCH1, INPUT_PULLUP);
  pinMode(KEY_SWITCH2, INPUT_PULLUP);
  ESCOn();
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte    = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
        USARTTimeoutPMill = millis();
    }
    else {  //if there is no data on the SoftwareSerial, update the LEDs, that way, we have the highest framerate without loosing serail data from the ESC
      if (!USARTLost) {
        if (ESCEnabled) {
          SpeedIndicator(analogRead(ANALOG_ACCELERATOR) > 1022, (Feedback.speedR_meas + Feedback.speedL_meas) >> 1);  //average both wheels
          RearLight((KeyPosition() == 0) ? 1 : 0, analogRead(ANALOG_BRAKE) > 50, analogRead(ANALOG_ACCELERATOR) > 1022);
        }
        else FastLED.clear();
        BatteryIndicator(constrain(map(Feedback.batVoltage, 3100, 3800, 0, 1023), 0, 1023), Feedback.batVoltage < 3050);
        FastLED.show();
      }
      
      return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;  
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    } 
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

void loop(void) {
  // Check for new received data
  Receive();

  // Send commands
  if (millis() - dataSendPMill >= 50) {
    dataSendPMill = millis();
    
    if(KeyPosition() == 0) {  //backwards
      int16_t SendSpeed = mapClamp(analogRead(ANALOG_ACCELERATOR), 750, 1023, 0, -500)/* + ((mapClamp(analogRead(ANALOG_BRAKE), 50, 450, 0, -100) * ((Feedback.speedR_meas + Feedback.speedL_meas) >> 1)) / 10)*/;  //the last commented out thing was my attempt at making a brake but we had otherissues so I disabled it
      Send(0,SpeedGradation(SendSpeed, 1, 8));
    }
    else if (KeyPosition() == 2) {  //forwards
      int16_t SendSpeed = mapClamp(analogRead(ANALOG_ACCELERATOR), 750, 1023, 0, 700)/* - ((mapClamp(analogRead(ANALOG_BRAKE), 50, 450, 0, -100) * ((Feedback.speedR_meas + Feedback.speedL_meas) >> 1)) / 10)*/;
      Send(0,SpeedGradation(SendSpeed, 1, 8));
    }
    else {  //pedals disconnected
      Send(0,0);
    }
  }

  /*if (millis() - KeyPMill >= KEY_TIMEOUT_OFF && KeyPosition() == 1) ESCOff(); //if the key is in the middle, after 5 seconds, turn of the ESC - we were having issues, so I turned it off
  else ESCOn();
  if (millis() - KeyPMill >= KEY_TIMEOUT_POWER && KeyPosition() == 1) ESCEmergencyOff();*/

  if (millis() - USARTTimeoutPMill > USART_COM_TIMEOUT && (ESCEnabled || USARTLost)) {  //if there is no communication on the USART line from the ESC, induce a sizeure to the driver
    ESCEmergencyOff();
    for (uint8_t i = 0; i < 16; i++) {
      leds[i] = ((millis()%250)<125) ? 0x0000ff : 0xff0000;
    }
    USARTLost = 1;
    FastLED.show();
  }

  digitalWrite(LED_BUILTIN, (millis()%2000)<1000);
}

void ESCOn() {  //turn on the contactor and turn on the ESC
  if (!ESCEnabled) {  //only do this if the ESC is off
    digitalWrite(ESC_PIN, 1);
    digitalWrite(CONTACTOR_PIN, 0);
    delay(100);
    digitalWrite(ESC_PIN, 0);
    delay(100);
    digitalWrite(ESC_PIN, 1);
    ESCEnabled = 1;
  }
}

void ESCOff() { //sends a pulse to the ESC enable relay to turn it off
  if (ESCEnabled) {  //only do this if the ESC is off
    digitalWrite(ESC_PIN, 0);
    delay(100);
    digitalWrite(ESC_PIN, 1);
    ESCEnabled = 0;
  }
}

void ESCEmergencyOff() {  //shut off the contactor
  //doesnt check if the variable is off for safety reasons (if in any way the variable says off but the ESC is still on)
  digitalWrite(CONTACTOR_PIN, 1);
  ESCEnabled = 0;
}

uint8_t KeyPosition() {
  uint8_t state = ((digitalRead(KEY_SWITCH1) << 1) | digitalRead(KEY_SWITCH2)) - 1; // 0-left, 01-middle, 10-right
  if (KeyPrev != state) {
    KeyPrev = state;
    KeyPMill = millis();  //reset the timer when the key position changes, used for timeout
  }
  return state;
}

int16_t SpeedGradation(int16_t DesiredSpeed, uint8_t riseSpeed, uint8_t addition) { //addsto a variable which it utputs, if the variable is higher than the target, it gets diminshed

  if (millis() - SpeedGradationPMill >= riseSpeed) {
    SpeedGradationPMill = millis();
    SpeedGradationValue += addition;
  }

  if (SpeedGradationValue > DesiredSpeed) SpeedGradationValue = DesiredSpeed;
  return SpeedGradationValue;
}

long mapClamp(long x, long min_in, long max_in, long min_out, long max_out) {
  if (min_out <= max_out) return constrain(map(x, min_in, max_in, min_out, max_out), min_out, max_out);
  else return constrain(map(x, min_in, max_in, min_out, max_out), max_out, min_out);
}

//LED STUFF----------------------
void BatteryIndicator(uint16_t input, bool batteryDischarged) {
  if (!batteryDischarged) {
    for (uint8_t i = 0; i < 8; i++) {
      leds[i] = CHSV(map(i, 0, 7, 0, 90), 255, ((i == input >> 7) ? (input & 0x7e) >> 1 : 64)  *  (i <= (input >> 7)) );
    }
  }
  else {
    for (uint8_t i = 0; i < 8; i++) {
      leds[i] = 0xff0000 * ((millis()%1000)<500);
    }
  }
}

void SpeedIndicator (bool flame, int16_t speed) {
  uint16_t SpeedLong = constrain(map(speed, 0, 500, 0, 1023), 0, 1023);

  if (millis() - FirePMill >= 16) { //timing the fire loop
    for (uint8_t i = 0; i < 8; i++) {  //main code is here too, so the fire can reliably overwrite it
      leds[i + 8] = CHSV(i < 6 ? 90 : 0, 255, ((i == SpeedLong >> 7) ? (SpeedLong & 0x7e) >> 1 : 64)  *  (i <= (SpeedLong >> 7)) );
    }
    
    FirePMill = millis();
    Fire2022(flame); //if pedal fully pressed, blazeitt!!!
  }
  
}

void RearLight(bool reverse, bool breaking, bool flame) {
  if (breaking) for (uint8_t i = 0; i < 4; i++) ledsBack[i] = 0xff0000;
  else if (reverse) for (uint8_t i = 0; i < 4; i++) ledsBack[i] = 0x7f7f7f;
  else if (flame) {
    if (millis() - RearFlamePMill >= 100) {
      RearFlamePMill = millis();
      if (random8() < 100) {
        ledsBack[3] = 0xffff99;
        ledsBack[2] = 0xffff66;
        ledsBack[1] = 0xffff00;
        ledsBack[0] = 0xff9900;
        RearFlameStarted = 10;
      }
    }
    else if (RearFlameStarted) {
      ledsBack[3] = 0xffff99;
      ledsBack[2] = 0xffff00;
      ledsBack[1] = 0x999900;
      ledsBack[0] = 0x666600;
      RearFlameStarted--;
    }
    else for (uint8_t i = 0; i < 4; i++) ledsBack[i] = 0xf0000;
  }
  else for (uint8_t i = 0; i < 4; i++) ledsBack[i] = 0xf0000;
  for (uint8_t i = 0; i < 4; i++) ledsBack[i + 4] = ledsBack[3 - i];  //copy ope part of the string to the other
}

uint8_t ShiftSpeedCounter;
static byte heat[8];
bool FlameStarting = 0;
#define COOLING  15
#define SPARKING 50
#define SHIFT_SPEED 4
void Fire2022(bool BlazeItt) {
  // Step 1.  Cool down every cell a little
    for( int i = 1; i < 8; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / 8) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    if (ShiftSpeedCounter == SHIFT_SPEED) {
      ShiftSpeedCounter = 0;
      for( int k= 8 - 1; k >= 1; k--) {
        if (k > 1) heat[k] = (heat[k - 1] + heat[k - 1] + heat[k - 2] ) / 3;
        else heat[k] = (heat[k - 1] * 2) / 3;
      }
    }
    else ShiftSpeedCounter++;
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if (BlazeItt && heat[0] < 240) {
      heat[0] += 15;
      if (heat[0] > 240) FlameStarting = 0;
    }
    if( random8() < SPARKING && BlazeItt && !FlameStarting) {
      if (heat[0] < 244 && heat[0] > 210) heat[0] += random8(20) - 9;//qadd8( heat[rndpos], random8(100,255) );
      else if (heat[0] > 240) heat[0] -= random8(10);
      else if (heat[0] < 210) heat[0] = 255;
    }
    else if (!BlazeItt && heat[0] > 0) {
      heat[0]--;
      FlameStarting = 1;
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < 8; j++) {
      CRGB color = 0x000000;
      if (heat[j] > 180)color += CHSV(177, 255, map(heat[j], 180, 255, 0, 255));
      if (heat[j] < 200)color += CHSV(24, map(heat[j], 0, 200, 255, 180), heat[j]);
      if (heat[0] > 120) leds[j + 8] = 0;
      leds[j + 8] += color;
    }
}
