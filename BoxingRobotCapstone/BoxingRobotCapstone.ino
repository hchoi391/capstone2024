// Project: Boxing Robot
// Group: Capstone 2024, Georgia Institute of Technology
// Date: 04/23/2024
// Description: The Arduino code to control the punching motors, arm touch sensors,
// arm limit switches, chest pad impact sensors, head impact sensors, timer, and
// neoPixel strips for chest pad and health-point,
// Microcontroller: Arduino Uno

// =================================================================================

// ------------ LIBRARIES ----------------------------------------------------------

#include <Adafruit_NeoPixel.h>
#include <TM1637Display.h>

// ------------ CONSTANTS (won't change) -------------------------------------------

// <<< State Machine >>>

// States of boxing robot
enum {idleState, testState, readyState, playState, pauseState, gameOverState};

unsigned long currentMicros = micros();

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< armSensor >>>

// arm sensors input pins
const int straightArmSensor = A1;
const int swingArmSensor = A2;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< chestSensor >>>

// chest pad sensors input pins
const int padSensorInputPin_A = 4;
const int padSensorInputPin_B = 3;
const int padSensorInputPin_C = 2;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< limitSwitch >>>

// limit switch input pins
const int straightArmSwitch = 5;
const int swingArmSwitch = 6;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< motor >>>

// combined motor control output pins
const int motorPUL = 7;
const int motorDIR = 8;

// separated motor enables 
const int straightENA = 9;
const int swingENA = 10;

//24400 steps = 90 degree, 32520 steps = 120 degree
const int motorFastSpeed = 30;
const int motorSlowSpeed = 30;
const double straightMotorSteps = 6000; // 10000 originally
const double swingMotorSteps = 12000; //////////////////////////////?asdlfkjaslkdjfl


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< neoPixel >>>

// ----- For Health Bar LED -----

const int neoPixelOutputPin1 = A3;      // neoPixel Arduino output pin
const int neoPixelSize1 = 20;     // number of NeoPixels on the strip
const long neoPixelTestInterval = 500000; // 5 sec (in micro seconds)

// ----- For Chest Pad LED ------
const int neoPixelOutputPin = 11;      // neoPixel Arduino output pin
const int neoPixelSize = 45; //20;     // number of NeoPixels on the strip
const int neoPixelIdleInterval = 8204; // Time (in micro seconds)

// Declare NeoPixel strip object
Adafruit_NeoPixel pixels(neoPixelSize, neoPixelOutputPin, NEO_RGBW + NEO_KHZ800);
Adafruit_NeoPixel pixels1(neoPixelSize1, neoPixelOutputPin1, NEO_GRB + NEO_KHZ800);

// Color data and array for idle state
const uint32_t cI1 = pixels.Color(20,100,0);
const uint32_t cI2 = pixels.Color(40,125,0);
const uint32_t cI3 = pixels.Color(60,150,0);
const uint32_t cI4 = pixels.Color(80,175,0);
const uint32_t cI5 = pixels.Color(100,200,0);
const uint32_t cI6 = pixels.Color(120,225,0);
const uint32_t cI7 = pixels.Color(140,225,0);

uint32_t colorIdle[18] = {cI1,cI2,cI3,cI4,cI5,cI6,cI6,cI7,cI7,
                          cI7,cI7,cI6,cI6,cI5,cI4,cI3,cI2,cI1};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< timer >>>

const int timerDIO = 12;        // timer DIO pin
const int timerCLK = 13;        // timer CLK pin

TM1637Display timer(timerCLK, timerDIO);


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< headSensor >>>

const int headSensorPin = A0;   // head sensor input pin

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< RandomSeed >>>

const int randomPunch = A5;
const int ramdomScriptButton = A4;
bool isScriptOn = false;


// ------------ VARIABLES (will change) --------------------------------------------

// <<< State Machine >>>

// Initital state of boxing robot
String raspiData;

String scriptData[2] = {"straightPunch", "swingPunch"}; 

bool scriptFlag = false;

unsigned char robotState = idleState;
bool stateFlag = false;
int stateIndex = 0;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< armSensor >>>

int armStraightRead = 0;
int armSwingRead = 0;
bool armStraightFlag = false;
bool armSwingFlag = false;

bool armST = false;
bool armSW = false;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< chestSensor >>>

// boolean flags for impact detection
bool leftPadDetect = false;        
bool bottomPadDetect = false;
bool topPadDetect = false;
bool rightPadDetect = false;

bool left = false;
bool right = false;
bool top = false;
bool bottom = false;

byte chestPadTestOutput = 0b000;

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< limitSwitch >>>

int limitStraightRead = 0;
int limitSwingRead = 0;

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< motor >>>


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< neoPixel >>>

// ------ For Health Bar LED ------

unsigned long previousNeoPixelMicros;
int leftIndex = 0;                        
int rightIndex = 19;
bool healthBarFlag = false;

// ------ For Chest Pad LED ------

int neoPixelBrightness = 50;              // set brightness (max = 255)
int neoPixelIndex = 0;                    // index for neoPixel setcolor()
int colorIndex = 0;                       // index for color array


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< timer >>>

// Countdown Timer
unsigned long COUNTDOWN_TIME = 5; // 5 seconds
unsigned long timerStart;
unsigned long timerCurrent;
unsigned long timerElapsed;
bool timerFlag = false;
unsigned long remainingTime;

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< headSensor >>>

int headSensorRead = 0;
bool headSensorFlag = false;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// <<< game data >>>

int boxerHealth;
int robotHealth;

// =================================================================================

void setup() {
  
  Serial.begin(9600);

  pixels.begin();         // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();         // Clear all data in buffer
  pixels.setBrightness(neoPixelBrightness);
  pixels.show();          // display all pixels ASAP
  
  pixels1.begin();
  pixels1.clear();
  pixels1.setBrightness(neoPixelBrightness);
  pixels1.show();
  
  pinMode (motorPUL, OUTPUT);
  pinMode (motorDIR, OUTPUT);
  pinMode (straightENA, OUTPUT);
  pinMode (swingENA, OUTPUT);

  pinMode(padSensorInputPin_A, INPUT);
  pinMode(padSensorInputPin_B, INPUT);
  pinMode(padSensorInputPin_C, INPUT);

  pinMode(straightArmSwitch, INPUT);
  pinMode(swingArmSwitch, INPUT);

  pinMode(straightArmSensor, INPUT);
  pinMode(swingArmSensor, INPUT);
  
  timer.setBrightness(7);
  timer.clear();

  pinMode(headSensorPin, INPUT);

  randomSeed(analogRead(randomPunch));

  pinMode(ramdomScriptButton, INPUT);
  
}

// =================================================================================

void loop() { 
  
  if (Serial.available()) {
    
    raspiData = Serial.readString();
    raspiData.trim(); 
    Serial.println(raspiData);
    
  } 

  switch (robotState) {

    case idleState: //-----------------------------

      if (raspiData == "testState") {
        robotState = playState;
      }
      raspiData = "none";
      currentMicros = micros();
      neoPixelIdle();
      
      break;
     
    case testState: //------------------------------
    
      motorStraightCCW(motorSlowSpeed);
      stateFlag = true;
      printNeoPixel(17, 30, pixels.Color(255, 0, 0));
      printNeoPixel(30, 38, pixels.Color(255, 0, 0));
      printNeoPixel(6, 17, pixels.Color(255, 0, 0));
      printNeoPixel(0, 6, pixels.Color(255, 0, 0));
      printNeoPixel(38, 45, pixels.Color(255, 0, 0));
      pixels.show();

      while(stateFlag) {
        chestSensor();

        if (leftPadDetect) {
          left = true;
          printNeoPixel(17, 30, pixels.Color(0, 0, 0));
          pixels.show();
        } 
        
        if (rightPadDetect) {
          right = true;
          printNeoPixel(0, 6, pixels.Color(0, 0, 0));
          printNeoPixel(38, 45, pixels.Color(0, 0, 0));
          pixels.show();
        }
        
        if (topPadDetect) {
          top = true;
          printNeoPixel(6, 17, pixels.Color(0, 0, 0));
          pixels.show();
        }
        
        if (bottomPadDetect) {
          bottom = true;
          printNeoPixel(30, 38, pixels.Color(0, 0, 0));
          pixels.show();
        }
        
        if (left && bottom && top && right) {
          stateFlag = false;
        }
      }
      motorSwingCW(motorSlowSpeed);
      motorStraightCW(motorSlowSpeed);
      Serial.print("test");
      stateFlag = true;

      while (stateFlag) {
        armStraightRead = analogRead(straightArmSensor);
        armSwingRead = analogRead(swingArmSensor);
        Serial.print("ST: ");
        Serial.print(armStraightRead);
        Serial.print("   SW: ");
        Serial.println(armSwingRead);

        if (armStraightRead > 900) {
          armST = true;
        }
        
        if (armSwingRead > 810) {
          armSW = true;
        }

        if (armSW && armST) {
          stateFlag = false;
        }
        
      }
      motorSwingCCW(motorSlowSpeed);
      limitSwingRead = 0;
      stateFlag = true;
      printNeoPixel1(0,20,pixels1.Color(0,255,0));
      pixels1.show();
      timerStart = millis();
      while (stateFlag) {
        if (timerFlag == false) {
          timerMachine();
        } 
        if (healthBarFlag == false) {
          currentMicros = micros();
          healthBarTest();
        }
        if (timerFlag && healthBarFlag) {
          while (!headSensorFlag) {
            headSensorRead = analogRead(headSensorPin);
            Serial.print("Head: ");
            Serial.println(headSensorRead);
            if (headSensorRead > 700) {
              headSensorFlag = true;
              stateFlag = false;
              robotState = readyState;
            }
          }
        }
      }
      
      break;

    case readyState: //-----------------------------

      if (raspiData == "playState") {
        robotState = playState;
      }
      raspiData = "none";
      currentMicros = micros();
      neoPixelIdle();
      
      break;


    case playState: //-----------------------------

      if (stateIndex == 0) {
        resetVariable();
        stateIndex ++;
        pixels.clear();
        printNeoPixel1(0,20,pixels1.Color(0,255,0));
        pixels.show();
        pixels1.show();
        timerStart = millis();
      }
      else if (stateIndex == 1) {

        if (timerFlag) {
          neoPixelIndex = 0;        
          colorIndex = 0;
          timerFlag = false;
          timerStart = millis();
        }
        timerMachine();
        currentMicros = micros();
        neoPixelIdle();

        int randomChestNum = random(5);

//        if (digitalRead(ramdomScriptButton) == true) {
//            if (isScriptOn == true) {
//              Serial.println("Script is off");
//              isScriptOn = false;  
//            }
//            else if (isScriptOn == false) {
//              Serial.println("Script is on");
//              isScriptOn = true;  
//            }
//        }
//        
//        if (isScriptOn == true) {
            int randomPunch = random(2);
            raspiData = scriptData[randomPunch];    
//        }
         
        if (raspiData == "straightPunch") {
          pixels.clear();
          switch (randomChestNum) {
            case 0: // left
              printNeoPixel(17, 30, pixels.Color(255, 0, 0));
              pixels.show();
              motorStraightCCW(motorFastSpeed);
              if (armST) {
                healthBarMachine();
                delay(50);
                leftIndex ++;
              }
              else {
                left = true;
              }
              timerMachine();
              break;
            case 1: // top
              printNeoPixel(6, 17, pixels.Color(255, 0, 0));
              pixels.show();
              motorStraightCCW(motorFastSpeed);
              if (armST) {
                healthBarMachine();
                delay(50);
                leftIndex ++;
              }
              else {
                top = true;
              }
              timerMachine();
              break;
            case 2: // bottom
              printNeoPixel(30, 38, pixels.Color(255, 0, 0));
              pixels.show();
              motorStraightCCW(motorFastSpeed);
              if (armST) {
                healthBarMachine();
                delay(50);
                leftIndex ++;
              }
              else {
                bottom = true;
              }
              timerMachine();
              break;
            case 3: // right
              printNeoPixel(0, 6, pixels.Color(255, 0, 0));
              printNeoPixel(38, 45, pixels.Color(255, 0, 0));
              pixels.show();
              motorStraightCCW(motorFastSpeed);
              if (armST) {
                healthBarMachine();
                delay(50);
                leftIndex ++;
              }
              else {
                right = true;
              }
              timerMachine();
              break;
            case 4: // head
              printNeoPixel(0, 45, pixels.Color(255, 255, 0));
              pixels.show();
              motorStraightCCW(motorFastSpeed);
              if (armST) {
                healthBarMachine();
                leftIndex ++;
              }
              else {
                headSensorFlag = true;
              }
              timerMachine();
              break;
          }
        }
        if (raspiData == "swingPunch") {
          pixels.clear();
          switch (randomChestNum) {
            case 0: // left
              printNeoPixel(17, 30, pixels.Color(255, 0, 0));
              pixels.show();
              motorSwingCW(motorFastSpeed);
              if (armSW) {
                healthBarMachine();
                leftIndex ++;
              }
              else {
                left = true;
              }
              timerMachine();
              break;
            case 1: // top
              printNeoPixel(6, 17, pixels.Color(255, 0, 0));
              pixels.show();
              motorSwingCW(motorFastSpeed);
              if (armSW) {
                healthBarMachine();
                leftIndex ++;
              }
              else {
                top = true;
              }
              timerMachine();
              break;
            case 2: // bottom
              printNeoPixel(30, 38, pixels.Color(255, 0, 0));
              pixels.show();
              motorSwingCW(motorFastSpeed);
              if (armSW) {
                healthBarMachine();
                leftIndex ++;
              }
              else {
                bottom = true;
              }
              timerMachine();
              break;
            case 3: // right
              printNeoPixel(0, 6, pixels.Color(255, 0, 0));
              printNeoPixel(38, 45, pixels.Color(255, 0, 0));
              pixels.show();
              motorSwingCW(motorFastSpeed);
              if (armSW) {
                healthBarMachine();
                leftIndex ++;
              }
              else {
                right = true;
              }
              timerMachine();
              break;
            case 4: // head
              printNeoPixel(0, 45, pixels.Color(255, 255, 0));
              pixels.show();
              motorSwingCW(motorFastSpeed);
              if (armSW) {
                healthBarMachine();
                leftIndex ++;
              }
              else {
                headSensorFlag = true;
              }
              timerMachine();
              break;
          }
        }
        if (!armST && !armSW) {
          currentMicros = micros();
          while (((micros() - currentMicros) < 850000)) {
            
            timerMachine();
            
            if (headSensorFlag) {
              headSensorRead = analogRead(headSensorPin);
              if (headSensorRead > 900) {
                printNeoPixel(0, 45, pixels.Color(0, 0, 0));
                pixels.show();
                headSensorFlag = false;
                healthBarMachine();
                rightIndex --;
              }
            }
            else {
              chestSensor();
              if (leftPadDetect) {
                printNeoPixel(17, 30, pixels.Color(0, 0, 0));
                pixels.show();
                left = false;
                healthBarMachine();
                rightIndex --;
                break;
              }
              if (topPadDetect) {
                printNeoPixel(6, 17, pixels.Color(0, 0, 0));
                pixels.show();
                top = false;
                healthBarMachine();
                rightIndex --;
                break;
              }
              if (bottomPadDetect) {
                printNeoPixel(30, 38, pixels.Color(0, 0, 0));
                pixels.show();
                bottom = false;
                healthBarMachine();
                rightIndex --;
                break;
              } 
              if (rightPadDetect) {
                printNeoPixel(0, 6, pixels.Color(0, 0, 0));
                printNeoPixel(38, 45, pixels.Color(0, 0, 0));
                pixels.show();
                right = false;
                healthBarMachine();
                rightIndex --;
                break;
              }             
            }
          }
          if (!left || !top || !bottom || !right) {
            printNeoPixel(0, 45, pixels.Color(0, 255, 0));
            pixels.show();
            delay(1000);
          }
          pixels.clear();
          pixels.show(); 
        }
        if (raspiData == "straightPunch") {
          motorStraightCW(motorFastSpeed);
          armST = false;
          raspiData = "none";
        }
        if (raspiData == "swingPunch") {
          motorSwingCCW(motorFastSpeed);  
          armSW = false;
          raspiData = "none";
        }

        if (healthBarFlag || timerFlag) {
          robotState = gameOverState;
          stateIndex = 0;
        }

        if (raspiData == "pauseState") {

          robotState = pauseState;
          timerFlag = true;
                
        }  
      }
      break;

    case pauseState: //-----------------------------

//      motorSwingCCW(motorSlowSpeed);
//      motorStraightCW(motorSlowSpeed);

      pixels.clear();
      pixels.show();
            
      if (raspiData == "playState") {
        robotState = playState;
      }
      raspiData = "none";
      currentMicros = micros();
      neoPixelIdle();
      
      break;

    case gameOverState: //-----------------------------

//      motorSwingCCW(motorSlowSpeed);
//      motorStraightCW(motorSlowSpeed);

      pixels.clear();
      pixels.show();

      printNeoPixel1(0,20,pixels1.Color(0,255,0));
      pixels1.show();

      // send the winning data to raspi
      boxerHealth = 10 - leftIndex;
      robotHealth = rightIndex - 10;
      if (boxerHealth > robotHealth) {
        Serial.println("boxer win");
      }
      else if (boxerHealth < robotHealth) {
        Serial.println("robot win");
      }
      else {
        Serial.println("...");
      }
      
      if (raspiData == "playState") {
        robotState = playState;
      }
      raspiData = "none";
      currentMicros = micros();
      neoPixelIdle();
      
      break;

    default:
      robotState = idleState;
      break;
  }

}


// =================================================================================

// <<<Functions>>>

void timerMachine() {

  if (!timerFlag) {
    timerCurrent = millis();
    timerElapsed = (timerCurrent - timerStart) / 1000;
  }
  
  if (timerElapsed <= COUNTDOWN_TIME) {
    if (!timerFlag) {
      unsigned long remainingTime = COUNTDOWN_TIME - timerElapsed;
    }

    // Display remaining time in Minutes:Seconds format
    unsigned int minutes = remainingTime / 60;
    unsigned int seconds = remainingTime % 60;
    timer.showNumberDecEx(minutes * 100 + seconds, 0b01000000, true);         
  }
  else {
    timerFlag = true;
  }
}

// ----------- neoPixel for health bar -------------
void healthBarMachine() {

  if (currentMicros - previousNeoPixelMicros >= neoPixelTestInterval) {
    previousNeoPixelMicros = currentMicros;
    
    if (leftIndex < rightIndex) {
      pixels1.setPixelColor(leftIndex,pixels1.Color(0,0,0));
      pixels1.setPixelColor(rightIndex,pixels1.Color(0,0,0));
      pixels1.show();
    } else {
      healthBarFlag = true;
    }
  }
}


void healthBarTest() {

  if (currentMicros - previousNeoPixelMicros >= neoPixelTestInterval) {
    previousNeoPixelMicros = currentMicros;
    
    if (leftIndex < rightIndex) {
      pixels1.setPixelColor(leftIndex,pixels1.Color(0,0,0));
      pixels1.setPixelColor(rightIndex,pixels1.Color(0,0,0));
      pixels1.show();
      leftIndex++;
      rightIndex--;
    } else {
      leftIndex = 0;
      rightIndex = 19;
      healthBarFlag = true;
    }
  }
}

void printNeoPixel1(int firstPixel, int lastPixel, uint32_t color) {
  for (int i = firstPixel; i < lastPixel; i++) { // For each pixel in strip...
    pixels1.setPixelColor(i, color);              // Set pixel's color
  }
}


// ----------- neoPixel for chest pad -------------
void neoPixelIdle() {

  if (currentMicros - previousNeoPixelMicros >= neoPixelIdleInterval) {
    previousNeoPixelMicros = currentMicros;
    pixels.setPixelColor(neoPixelIndex, colorIdle[colorIndex]);
    neoPixelIndex ++;
    if (neoPixelIndex == neoPixelSize) {
      pixels.show();
      neoPixelIndex = 0;
      colorIndex ++;
      colorIndex = colorIndex % 18;
    }
  }
  
}

void printNeoPixel(int firstPixel, int lastPixel, uint32_t color) {
  for (int i = firstPixel; i < lastPixel; i++) { // For each pixel in strip...
    pixels.setPixelColor(i, color);          // Set pixel's color
  }
}

// -------- Chest Pad Sensor ---------

void chestSensor() {
    
  byte padSensorReading_A = digitalRead(padSensorInputPin_A);
  byte padSensorReading_B = digitalRead(padSensorInputPin_B) << 1;
  byte padSensorReading_C = digitalRead(padSensorInputPin_C) << 2;
  chestPadTestOutput = padSensorReading_A | padSensorReading_B | padSensorReading_C;

  switch (chestPadTestOutput) {

    case 0b110:
      if (!leftPadDetect) {
        leftPadDetect = true;
        Serial.println("1");
      }
      break;

    case 0b011:
      if (!bottomPadDetect) {
        bottomPadDetect = true;
        Serial.println("2");
      }
      break;

    case 0b100:
      if (!topPadDetect) {
        topPadDetect = true;
        Serial.println("3");
      }
      break;

    case 0b101:
      if (!rightPadDetect) {
        rightPadDetect = true;
        Serial.println("4");
      }
      break;

    default:
      leftPadDetect = false;
      bottomPadDetect = false;
      topPadDetect = false;
      rightPadDetect = false;
      break;
  }
  delay(25);
}


// -------- Swing Punch -----------

void motorSwingCW(int pulseDelay) {
  
    digitalWrite(straightENA,HIGH);
                           
    for(double stepCount = 0; stepCount < swingMotorSteps; stepCount++) { 

      digitalWrite(motorDIR,LOW);
      digitalWrite(swingENA,LOW);
      digitalWrite(motorPUL,LOW);
      delayMicroseconds(pulseDelay);
      digitalWrite(motorPUL,HIGH);
      delayMicroseconds(pulseDelay);

//      if (analogRead(swingArmSensor) > 810) {
//        armSW = true;
//        break;
//      }

    }
    
    delay(50);
    digitalWrite(swingENA,HIGH);
}


void motorSwingCCW(int pulseDelay) {
  
    digitalWrite(straightENA,HIGH);
                           
    for(double stepCount = 0; stepCount < swingMotorSteps; stepCount++) { 

      digitalWrite(motorDIR,HIGH);
      digitalWrite(swingENA,LOW);
      digitalWrite(motorPUL,LOW);
      delayMicroseconds(pulseDelay);
      digitalWrite(motorPUL,HIGH);
      delayMicroseconds(pulseDelay);

      if (digitalRead(swingArmSwitch)) {
        limitMotorSwingCW();
        break;
      }
      
//      if (analogRead(swingArmSensor) > 810) {
//        delay(3000);
//      }      
      
    }
    
    delay(50);
//    digitalWrite(swingENA,HIGH);
}

void limitMotorSwingCW() {
  
    digitalWrite(straightENA,HIGH);
                           
    for(double stepCount = 0; stepCount < 500; stepCount++) { 

      digitalWrite(motorDIR,LOW);
      digitalWrite(swingENA,LOW);
      digitalWrite(motorPUL,LOW);
      delayMicroseconds(50);
      digitalWrite(motorPUL,HIGH);
      delayMicroseconds(50);
      
    }
    
    delay(50);
    digitalWrite(swingENA,HIGH);
}

// --------- Straight Punch ----------

void motorStraightCW(int pulseDelay) {
  
    digitalWrite(swingENA,HIGH);
                           
    for(double stepCount = 0; stepCount < straightMotorSteps; stepCount++) { 

      digitalWrite(motorDIR,LOW);
      digitalWrite(straightENA,LOW);
      digitalWrite(motorPUL,LOW);
      delayMicroseconds(pulseDelay);
      digitalWrite(motorPUL,HIGH);
      delayMicroseconds(pulseDelay);

      if (digitalRead(straightArmSwitch)) {
        limitMotorStraightCCW();
        break;
      }

//      if (analogRead(straightArmSensor) > 900) {
//        delay(3000);
//      }
    }
    
    delay(50);
    digitalWrite(straightENA,HIGH);
}

void limitMotorStraightCCW() {

    digitalWrite(swingENA,HIGH);
                           
    for(double stepCount = 0; stepCount < 500; stepCount++) { 

      digitalWrite(motorDIR,HIGH);
      digitalWrite(straightENA,LOW);
      digitalWrite(motorPUL,LOW);
      delayMicroseconds(50);
      digitalWrite(motorPUL,HIGH);
      delayMicroseconds(50);
      
    }
    
    delay(50);
    digitalWrite(straightENA,HIGH);
}


void motorStraightCCW(int pulseDelay) {

    digitalWrite(swingENA,HIGH);
                           
    for(double stepCount = 0; stepCount < straightMotorSteps; stepCount++) { 

      digitalWrite(motorDIR,HIGH);
      digitalWrite(straightENA,LOW);
      digitalWrite(motorPUL,LOW);
      delayMicroseconds(pulseDelay);
      digitalWrite(motorPUL,HIGH);
      delayMicroseconds(pulseDelay);

//      if (analogRead(straightArmSensor) > 900) {
//        armST = true;
//        break;
//      }
      
    }
    
    delay(50);
}

// ------ Reset Variable Instance -------

void resetVariable() {
  
  stateFlag = false;
  
  armStraightFlag = false;
  armSwingFlag = false;
  armST = false;
  armSW = false;
  
  leftPadDetect = false;
  bottomPadDetect = false;
  topPadDetect = false;
  rightPadDetect = false;
  left = false;
  right = false;
  top = false;
  bottom = false;
  
  leftIndex = 0;                        
  rightIndex = 19;
  healthBarFlag = false;
  
  neoPixelIndex = 0;        
  colorIndex = 0;
  
  COUNTDOWN_TIME = 180;
  timerFlag = false;
  
  headSensorFlag = false;  
  
}

// ---------------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
