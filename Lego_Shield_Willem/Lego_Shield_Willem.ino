/*
  Lego Powerfunctions Shield made by Willem
  Arduino Uno controls motors and servo via ioexpander, input via bluetooth
 
  To do: Remote controlled with android app developed with MIT App Inventor 2
 
  Circuit:
  * Serial communication       (uses Uno pin 0,1)    Bluetooth-module is attached (has to be detached when loading program over USB)

  Motordriver connected directly to Arduino:           
  *  (first two pins: GND&GND = stop; 5V&5V = stop; GND&5V Direction 1; 5V&GND Direction 2, last pin: speed via PMW)

  * Lego3: Motor or Servo      (uses Arduino pins  7,  8,  9)
  * Lego4: Motor or Servo      (uses Arduino pins 13, 12,  6)
  * Lego1: Motor or Servo      (uses Arduino pins A0,  4, 10)
  * Lego2: Motor or Servo      (uses Arduino pins  2, A1, 11)

  Motordriver connected via IO Expander PCF8574P as IC1 at address 0X20
  * Port expander:             (uses Arduino pins A4 (SDA) & A5 (SCL)

  * Lego7: Motor or Servo      (uses IC1 pins  7, 6, and always high = always full speed)
  * Lego8: Motor or Servo      (uses IC1 pins  5, 4, and always high = always full speed)
  * Lego5: Motor or Servo      (uses IC1 pins  2, 3, and Arduino pin 5)
  * Lego6: Motor or Servo      (uses IC1 pins  1, 0, and Arduino pin 3) 

  * Motor speed 0..255
  * Servo position 0..255
*/

//*****************************************************************************************************//
// libraries:
//

  #include <SPI.h>                             // Serial Peripheral Interface Library
  #include <String.h>                          // Contains function strtok: split string into tokens
  #include <Wire.h>                            // For ioexpander

//*****************************************************************************************************//
// Constants
//

// Serial buffer size: calculate based on max input size expected for one command over bluetooth serial interface
  #define INPUT_SIZE 30

// IOExpander Address
  #define IOExpander_ADDR 0x20

// IO Expander Input Pins are fixed A4 (SDA) and A5 (SCL)

// if no pin used
  const int noPinUsed = 99;
  const int noIOExpanderused = 0;

// IOExpander Output Pins
  const int P0 = 0;
  const int P1 = 1;
  const int P2 = 2;
  const int P3 = 3;
  const int P4 = 4;
  const int P5 = 5;
  const int P6 = 6;
  const int P7 = 7;
 
// Motor Control Arduino Output pins:
  const int Lego3A = 7;
  const int Lego3B = 8;
  const int Lego3EN = 9;

  const int Lego4A = 13;
  const int Lego4B = 12;
  const int Lego4EN = 6;

  const int Lego1A = A0;
  const int Lego1B = 4;
  const int Lego1EN = 10;

  const int Lego2A = 2;
  const int Lego2B = A1;
  const int Lego2EN = 11;

// Lego 5-8 via IOExpander & Arduino
  // Lego7: P7, P6 & Always On
  const int Lego7A = P7;
  const int Lego7B = P6;
  const int Lego7EN = noPinUsed; // NOT DEFINED;

  // Lego8: P5, P4 & Always On
  const int Lego8A = P5;
  const int Lego8B = P4;
  const int Lego8EN = noPinUsed; // NOT DEFINED;

  // Lego5: P2, P3 & Arduino Pin 5
  const int Lego5A = P2;
  const int Lego5B = P3;
  const int Lego5EN = 5;
  
  // Lego6: P1, P0 & Arduino Pin 3
  const int Lego6A = P1;
  const int Lego6B = P0;
  const int Lego6EN = 3;

  const int legoControlMatrix[8][3] = { { Lego1A, Lego1B, Lego1EN },
                                        { Lego2A, Lego2B, Lego2EN },
                                        { Lego3A, Lego3B, Lego3EN },
                                        { Lego4A, Lego4B, Lego4EN },
                                        { Lego5A, Lego5B, Lego5EN },
                                        { Lego6A, Lego6B, Lego6EN },
                                        { Lego7A, Lego7B, Lego7EN },
                                        { Lego8A, Lego8B, Lego8EN } };
  
  const int legoSpeed = 255;

//*****************************************************************************************************//
// Variables
//

  // define C-string to hold bluetooth input command (add 1 byte for final 0)
  char inputCommand[INPUT_SIZE + 1];     // array of type char (C-string) to hold BlueTooth input
  int inputCommandByteCount;



//*****************************************************************************************************//
// Setup
//

void setup() 
{
  // Setup serial with BlueTooth
  Serial.begin(9600);                 // initialize serial communication PIN = 1234
  Serial.setTimeout(1000);            // 1000 ms time out test is 10 sec

  // Setup IOExpander
  Wire.begin();   // Start de I2C to IOExpander
   
  // Declare digital output pins and set them low:
  for (int i=0; i<4; i++){
    for (int j=0; j<2; j++){
      pinMode(legoControlMatrix[i][j], OUTPUT);
      digitalWrite(legoControlMatrix[i][j], LOW);
    }
    pinMode(legoControlMatrix[i][3], OUTPUT);
    analogWrite(legoControlMatrix[i][3], 0);
  }

  write8(IOExpander_ADDR, 0);

}

//*****************************************************************************************************//
// Main Loop
//

void loop() 
{
// put your main code here, to run repeatedly:
  
  // testMotors();          // Uncomment to test all motors
  // blueToothMotorTest();  // uncomment to test motor 3
  // blueToothtest();       // uncomment to test bluetooth connection via Serial monitor

  smartDelay(100); // to be able to still read serial signals
}

//*****************************************************************************************************//
// Functions
//

//*********************************************
// Test Functions
//

// Bluetooth Input test
void blueToothtest ()
{  
  // Print 
  inputCommand[0] = 0;
  inputCommandByteCount = readBTCommand(inputCommand, INPUT_SIZE);
  if (inputCommand[0] != 0){
    Serial.println(inputCommand);
  }
}

// Motor Test Protocol: drive and reverse each motor
void testMotors()
{
  // Set each motor
  int motorSpeed = 255;
  bool reverse = HIGH;
  SetMotor(noIOExpanderused, Lego1A, Lego1B, Lego1EN, motorSpeed, reverse);
  SetMotor(noIOExpanderused, Lego2A, Lego2B, Lego2EN, motorSpeed, reverse);
  SetMotor(noIOExpanderused, Lego3A, Lego3B, Lego3EN, motorSpeed, reverse);
  SetMotor(noIOExpanderused, Lego4A, Lego4B, Lego4EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego5A, Lego5B, Lego5EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego6A, Lego6B, Lego6EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego7A, Lego7B, Lego7EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego8A, Lego8B, Lego8EN, motorSpeed, reverse);
  
  smartDelay(2000);
  reverse = LOW;
  SetMotor(noIOExpanderused, Lego1A, Lego1B, Lego1EN, motorSpeed, reverse);
  SetMotor(noIOExpanderused, Lego2A, Lego2B, Lego2EN, motorSpeed, reverse);
  SetMotor(noIOExpanderused, Lego3A, Lego3B, Lego3EN, motorSpeed, reverse);
  SetMotor(noIOExpanderused, Lego4A, Lego4B, Lego4EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego5A, Lego5B, Lego5EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego6A, Lego6B, Lego6EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego7A, Lego7B, Lego7EN, motorSpeed, reverse);
  SetMotor(IOExpander_ADDR, Lego8A, Lego8B, Lego8EN, motorSpeed, reverse);

  smartDelay(2000);

}


// Bluetooth test
void blueToothMotorTest(){
    int BluetoothData=Serial.read();
    if(BluetoothData=='1'){   // if number 1 pressed ....
      SetMotor(noIOExpanderused, Lego3A, Lego3B, Lego3EN, 255, LOW);
      Serial.println("Motor is ON ! ");
    }
    if (BluetoothData=='0'){// if number 0 pressed ....
      SetMotor(noIOExpanderused, Lego3A, Lego3B, Lego3EN, 0, LOW);
      Serial.println("Motor is Off ! ");
    }
  smartDelay(100);
}


//*********************************************
// Bluetooth Functions
//

// Read BlueTooth Command
int readBTCommand(char* command, int maxSize){
  int commandByteCount;

  if (Serial.available()){
    //read Serial until new line or buffer full or time out
    commandByteCount = Serial.readBytesUntil('\n', command, maxSize);   
    // Add the final 0 to end the C-string
    command[commandByteCount] = 0;
  }
  return commandByteCount; // return size of the string
}

//*********************************************
// IOExpander Functions
//

// Read all 8 pins
uint8_t read8(int address)
{
  uint8_t data =0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
//  if(Wire.available()) {
    data = Wire.read();
//  }
  Wire.endTransmission();
  return data;
}

// Write all 8 pins 
void write8(int address, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(value);
  Wire.endTransmission();
}

// Read a single Pin
uint8_t readPin(int address, uint8_t pin)
{
  uint8_t data =0;
  data = read8(address);
  return (data & (1<<pin)) > 0;
}

// Write a single Pin
void writePin(int address, uint8_t pin, uint8_t value)
{
  uint8_t data =0; 
  data = read8(address);         // first read all pins
  if (value == LOW) 
  {
    data &= ~(1<<pin);           // now change 1 value
  }
  else 
  {
    data |= (1<<pin);            // now change 1 value
  }
  write8(address, data);         // write back all pins
}


//*********************************************
// Motor Funstions
//
// Set a single motor 
void SetMotor(int address, int pinA, int pinB, int pinEN, int motorSpeed, boolean reverse)
{
  if (address==noIOExpanderused)
  {
    ArduinoSetMotor(pinA, pinB, pinEN, motorSpeed, reverse);
  }
  else
  {
    IOExpanderSetMotor(address, pinA, pinB, pinEN, motorSpeed, reverse);
  }
}

// Set a single motor via Arduino
void ArduinoSetMotor(int pinA, int pinB, int pinEN, int motorSpeed, boolean reverse)
{
  analogWrite(pinEN, motorSpeed);
  digitalWrite(pinA, ! reverse);
  digitalWrite(pinB, reverse);
}

// Set a single motor via IOExpander
void IOExpanderSetMotor(int address, int pinA, int pinB, int pinEN, int motorSpeed, boolean reverse)
{
  if (pinEN == noPinUsed)
  {
    // No EN pin is used.
  } 
  else
  {
    analogWrite(pinEN, motorSpeed);
  }
  writePin(address, pinA, ! reverse);
  writePin(address, pinB, reverse);
}


//*********************************************
// Motor Funstions
//
// Enable smart delay for later expansions
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    // while (GPSSerial.available())    // example of reading serial during smart delay
    // gps.encode(GPSSerial.read());
  } while (millis() - start < ms);
}

