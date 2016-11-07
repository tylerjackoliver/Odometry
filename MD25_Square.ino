#include <Wire.h>                                             // Calls for I2C bus library

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

int DualSpeedValue = 0;                                       // Combined motor speed variable
int Mode = 2  ;                                                 // MODE in which the MD25 will operate selector value 
float Wheel_1_Distance_CM = 0;                                // Wheel 1 travel distance variable
float Wheel_2_Distance_CM = 0;                                // Wheel 2 travel distance variablef
float x                   = 0;                                // Distance travelled variable
//float DualSpeedValue      = 150;                              // Wheel speed value, forward/backward
float rotateDualSpeedValue= 150;                              // Wheel speed value, rotational
float r =  0;                                                 // Radius of the cirlce of the arc
float l = 0;                                                  // Length of the circle of the arc
float robotWidth = 0;                                         // Value for the width of the robot
float deg = 0;                                                // Value of degrees for the arc function
float ratio = 0;                                              // Initialise ratio variable for arc
float w2Speed = 0;                                            // Variable to change wheel 2 seed for the arc
float w1Speed = 0;                                            // Variable to change wheel 1 speed for the arc
int ledPulse = 0;                                             // Variable to count pulses of an LED

void setup(){
  Wire.begin();                                               // Begin I2C bus
  Serial.begin(9600);                                         // Begin serial
  delay(100);                                                 // Wait for everything to power up
  pinMode(7, OUTPUT);
  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation MODE
  Wire.write(MODE_SELECTOR);
  Wire.write(Mode);                                           
  Wire.endTransmission();

  encodeReset();                                              // Cals a function that resets the encoder values to 0 
}

void loop(){

  delay(2000);
  moveForward(38.5);
  lightLED();
  delay(20);
  turnRight(19.7);
  lightLED();
  moveForward(26);
  delay(2000);
  setMode(2);
  turnRight(60);
  lightLED();
  delay(2000);
  moveForward(51);
}
void forward(){                                               // This function moves the platform forward by a previously set distance
  
  encoder1();                                                 // Calls a function that reads value of encoder 1
  encoder2();                                                 // Calls a function that reads value of encoder 2

    if (encoder1() <= Wheel_1_Distance_CM && encoder2() <= Wheel_2_Distance_CM){     // If statement to check the status of the traveled distance
    
    Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
    Wire.write(SPEED1);
    Wire.write(DualSpeedValue);
    Wire.endTransmission();
    
    forward();
  }
}


void Right_Turn(){                                           // This function turns the platform right by a previously set angle
  encoder1();                                                // Calls a function that reads value of encoder 1
  encoder2();                                                // Calls a function that reads value of encoder 2

    if (encoder1() <= Wheel_1_Distance_CM && encoder2() >= Wheel_2_Distance_CM){     // If statement to check the status of the traveled distance
    
    Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(DualSpeedValue);
    Wire.endTransmission();
    
    Right_Turn();
  }
}

void Left_Turn(){
  encoder1();
  encoder2();
  setMode(2);
    if (abs(encoder1()) <= abs(Wheel_1_Distance_CM) && abs(encoder2()) <= abs(Wheel_2_Distance_CM)){
      Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();

      Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
      Wire.write(SPEED2);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();
      
      Wire.beginTransmission(MD25ADDRESS);
      Wire.write(SPEED1);
      Wire.write(106);
      Left_Turn();
  }
}
      


void encodeReset(){                                         // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                                         
  Wire.endTransmission(); 
  delay(50);
}



float encoder1(){                                           // Function to read and display value of encoder 1 as a long
  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                     // Fourth byte for encoder 1, LLalue
  delay(5);                                                 // Wait for everything to make sure everything is sent
  return(poss1*0.09);                                       // Convert encoder value to cm
}

float encoder2(){                                            // Function to read and display velue of encoder 2 as a long
  Wire.beginTransmission(MD25ADDRESS);           
  Wire.write(ENCODERTWO);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to become available
  long poss2 = Wire.read();                                 // First byte for encoder 2, HH
  poss2 <<= 8;
  poss2 += Wire.read();                                     // Second byte for encoder 2, HL             
  poss2 <<= 8;
  poss2 += Wire.read();                                     // Third byte for encoder 2, LH             
  poss2 <<= 8;
  poss2  +=Wire.read();                                     // Fourth byte for encoder 2, LLalue
  delay(5);                                                 // Wait to make sure everything is sent
  return(poss2*0.09);                                       // Convert encoder value to cm
}


void stopMotor(){                                           // Function to stop motors
  
  Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 10 (0.65s)
  Wire.write(ACCELERATION);
  Wire.write(10);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
  Wire.write(SPEED1);
  Wire.write(128);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
  Wire.write(SPEED2);
  Wire.write(128);
  Wire.endTransmission();
  delay(50);

  encoder1();                                                // Calls a function that reads value of encoder 1
  encoder2();                                                // Calls a function that reads value of encoder 2

  Serial.print("Encoder 1 Distance CM - ");                  // Displays las recorded traveled distance
  Serial.print(encoder1());
  Serial.print("   ");
  Serial.print("Encoder 2 Distance CM - ");
  Serial.print(encoder2());
  Serial.println(" ");
}  

void moveForward(float x){
  
  DualSpeedValue = 150;
  Wheel_1_Distance_CM = x;                                   // Sets wheel 1 travel distance value - cm, for upcoming forward motion function
  Wheel_2_Distance_CM = x;                                   // Sets wheel 1 travel distance value - cm, for upcoming forward motion function
  forward();                                                  // Cals a function that moves the platform forward
  stopMotor();                                                // Cals a function that stops the platform 
  encodeReset();                                              // Cals a function that resets the encoder values to 0 
  
}
void lightLED(){
  ledPulse = 0;
  while (ledPulse < 4){
   digitalWrite(7, HIGH);
   delay(100);
   digitalWrite(7, LOW);
   ledPulse += 1
   } 
}

void turnRight(float x){
  
  DualSpeedValue = 145;                                       // Sets a combined motor speed value for upcoming right turn function
  Wheel_1_Distance_CM = x;                                // Sets wheel 2 travel distance value - cm, for upcoming right turn motion function
  Wheel_2_Distance_CM = -x;                               // Sets wheel 2 travel distance value - cm, for upcoming right turn motion function
  Right_Turn();                                               // Cals a function that turn the platform right by 90 degrees
  stopMotor();                                                // Cals a function that stops the platform 
  encodeReset();
  
}

void turnLeft(float x){
    Wheel_1_Distance_CM = -x;
    Wheel_2_Distance_CM = x;
    Left_Turn();
    stopMotor();
    encodeReset();
}

void setMode(int Mode){
    Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation MODE
    Wire.write(MODE_SELECTOR);
    Wire.write(Mode);                                           
    Wire.endTransmission();
}

void turn(float x, int DualSpeedValue){                       // DEPENDENCY FUNCTION, DO NOT CALL DIRECTLY: This function turns the platform right by a previously set angle

        Wheel_1_Distance_CM = x;
        Wheel_2_Distance_CM = -x;
        encoder1();                                           // Calls a function that reads value of encoder 1
        encoder2();                                           // Calls a function that reads value of encoder 2

        while (abs(encoder1()) <= abs(Wheel_1_Distance_CM)) { // If statement to check the status of the traveled distance

                Wire.beginTransmission(MD25ADDRESS);          // Sets the acceleration to register 5
                Wire.write(ACCELERATION);
                Wire.write(1);
                Wire.endTransmission();

                Wire.beginTransmission(MD25ADDRESS);          // Sets a combined motor speed value
                Wire.write(SPEED2);
                Wire.write(DualSpeedValue);
                Wire.endTransmission();
        }
}

void turnRotate(float x, int DualSpeedValue){
          float s = x * 0.1025;                                 // Calculates the distance, s, required to travel by one wheel to make angle r
        if (x < 0) {                                          // Determines if the rotation is clockwise
                turn(s, 150);
        }
        else {                                                // Determines if the rotation is counter-clockwise
                turn(s, 128 - (DualSpeedValue - 128));
        }
        stopMotor();                                          // Stops the motor
        encodeReset();
}

void arc(float r, float l, float deg){                        // Move the robot in an arc         
        ratio = (r+robotWidth)/(r-robotWidth);                // Calculate the wheel speed ratio
        setMode(0);                                           // Set the mode to 0 to control each motor individually
        encoder1();
        encoder2();

        if (deg > 0){                                         // Turning left or turning right?
              //* SPEED1 < SPEED2 *//
          w1Speed = DualSpeedValue * ratio;                    // Set wheel speed off of nominal speed + ratio
          w2Speed = DualSpeedValue;                            // Set wheel speed 2 
          Wheel_1_Distance_CM = (r+w/2)*deg;
          Wheel_2_Distance_CM = (r-w/2_*deg);
}
        else if (deg < 0){
          w2Speed = DualSpeedValue * ratio;                    // Set wheel speed off of nominal speed + ratio
          w1Speed = DualSpeedValue;                            // Set wheel speed 2 
          Wheel_2_Distance_CM = (r+w/2)*deg;
          Wheel_1_Distance_CM = (r-w/2_*deg);       
}
          //* BEGIN TRANSMISSION OF WHEEL READINGS *//
        while (abs(encoder1()) <= abs(Wheel_1_Distance_CM) && abs(encoder2()) <= abs(Wheel_2_Distance_CM)){
          Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 1 (6.375s)
          Wire.write(ACCELERATION);
          Wire.digitalWrite(1);
          Wire.endTransmission();

          Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
          Wire.write(SPEED2);
          Wire.write(w2Speed);
          Wire.write(SPEED1);
          Wire.write(w1Speed);
          Wire.endTransmission();
}                                         
        stopmotor();
        encodeReset();
}
