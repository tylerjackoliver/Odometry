#include <Wire.h>                                             // Calls for I2C bus library
#include <Servo.h>                                            // Calls for Servo library

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

// DECLARATIONS HERE! //
int DualSpeedValue = 0;                                       // Combined motor speed variable
int Mode           = 2;                                       // MODE in which the MD25 will operate selector value
const int ledPin   = 7;                                       // Pin for the LED   
const int servoPin = 9;                                       // Pin for the Servo  
int pos = 0;
int currentPos = 0;
int currentPosBack =0;

float Wheel_1_Distance_MM = 0;                                // Wheel 1 travel distance variable
float Wheel_2_Distance_MM = 0;                                // Wheel 2 travel distance variable

float prevOvershootDistance = 0;                              // Robot overshoot distance
float nextOvershootDistance = 0;                              // Robot overshoot distance
float OvershootDistance     = 0;                              // Robot overshoot distance
float prevOvershootAngle    = 0;                              // Robot overshoot angle
float nextOvershootAngle    = 0;                              // Robot overshoot angle
float OvershootAngle        = 0;                              // Robot overshoot angle
float ledPinHighStatus      = 0;                              // Record whether the LED pin is on or not
float CF                    = 1.0;                              // Correction factor for the robot      
unsigned long time;                                           // Variable to hold the time for millis() functions


Servo dropServo;                                              // Define servo for Arduino  

void setup(){
        Wire.begin();
        // Begin I2C bus
        Serial.begin(9600);                                   // Begin serial
        delay(500);                                           // Wait for everything to power up
        Wire.beginTransmission(MD25ADDRESS);                  // Set MD25 operation MODE
        Wire.write(MODE_SELECTOR);
        Wire.write(Mode);
        Wire.endTransmission();

        encodeReset();                                        // Cals a function that resets the encoder values to 0
        dropServo.attach(servoPin);                           // Attaches servo so it can be used.
        pinMode(ledPin, OUTPUT);                              // Define the LED as an output so that it can light    
}


void move_forward(float x, int DualSpeedValue){               // DEPENDENCY FUNCTION, DO NOT CALL DIRECTLY: This function moves the platform forward by a distance of 'x'mm
        Wheel_1_Distance_MM = x;
        Wheel_2_Distance_MM = x;
        encoder1();                                           // Calls a function that reads value of encoder 1
        encoder2();                                           // Calls a function that reads value of encoder 2

        while (abs(encoder1()) <= abs(Wheel_1_Distance_MM)) { // If statement to check the status of the traveled distance

                Wire.beginTransmission(MD25ADDRESS);          // Sets the acceleration to register 3
                Wire.write(ACCELERATION);
                Wire.write(3);
                Wire.endTransmission();

                Wire.beginTransmission(MD25ADDRESS);          // Sets a combined motor speed value
                Wire.write(SPEED1);
                Wire.write(DualSpeedValue);
                Wire.endTransmission();
        }
}

void turn(float x, int DualSpeedValue){                       // DEPENDENCY FUNCTION, DO NOT CALL DIRECTLY: This function turns the platform right by a previously set angle
        Wheel_1_Distance_MM = x;
        Wheel_2_Distance_MM = -x;
        encoder1();                                           // Calls a function that reads value of encoder 1
        encoder2();                                           // Calls a function that reads value of encoder 2

        while (abs(encoder1()) <= abs(Wheel_1_Distance_MM)) { // If statement to check the status of the traveled distance

                Wire.beginTransmission(MD25ADDRESS);          // Sets the acceleration to register 5
                Wire.write(ACCELERATION);
                Wire.write(5);
                Wire.endTransmission();

                Wire.beginTransmission(MD25ADDRESS);          // Sets a combined motor speed value
                Wire.write(SPEED2);
                Wire.write(DualSpeedValue);
                Wire.endTransmission();
        }
}

void encodeReset(){                                           // This function resets the encoder values to 0
        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(CMD);
        Wire.write(0x20);
        Wire.endTransmission();
        delay(25);
}

float encoder1(){                                             // Function to read and display value of encoder 1 as a long
        Wire.beginTransmission(MD25ADDRESS);                  // Send byte to get a reading from encoder 1
        Wire.write(ENCODERONE);
        Wire.endTransmission();

        Wire.requestFrom(MD25ADDRESS, 4);                     // Request 4 bytes from MD25
        while(Wire.available() < 4) ;                         // Wait for 4 bytes to arrive
        long poss1 = Wire.read();                             // First byte for encoder 1, HH.
        poss1 <<= 8;
        poss1 += Wire.read();                                 // Second byte for encoder 1, HL
        poss1 <<= 8;
        poss1 += Wire.read();                                 // Third byte for encoder 1, LH
        poss1 <<= 8;
        poss1  +=Wire.read();                                 // Fourth byte for encoder 1, LLalue
        delay(5);                                             // Wait for everything to make sure everything is sent
        return(poss1*0.093);                                  // Convert encoder value to mm
}

float encoder2(){                                             // Function to read and display velue of encoder 2 as a long
        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(ENCODERTWO);
        Wire.endTransmission();

        Wire.requestFrom(MD25ADDRESS, 4);                     // Request 4 bytes from MD25
        while(Wire.available() < 4) ;                         // Wait for 4 bytes to become available
        long poss2 = Wire.read();                             // First byte for encoder 2, HH
        poss2 <<= 8;
        poss2 += Wire.read();                                 // Second byte for encoder 2, HL
        poss2 <<= 8;
        poss2 += Wire.read();                                 // Third byte for encoder 2, LH
        poss2 <<= 8;
        poss2  +=Wire.read();                                 // Fourth byte for encoder 2, LLalue
        delay(5);                                             // Wait to make sure everything is sent
        return(poss2*0.093);                                  // Convert encoder value to mm
}

void stopMotor(){                                             // Function to stop motors
        Wire.beginTransmission(MD25ADDRESS);                  // Sets the acceleration to register 3 (0.65s)
        Wire.write(ACCELERATION);
        Wire.write(3);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS);                  // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
        Wire.write(SPEED1);
        Wire.write(128);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS);                  // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
        Wire.write(SPEED2);
        Wire.write(128);
        Wire.endTransmission();

}

void mod_forward(float x, float CF){
        if (x > 0) {                                          // Detects if movement is a forward movement
                move_forward(x*CF, 255);
        }
        else {                                                // Detects if movement is a backward movement
                move_forward(x*CF, 0);
        }
        stopMotor();                                          // Stops the motor
        Serial.println("Motor Stopped");
        StationaryCheck();                                    // Waits for robot to come to a stop
        Serial.println("Robot has stopped");
        prevOvershootDistance = (x - encoder1());             // Calculate the overshoot distance
        Serial.print("First overshoot: ");
        Serial.println(prevOvershootDistance);
        encodeReset();                                        // Resets the encoders, this is to prepare for the robot to correct itself
        forwardCorrect(prevOvershootDistance);

        // Below simply corrects is again, minimising error
        stopMotor();
        Serial.println("Motor Stopped");
        StationaryCheck();
        Serial.println("Robot has stopped");
        nextOvershootDistance = (prevOvershootDistance - encoder1());
        Serial.print("Second overshoot: ");
        Serial.println(nextOvershootDistance);
        forwardCorrect(nextOvershootDistance);

        stopMotor();                                          // Stops the robot
        encodeReset();                                        // Resets encoders to prepare robot for the next movement
}

void mod_rotate(float r, float CF){
        float s = r * 0.1875;                                 // Calculates the distance, s, required to travel by one wheel to make angle r
        if (r > 0) {                                          // Determines if the rotation is clockwise
                turn(s*CF, 255);
        }
        else {                                                // Determines if the rotation is counter-clockwise
                turn(s*CF, 0);
        }
        stopMotor();                                          // Stops the motor
        StationaryCheck();
        prevOvershootAngle = (s - encoder1());                // The Overshoot Angle is calculated and printed
        Serial.print("First Overshoot Angle = ");
        Serial.println(prevOvershootAngle);
        encodeReset();
        rotateCorrect(prevOvershootAngle);

        stopMotor();
        StationaryCheck();
        nextOvershootAngle = (prevOvershootAngle - encoder1());
        Serial.print("Second Overshoot Angle = ");
        Serial.println(prevOvershootAngle);
        rotateCorrect(nextOvershootAngle);

        stopMotor();
        encodeReset();
}

void forwardCorrect(float fwd_overshoot){
        if (fwd_overshoot > 0) {                              // If the overshoot distance is positive [OVERDRIVE]
                move_forward(fwd_overshoot*0.975, 135);       // Compensate by moving at speed 135 (+7)
        }
        if (fwd_overshoot < 0) {                              // If the overshoot distance is positive [UNDERDRIVE]
                move_forward(fwd_overshoot*0.975, 121);       // Compensate by moving at speed 121 (-7)
        }
}

void rotateCorrect(float rotate_overshoot){
        if (rotate_overshoot > 0) {                           // If the overshoot distance is positive [OVERDRIVE]
                turn(rotate_overshoot*0.99, 130);             // Compensate by moving at speed 135 (+7)
        }
        if (rotate_overshoot < 0) {                           // If the overshoot distance is positive [UNDERDRIVE]
                turn(rotate_overshoot*0.99, 126);             // Compensate by moving at speed 121 (-7)
        }
}

void StationaryCheck(){                                       // Function to check if the robot is stationary
        float previous_encoder_count = encoder1();
        delay(50);
        float current_encoder_count = encoder1();
        if (abs(current_encoder_count) > abs(previous_encoder_count)) {
                StationaryCheck();
        }
}

void LightUp(){                                              // Light up an LED at waypoint  
        if (ledPinHighStatus = 1) {                           // if LED is on, turn it off
                digitalWrite(ledPin, LOW);
                ledPinHighStatus = 0;                         // Record LED AS OFF  
        }
        else if (ledPinHighStatus = 0){                       // if LED is on, pulse  
                digitalWrite(ledPin, HIGH);
                ledPinHighStatus = 1;
                delay(10);
                digitalWrite(ledPin, LOW);
                ledPinHighStatus = 0;

        }        
}

void drop(){                                                 // Drop the assorted un-named candy  

        StationaryCheck();                                    // Make sure robot is stationary
        currentPos = analogRead(servoPin);                    // Grab the current value of servo position
        if (currentPos != 0){                                 // If current position not 0, make it zero  
                for (pos = currentPos; pos >= 0; pos-=1){      // Move the servo, yo
                        dropServo.write(pos);
                }
        }  
        else if (currentPos < 110){                           // Nove to the required position
                for (pos = currentPos; pos <= 110; pos+=1){
                        dropServo.write(pos);
                }
        }

}

void dispenserBack(){                                         // Move the servo back to the original position

        currentPosBack = analogRead(servoPin);                // Read the current position of the servo
        for (pos = currentPosBack; pos <= 110; pos -=1){
                dropServo.write(pos);                          // Move that servo, gurl. Werk it.  
        }

}



//* MAIN SCRIPT FOR THE COURSE *//


void loop(){
  drop();
  mod_forward(100, CF);                                        // Traverse the first part of the course
  mod_rotate(90, CF);                                          // Rotate the first corner 
  drop();
  LightUp();
  dispenserBack();
  mod_forward(50,CF);
  mod_rotate(-90,CF);
  drop();
  LightUp();
  dispenserBack();
}
