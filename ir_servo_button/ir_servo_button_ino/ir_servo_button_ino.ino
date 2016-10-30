/* Function to actuate servo based on IR sensor output data, with button*/

#include <Servo.h> // Include servo operation library

int sensorPin = 0; //analog pin 0
int pos = 0; // variable to store sensor position
const int buttonPin = 1; // pin button is connected to
Servo myservo; // create servo object
int buttonState = 0;
int flag = 0; //  Initialise buttonState and flag variables to 0.

void setup(){
	myservo.attach(9); // attaches servo on pin 9 to servo library
	Serial.begin(9600); // Begin comms via Serial at 9600Hz
        pinMode(buttonPin, INPUT_PULLUP);
}

void loop(){
        buttonState = digitalRead(buttonPin); // Pressed or not pressed?
        
        if (buttonState == LOW){ //pressed...
        
          if (flag == 0){
            flag=1
    	    if (analogRead(sensorPin) < 100){ //  If reflectivity < 100
    		for (pos = 0; pos <= 180; pos += 1){ //  from 0 to 180 degrees
    			myservo.write(pos); // go to position
    			delay(5);
    		}
    	     }
    	    if (analogRead(sensorPin) > 100){ // If reflectivity > 100
    		for (pos = 180; pos >= 0; pos -= 1){ // As above
    			myservo.write(pos);
    			delay(5);
    		  }		 
    	        }
            }
           else if (flag == 1){
            flag=0;
            }
        }
}
