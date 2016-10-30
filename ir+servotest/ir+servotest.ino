/* Function to actuate servo based on IR sensor output data */

#include <Servo.h> // Include servo operation library

int sensorPin = 0; //analog pin 0
int pos = 0; // variable to store sensor position
Servo myservo; // create servo object

void setup() {
	myservo.attach(9); // attaches servo on pin 9 to servo library
	Serial.begin(9600); // Begin comms via Serial at 9600Hz
}

void loop() {

	for (analogRead(sensorPin) < 100) { //  If reflectivity < 100
		for (pos = 0; pos <= 180; pos += 1) { //  from 0 to 180 degrees
			myservo.write(pos); // go to position
			delay(15);
		}
	}

	for (analogRead(sensorPin) > 100) { // If reflectivity > 100

		for (pos = 180; pos >= 0; pos -= 1) { // As above
			myservo.write(pos);
			delay(15);
		}		

	}

}