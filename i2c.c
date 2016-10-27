
// Using the Wire.h library as the API for the I2C protocol
#include <Wire.h>

// Defining the Arduino slave address
#define SLAVE_ADDRESS 0x04

// Defining buffers for incoming and outcoming data
String __data = "";
String __parsedData = "";

void dataHandler(int byteCount){
// Data handler for the onReceive callback
// This is where in incoming bytes are processed into strings

  Serial.println(F("[API ] API call received"));

  if (byteCount == 1){
  // byteCount = 1 implies that it is a GET request (one checksum byte)
  // TODO: Verify the checksum byte before processing sendData()
  
    Serial.println(F("[GET ] GET request received, reading and dumping..."));

    // All bytes from the I2C stream needs to be read (otherwise it will block)
    while(Wire.available()){
      Wire.read();
    }
    // Break the function after stream is read
    return;
  }
  
    
  Serial.println(F("[POST] POST request received"));
  __data = "";
  int __count = 0;

  // Read the incoming stream 
  while(Wire.available()){
    if (__count < byteCount){
      __data += (char)Wire.read();
      __count++;
    }
    // Fallback for corrupted stream
    else {
      __count = 0;
      __data += (char)Wire.read();
    }
  }

  // Truncuate the checksum byte
    __data = __data.substring(1);
  // Pass the string over to the API handler
    __apiHandler(__data);
}


void __apiHandler(String __data){
// API handler for the onReceive callback
// This is where the analog values of the requested pin are read

  // We only have 4 analog pins to use, anything out of range is an invalid request
  if (0 <= __data.toInt() <= 4){
    Serial.print(F("[GET ] Pi is requesting data from pin "));
    Serial.println(__data.toInt());
    __parsedData = String(analogRead(__data.toInt()));
  }
  else{
    __parsedData = String(F("Invalid request"));
  }
  return;
}


void sendData(){
// The callback function for onRequest
  Wire.write(__parsedData.c_str());
  Serial.print(F("[GET ] Sending packet: "));
  Serial.println(__parsedData.c_str());
  return;
}

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(dataHandler);
  Wire.onRequest(sendData);
  Serial.begin(9600);
  Serial.println(F("Started!"));
}
  
void loop() {
    delay(1000);

