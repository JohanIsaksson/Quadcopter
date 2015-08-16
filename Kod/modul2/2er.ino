#include <Wire.h>

void setup(){
  Wire.begin(2);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
}

void loop(){
  delay(100);
}


void requestEvent(){
  Wire.write("hello "); // respond with message of 6 bytes
                       // as expected by master
}


void receiveEvent(int howMany){
  while(0 < Wire.available()){
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
}