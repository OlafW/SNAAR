#include <OSCMessage.h>
#include <SLIPEncodedSerial.h>

SLIPEncodedSerial SLIPSerial(Serial);

const int hallSensor = 25;  // lolin32
const bool plotSerial = false; 

void setup() {
  SLIPSerial.begin(921600);
  pinMode(hallSensor, INPUT);
}

void loop() {
  int val = analogRead(hallSensor);

  // float voltage = val / 1023.0;  //uno
  
  float voltage = val / 4096.0; //lolin32
  voltage = voltage * 2.0 - 1.0;

  if (plotSerial) {
    Serial.print(-1);
    Serial.print(' ');
    Serial.print(1);
    Serial.print(' ');
    Serial.println(voltage);
  } 
  else {
    OSCMessage msg("/hall");
    msg.add(voltage);
    SLIPSerial.beginPacket();
    msg.send(SLIPSerial);
    SLIPSerial.endPacket();
    msg.empty();
  }
}
