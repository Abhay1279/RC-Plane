#include<Servo.h>
int pin2;
int pin3;
int pin4;
Servo esc;
Servo servo1;
Servo servo2;
unsigned long previousMillis = 0;                           
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
const uint64_t pipeIn = 0xE8E8F0F0E1LL;     
RF24 radio(9, 10); 
struct MyData {
byte throttle;      /
byte pitch;
byte roll;
};
MyData data;
void resetData(){
data.throttle = 0;
data.pitch = 130;
data.roll = 130;
}
void setup()
{Serial.begin(250000);
  esc.attach(2); 
 servo1.attach(3);
servo2.attach(4);esc.writeMicroseconds(1000);
resetData();
radio.begin();
radio.setAutoAck(false);
radio.setDataRate(RF24_250KBPS);
radio.openReadingPipe(1,pipeIn);
radio.startListening();
}
unsigned long lastRecvTime = 0;
void recvData(){
while ( radio.available() ) {
radio.read(&data, sizeof(MyData));
lastRecvTime = millis(); 
}
}
void loop()
{
recvData();
unsigned long now = millis();
if ( now - lastRecvTime > 1000 ) {
// Signal lost?
resetData();
}
pin2 = map(data.throttle, 0, 255, 1000, 2000);     
pin3 = map(data.roll,    0, 255, 30, 130);     
pin4 = map(data.pitch,     0, 255, 30, 130);     
  esc.writeMicroseconds(pin2);
  delay(15);
  servo1.write(pin3);
  delay(15);
  servo2.write(pin4);
  delay(15);
  Serial.print("Throttle: "); Serial.print(pin2);  Serial.print("    ");
  Serial.print("Roll: ");     Serial.print(data.roll);      Serial.print("    ");
  Serial.print("Pitch: ");     Serial.print(data.pitch);      Serial.print("    ");
  Serial.print("\n");
 }
