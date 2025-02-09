#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeOut = 0xE8E8F0F0E1LL; 

RF24 radio(9, 10); 
struct MyData {
  byte throttle;
  byte pitch;
  byte roll;   .
  };

MyData data;

void resetData() 
{
  data.throttle = 0;
  data.pitch = 127;
  data.roll = 127;
   
 }

void setup()
{
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.setPALevel(RF24_PA_MAX); 
  radio.stopListening();
  resetData();
 }

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void loop()
{
  data.throttle = mapJoystickValues( analogRead(A0), 13, 524, 1015, true );
  data.roll      = mapJoystickValues( analogRead(A1), 12, 544, 1021, true );
  data.pitch    = mapJoystickValues( analogRead(A2), 34, 522, 1020, true );
  radio.write(&data, sizeof(MyData));
}
