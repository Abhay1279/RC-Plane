#include <Wire.h>
#include <Servo.h>
//#include<EEPROM.h>
          //RFID
Servo esc;
Servo pitch_servo;
Servo yaw_servo;
Servo roll_servoL;
Servo roll_servoR;

unsigned long previousMillis = 0;
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
const uint64_t pipeIn = 0xE8E8F0F0E1LL;     

RF24 radio(9, 10); 

struct MyData {
byte throttle;      
byte pitch;
byte roll;
byte yaw;
byte TPU;
byte TPD;
byte TRR;
byte TRL;
};
MyData data;

void resetData()
{
//We define the inicial value of each data input
//3 potenciometers will be in the middle position so 127 is the middle from 254
data.throttle = 0;
data.pitch = 130;
data.roll = 130;
data.yaw=130;
data.TPU=0;
data.TPD=0;
data.TRR=0;
data.TRL=0;
}






/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];



float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;

float errorpitch,errorroll,pwmpitch, pwmroll,PIDpitch,PIDroll, previous_errorpitch,previous_errorroll;
float pidpitch_p=0;
float pidpitch_i=0;
float pidpitch_d=0;
float pidroll_p=0;
float pidroll_i=0;
float pidroll_d=0;
double kp=1.5;
double ki=0;
double kd=0;
int throttle,yaw;
double pitch;
double roll;
float tpu,tpd,trr,trl,pitchTrim,rollTrim;
float desired_angle = 0; /
                         
void setup() {
  Wire.begin(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  esc.attach(2);
  roll_servoL.attach(3);
  roll_servoR.attach(4);
   pitch_servo.attach(5);
   yaw_servo.attach(6);
  esc.writeMicroseconds(1000);
  delay(2000);
 resetData();
radio.begin();
radio.setAutoAck(false);

radio.setDataRate(RF24_250KBPS);
radio.openReadingPipe(1,pipeIn);
radio.startListening();
  time = millis(); 
}
unsigned long lastRecvTime = 0;
void recvData(){
while ( radio.available() ) {
radio.read(&data, sizeof(MyData));
lastRecvTime = millis();
}
}
void loop() {
  recvData();
      timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    if ( time - lastRecvTime > 1000 ) {
resetData();
}
throttle = map(data.throttle, 0, 255, 1000, 2000);     
pitch = map(data.pitch,    0, 255, 30, 140);     
roll = map(data.roll,     0, 255, 30, 140);     
yaw = map(data.yaw,     0, 255, 30, 140);
tpu=data.TPU;
tpd=data.TPD;
trr=data.TRR;
trl=data.TRL;
if (tpu==1){
  pitchTrim=pitchTrim+0.1;
  if(pitchTrim>50)
  {
    pitchTrim=50;
  }
}
if(tpd==1)
{
  pitchTrim=pitchTrim-0.1;
 if(pitchTrim<-50)
 {
  pitchTrim=-50;
 }
}
if (tpu==0||tpd==0)
{
  pitchTrim=pitchTrim+0;
}

if(trr==1)
{
  rollTrim=rollTrim+0.1;
  if(rollTrim>50)
  {
    rollTrim=50;
  }
}
if(trl==1)
{
  rollTrim=rollTrim-0.1;
  if(rollTrim<-50)
  {
    rollTrim=-50;
  }
}
if(trr==0||trl==0)
{
  rollTrim=rollTrim+0;
}


     Wire.beginTransmission(0x68);
     Wire.write(0x3B); 
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 

     Acc_rawX=Wire.read()<<8|Wire.read(); 
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   Wire.beginTransmission(0x68);
   Wire.write(0x43); 
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); 
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); 
   Gyr_rawY=Wire.read()<<8|Wire.read();

   Gyro_angle[0] = Gyr_rawX/131.0; 
   Gyro_angle[1] = Gyr_rawY/131.0;

   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   

errorpitch = Total_angle[1] - desired_angle;
errorroll = Total_angle[0] - desired_angle;


pidpitch_p = kp*errorpitch;
pidroll_p=kp*errorroll;

if(-3 <errorpitch <3)
{
  pidpitch_i = pidpitch_i+(ki*errorpitch);  
}

if(-3 <errorroll <3)
{
  pidroll_i = pidroll_i+(ki*errorroll);  
}


pidpitch_d = kd*((errorpitch - previous_errorpitch)/elapsedTime);

pidroll_d = kd*((errorroll - previous_errorroll)/elapsedTime);

PIDpitch = pidpitch_p + pidpitch_i + pidpitch_d;

PIDroll = pidroll_p + pidroll_i + pidroll_d;



{
  PIDpitch=-140;
}
if(PIDpitch > 140)
{
  PIDpitch=140;
}

if(PIDroll < -140)
{
  PIDroll=-140;
}
if(PIDroll > 140)
{
  PIDroll=140;
}

pwmpitch = pitch - PIDpitch-pitchTrim;
pwmroll = roll - PIDroll+rollTrim;


if(pwmpitch < 30)
{
  pwmpitch= 30;
}
if(pwmpitch > 140)
{
  pwmpitch=140;
}
if(pwmroll < 30)
{
  pwmroll= 30;
}
if(pwmroll > 140)
{
  pwmroll=140;
}

esc.writeMicroseconds(throttle);
pitch_servo.write(pwmpitch);
yaw_servo.write(yaw);
roll_servoL.write(140-pwmroll+30);
roll_servoR.write(140-pwmroll+30);
previous_errorpitch = errorpitch; 
previous_errorroll = errorroll;
Serial.print("PitchTrim: ");  Serial.print(pitchTrim);      Serial.print("    ");
Serial.print("RollTrim: ");   Serial.print(rollTrim);       Serial.print("    ");
Serial.print("AnglePitch");   Serial.print(errorpitch); Serial.print("    ");
Serial.print("AngleRoll");    Serial.print(errorroll); Serial.print("    ");
Serial.print("Pitch");    Serial.print(pitch);       Serial.print("    ");
Serial.print("Roll");     Serial.print(roll);        Serial.print("    ");
Serial.print("PWM Pitch");    Serial.print(pwmpitch);       Serial.print("    ");
Serial.print("PWN Roll");     Serial.print(pwmroll);        Serial.print("    ");
Serial.print("\n");
  
}
