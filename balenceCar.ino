#include <robomodule_due_CAN.h>
#include <PID_v1.h>
#include <JY901.h>
#include <Wire.h>
//#include "robomodule_due_CAN.h"
double Setpoint, Input, Output;
char s[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,00,0,0,0,0,0,0,0,0,0};
CRobomodule_due_CAN ocan1;
double Kp=14, Ki=0, Kd=0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int i;
int multp,multi,multd;
/*
http://item.taobao.com/item.htm?id=43511899945
Test on mega2560.
JY901   mega2560
TX <---> 0(Rx)
*/
void setup() 
{
  Serial2.begin(115200); 
  Serial.begin(9600); 
  Serial1.begin(115200);
  ocan1.initdriver(CAN_BPS_1000K,0,0,1);
  Input = 0;
  Setpoint = 0;
   myPID.SetOutputLimits(-5000,5000);
   myPID.SetSampleTime(10);
   myPID.SetMode(AUTOMATIC);
}

void loop() 
{
  //print received data. Data was received in serialEvent;
 // Serial.print("Time:20");Serial.print(JY901.stcTime.ucYear);Serial.print("-");Serial.print(JY901.stcTime.ucMonth);Serial.print("-");Serial.print(JY901.stcTime.ucDay);
 // Serial.print(" ");Serial.print(JY901.stcTime.ucHour);Serial.print(":");Serial.print(JY901.stcTime.ucMinute);Serial.print(":");Serial.println((float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);
               
 // Serial.print("Acc:");Serial.print((float)JY901.stcAcc.a[0]/32768*16);Serial.print(" ");Serial.print((float)JY901.stcAcc.a[1]/32768*16);Serial.print(" ");Serial.println((float)JY901.stcAcc.a[2]/32768*16);
  
 // Serial.print("Gyro:");Serial.print((float)JY901.stcGyro.w[0]/32768*2000);Serial.print(" ");Serial.print((float)JY901.stcGyro.w[1]/32768*2000);Serial.print(" ");Serial.println((float)JY901.stcGyro.w[2]/32768*2000);
  
  Serial2.print("Angle:");Serial2.print((float)JY901.stcAngle.Angle[0]/32768*180);Serial2.print(" ");Serial2.print((float)JY901.stcAngle.Angle[1]/32768*180);Serial2.print(" ");Serial2.println((float)JY901.stcAngle.Angle[2]/32768*180);

  //Serial.print("Mag:");Serial.print(JY901.stcMag.h[0]);Serial.print(" ");Serial.print(JY901.stcMag.h[1]);Serial.print(" ");Serial.println(JY901.stcMag.h[2]);
  
  //Serial.print("Pressure:");Serial.print(JY901.stcPress.lPressure);Serial.print(" ");Serial.println((float)JY901.stcPress.lAltitude/100);
  
  //Serial.print("DStatus:");Serial.print(JY901.stcDStatus.sDStatus[0]);Serial.print(" ");Serial.print(JY901.stcDStatus.sDStatus[1]);Serial.print(" ");Serial.print(JY901.stcDStatus.sDStatus[2]);Serial.print(" ");Serial.println(JY901.stcDStatus.sDStatus[3]);
  
 //Serial.print("Longitude:");Serial.print(JY901.stcLonLat.lLon/10000000);Serial.print("Deg");Serial.print((double)(JY901.stcLonLat.lLon % 10000000)/1e5);Serial.print("m Lattitude:");
  //Serial.print(JY901.stcLonLat.lLat/10000000);Serial.print("Deg");Serial.print((double)(JY901.stcLonLat.lLat % 10000000)/1e5);Serial.println("m");
  
 // Serial.print("GPSHeight:");Serial.print((float)JY901.stcGPSV.sGPSHeight/10);Serial.print("m GPSYaw:");Serial.print((float)JY901.stcGPSV.sGPSYaw/10);Serial.print("Deg GPSV:");Serial.print((float)JY901.stcGPSV.lGPSVelocity/1000);Serial.println("km/h");
  
 // Serial.print("SN:");Serial.print(JY901.stcSN.sSVNum);Serial.print(" PDOP:");Serial.print((float)JY901.stcSN.sPDOP/100);Serial.print(" HDOP:");Serial.print((float)JY901.stcSN.sHDOP/100);Serial.print(" VDOP:");Serial.println((float)JY901.stcSN.sVDOP/100);
  
  //Serial2.println("");
  delay(10);
 if(Serial2.available()){
    for(i=0;i<24;i++)
    {
      while(Serial2.available()==0);
      s[i]=Serial2.read();
    }
    /*s[0]=Serial2.read();
    while(Serial2.available()==0);
    s[1]=Serial2.read();
    while(Serial2.available()==0);
    s[2]=Serial2.read();
    while(Serial2.available()==0);
    s[3]=Serial2.read();
    while(Serial2.available()==0);
    s[4]=Serial2.read();
    while(Serial2.available()==0);
    s[5]=Serial2.read();
    while(Serial2.available()==0);
    s[6]=Serial2.read();
    while(Serial2.available()==0);
    s[7]=Serial2.read();
    while(Serial2.available()==0);
    s[8]=Serial2.read();
    while(Serial2.available()==0);
    s[9]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[11]=Serial2.read();
    while(Serial2.available()==0);
    s[12]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();
    while(Serial2.available()==0);
    s[10]=Serial2.read();*/
  //  while(Serial2.available()==0);
  //  s[11]=Serial2.read();
  //  while(Serial2.available()==0);
  //  s[12]=Serial2.read();
   // while(Serial2.available()==0);
   // s[13]=Serial2.read();
   // while(Serial2.available()==0);
  //  s[14]=Serial2.read();
    multp=(s[12]-48)*1000+(s[13]-48)*100+(s[14]-48)*10+s[15];
    multi=(s[16]-48)*1000+(s[17]-48)*100+(s[18]-48)*10+s[19];
    multd=(s[20]-48)*1000+(s[21]-48)*100+(s[22]-48)*10+s[23];
    Kp=((s[0]-48)+(s[1]-48)/10+(s[2]-48)/100.0+(s[3]-48)/1000.0)*multp;
    Ki=((s[4]-48)+(s[5]-48)/10+(s[6]-48)/100.0+(s[7]-48)/1000.0)*multi;
    Kd=((s[8]-48)+(s[9]-48)/10+(s[10]-48)/100.0+(s[11]-48)/1000.0)*multd;
    myPID.SetTunings(Kp,Ki,Kd);
  }
  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
  Input = JY901.stcAngle.Angle[1];
  myPID.Compute();
  ocan1.speedwheel(Output,0,0);
  Serial2.print(Kp);
  Serial2.print(',');
  Serial2.print(Ki);
  Serial2.print(',');
  Serial2.print(Kd);

  Serial2.print("Output:");Serial2.print(Output);
  Serial2.print("INput:");Serial2.println(Input);
}

