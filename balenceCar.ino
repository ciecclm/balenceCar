#include <robomodule_due_CAN.h>
#include <PID_v1.h>
#include <JY901.h>
#include <Wire.h>
//#include <eeprom.h>
//#include "robomodule_due_CAN.h"
double Setpoint, Input, Output;
char s[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,00,0,0,0,0,0,0,0,0,0};
char tempsp[7]="";
char tempsi[7]="";
char tempsd[7]="";
char tempsangle[7]="";
CRobomodule_due_CAN ocan1;
double Kp=320, Ki=0, Kd=1.6;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int i;
int multp,multi,multd;

int WindowSize = 5;
unsigned long windowStartTime;
struct Kpid
{
  double Kp;
  double Ki;
  double Kd;
};
union kpid_data
{
  struct Kpid m_Kpid;
  byte k[sizeof(double)*3];
}m_pid;

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
  for( i=0;i<sizeof(double)*3;i++)
  {
    // m_pid.k[i]=EEPROM.write(i);
  }
  /*
  Kp=m_pid.m_Kpid.Kp;
  Ki=m_pid.m_Kpid.Ki;
  Kd=m_pid.m_Kpid.Kd;*/
  myPID.SetOutputLimits(-5000,5000);
  myPID.SetSampleTime(5);
  myPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
   
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
  //delay(10);
 if(Serial2.available()){
    for(i=0;i<24;i++)
    {
      while(Serial2.available()==0);
      s[i]=Serial2.read();
      if(i<6)
      {
        tempsp[i]=s[i];
      }
      if((i>=6)&&(i<12))
      {
        tempsi[i-6]=s[i];
      }
      if((i>=12)&&(i<18))
      {
        tempsd[i-12]=s[i];
      }
      if((i>=18)&&(i<24))
      {
        tempsangle[i-18]=s[i];
      }
    }
    
    Kp=atof(tempsp)*10;
    Ki=atof(tempsi);
    Kd=atof(tempsd);
    
    Setpoint=atof(tempsangle);
    myPID.SetTunings(Kp,Ki,Kd);
    m_pid.m_Kpid.Kp=Kp;
    m_pid.m_Kpid.Ki=Ki;
    m_pid.m_Kpid.Kd=Kd;
    for( i=0;i<sizeof(double)*3;i++)
    {
      //EEPROM.write(i,m_pid.k[i]);
    }
    
  }
  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
  Input = JY901.stcAngle.Angle[1]/32768.0*180+2.8;
  myPID.Compute();
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime=millis();
    myPID.Compute();
    ocan1.speedwheel(Output+2000,0,0);
    
    if(Input<3 && Input>-3)
               myPID.SetTunings(160,10,0.8);
          else
               myPID.SetTunings(240,0,1.2);    
  }
  
  Serial2.print(Kp);
  Serial2.print(',');
  Serial2.print(Ki);
  Serial2.print(',');
  Serial2.print(Kd);

  Serial2.print("Output:");Serial2.print(Output);
  Serial2.print("INput:");Serial2.println(Input);
}

