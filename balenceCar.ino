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
double Kp=140, Ki=0, Kd=0.07;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int i;
int multp,multi,multd;

int WindowSize = 5;
int x_val, y_val, z_val;
//unsigned long windowStartTime;
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
#define NUMREADINGS 2 //Gyro noise filter
int readings[NUMREADINGS];                // the readings from the analog input (gyro)
//int index = 0;                            // the index of the current reading
int total = 0;                            // the running total
int average = 0;                          // the average
int gyroPin =A0;                          //Gyro Analog input

///////////////////////////////////
float dt = .05; // .06; //( 1024.0 * 256.0 ) / 16000000.0; (Kalman)  // what does this means ?? hand tuned
int mydt = 5; //in ms.
static float PP[2][2] = { //(Kalman)
  {
    1, 0   }
  , //(Kalman)
  {
    0, 1   }
  ,//(Kalman)
}; //(Kalman)


/*
 * Our two states, the angle and the gyro bias.  As a byproduct of computing
 * the angle, we also havef an unbiased angular rate available.   These are
 * read-only to the user of the module.
 */
float angle = 0.0; //(Kalman)
float q_bias = 0.0; //(Kalman)
float rate = 0.0; //(Kalman)
float q_m = 0.0; //(Kalman)

int ax_m=0;
int ay_m=0;
int cnt = 0;            //Counter
unsigned long lastread=0;
unsigned long startmillis=0;
float calibration =0.6; // read from a pot or something

/*
 * R represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect 0.3 rad jitter
 * from the accelerometer.
 */
float R_angle = .3; //.3 deafault, but is adjusted externally (Kalman)


/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the acceleromter
 * relative to the gyros.
 */
static const float Q_angle = 0.001; //(Kalman)
static const float Q_gyro = 0.003; //(Kalman)

float oldAngle=0.0;

int temp=HIGH;

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
  for (int i = 0; i < NUMREADINGS; i++)
      readings[i] = 0;                      // initialize all the readings to 0 (gyro average filter)
   startmillis = millis();     
     delay(250);
       
  myPID.SetOutputLimits(-5000,5000);
  myPID.SetSampleTime(5);
  myPID.SetMode(AUTOMATIC);
  //windowStartTime = millis();
  startmillis = millis();     
  delay(250);
   
}
int test_angle=0,count=0;
void loop() 
{
  //print received data. Data was received in serialEvent;
 // Serial.print("Time:20");Serial.print(JY901.stcTime.ucYear);Serial.print("-");Serial.print(JY901.stcTime.ucMonth);Serial.print("-");Serial.print(JY901.stcTime.ucDay);
 // Serial.print(" ");Serial.print(JY901.stcTime.ucHour);Serial.print(":");Serial.print(JY901.stcTime.ucMinute);Serial.print(":");Serial.println((float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);
               
  //Serial.print("Acc:");Serial.print((float)JY901.stcAcc.a[0]/32768.0*16);Serial.print(" ");Serial.print((float)JY901.stcAcc.a[1]/32768.0*16);Serial.print(" ");Serial.println((float)JY901.stcAcc.a[2]/32768.0*16);
  
 // Serial.print("Gyro:");Serial.print((float)JY901.stcGyro.w[0]/32768*2000);Serial.print(" ");Serial.print((float)JY901.stcGyro.w[1]/32768*2000);Serial.print(" ");Serial.println((float)JY901.stcGyro.w[2]/32768*2000);
  
  //Serial2.print("Angle:");Serial2.print((float)JY901.stcAngle.Angle[0]/32768*180);Serial2.print(" ");Serial2.print((float)JY901.stcAngle.Angle[1]/32768*180);Serial2.print(" ");Serial2.println((float)JY901.stcAngle.Angle[2]/32768*180);

 // Serial.print("Angle:");Serial.print((float)JY901.stcAngle.Angle[0]/32768.0*180);Serial.print(" ");Serial2.print((float)JY901.stcAngle.Angle[1]/32768.0*180);Serial2.print(" ");Serial2.println((float)JY901.stcAngle.Angle[2]/32768.0*180);
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
  /*
  Input = JY901.stcAngle.Angle[1]/32768.0*180+2.8;
  myPID.Compute();
  if (millis() - startmillis > dt)
  { //time to shift the Relay Window
    startmillis=millis();
    myPID.Compute();
    ocan1.speedwheel(Output+2000,0,0);
    
    if(Input<3 && Input>-3)
               myPID.SetTunings(160,10,0.8);
          else
               myPID.SetTunings(240,0,1.2);    
  }*/
  
    int delta = millis()-lastread;
    if( delta >= mydt) 
    {     
      // sample every dt ms -> 1000/dt hz.
          lastread = millis();  
         
          /*total -= readings[index];               // subtract the last gyro reading
          readings[index] = analogRead(gyroPin); // read from the gyro sensor
          total += readings[index];               // add the reading to the total
          index = (index + 1);                    // advance to the next index
          if (index >= NUMREADINGS)               // if we're at the end of the array...
                index = 0;                            // ...wrap around to the beginning
          average = (total / NUMREADINGS);    // calculate the average of gyro input*/
          
          average=analogRead(gyroPin);
          dt = ((float)delta) / 1000.0;
          q_m= ((float)average)*0.6*PI/180;  // HAC remove 1.5 mult
          /* Unbias our gyro */
          const float q = q_m - q_bias; //(Kalman)
          const float Pdot[2 * 2] =
          {
                 Q_angle - PP[0][1] - PP[1][0], /* 0,0 */ //(Kalman)   
                 - PP[1][1], /* 0,1 */
                 - PP[1][1], /* 1,0 */
                 Q_gyro /* 1,1 */
          };
          /* Store our unbias gyro estimate */
          rate = q; //(Kalman)
          /*
          * Update our angle estimate
          * angle += angle_dot * dt
          *       += (gyro - gyro_bias) * dt
          *       += q * dt
          */
          angle += q * dt; //(Kalman)
          /* Update the covariance matrix */
          PP[0][0] += Pdot[0] * dt; //(Kalman)
          PP[0][1] += Pdot[1] * dt; //(Kalman)
          PP[1][0] += Pdot[2] * dt; //(Kalman)
          PP[1][1] += Pdot[3] * dt; //(Kalman)
          // read here!
         // getxyz(x_val,y_val,z_val);
          /*  ax_m = z_val;
          ay_m = y_val;*/
          // non aggiusto R
          // R_angle= (joy_y_axis+1)*0.0098039; //external adjust jitter of accelerometer with nunchuck joystick
          //getxyz(x_val, y_val, z_val);
          x_val=(float)JY901.stcAcc.a[0];
          y_val=(float)JY901.stcAcc.a[1];
          z_val=(float)JY901.stcAcc.a[2];  
          const float angle_m = atan2(x_val,sqrt(sq((float)y_val)+sq((float)z_val))); //(Kalman)
          const float angle_err = angle_m - angle; //(Kalman)
          const float C_0 = 1; //(Kalman)
          const float PCt_0 = C_0 * PP[0][0];  //(Kalman)
          const float PCt_1 = C_0 * PP[1][0]; //(Kalman)
          const float E =R_angle+ C_0 * PCt_0; //(Kalman)
          const float K_0 = PCt_0 / E; //(Kalman)
          const float K_1 = PCt_1 / E; //(Kalman)
          const float t_0 = PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] (Kalman) */
          const float t_1 = C_0 * PP[0][1]; /* + C_1 * P[1][1]  = 0 (Kalman) */
          PP[0][0] -= K_0 * t_0; //(Kalman)
          PP[0][1] -= K_0 * t_1; //(Kalman)
          PP[1][0] -= K_1 * t_0; //(Kalman)
          PP[1][1] -= K_1 * t_1; //(Kalman)
          angle += K_0 * angle_err; //(Kalman)
          q_bias += K_1 * angle_err; //(Kalman)
          
          float myangle=(angle*57.2957795130823)+ calibration;
          float rate_deg = rate * 57.2957795130823;
          count++;
          
          Input= myangle;
          myPID.Compute();
          if(myangle<60 && myangle>-60)
              ocan1.speedwheel(-Output,0,0); 
          else
                ocan1.speedwheel(0,0,0); 
                  
          if(myangle<3 && myangle>-3 && count>500)
               myPID.SetTunings(Kp,Ki,Kd);
          else
               myPID.SetTunings(Kp,0,Kd) ;     
          Serial.print(" ,myangle=" );Serial.print(myangle);
          Serial.print(" ,Output=" );Serial.println(Output);

          oldAngle = myangle;           
          }
         // digitalWrite(17,temp);
          temp=!temp;        
       
  
  Serial2.print(Kp);
  Serial2.print(',');
  Serial2.print(Ki);
  Serial2.print(',');
  Serial2.println(Kd);

 // Serial2.print("Output:");Serial2.print(Output);
  //Serial2.print("INput:");Serial2.println(Input);
}

