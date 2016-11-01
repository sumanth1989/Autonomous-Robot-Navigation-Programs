//This script uses the IR sensor onboard a small differentially sterred robot. It sweeps the sensor across a range of angles to determine obstacles in front of it and steers around these obstacles.
#include "mbed.h"
#include "DRV8835.h"
#include "Adafruit_ST7735.h"
#include "QEI.h"
#include "LSM303D.h"
#include "L3GD20H.h"
#include "MSCFileSystem.h"
#include "MODSERIAL.h"
#include "GPS.h"
#include "SweptIR.h"
#include "math.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
DRV8835 motors(p23,p18,p24,p11);
LSM303D comp(p5,p6,p7,p15);
L3GD20H gyro(p5,p6,p7,p22);
Adafruit_ST7735 tft(p5,p6,p7,p12,p8);
QEI Left(p30,p29,NC,909.72,QEI::X4_ENCODING);  // encoder object for Left wheel
QEI Right(p17,p16,NC,909.72,QEI::X4_ENCODING);  // encoder object for Right wheel
MODSERIAL Bluetooth(p28, p27, 256); // tx, rx read from bluetooth module
GPS gps(p13,p14);
AnalogIn batt(p19);
SweptIR sweep(p25,p20);


Ticker obstacleavoidance; // Ticker function to call the obstacle avoidance function.
float distance_error; // Used to calculate distance error to determine turn radius.
float  distanceL, distanceC, distanceR; // Store distance of obeject from Robot
float V= 0.5; // Constant velocity 
float vR,vL,vC;
double pi = 3.1415926535897;
double rad = pi/180;
float Kp = 3; // Proportional constant for control loop
void obstavd()
{ 
    tft.setCursor(0,0);
    //Taking radings from IR sensor
    sweep.SetRange(40,3);
    sweep.start();
    vL = sweep.values[2] ;
    vC = sweep.values[1] ;
    vR = sweep.values[0] ; 
    
    //Calculating distance of obstacle from IR sensor values
    //Left Distance Calculation
    vL = (vL-0.56)/0.24;
    distanceL = (1.8*pow(vL,6))-(5.7*pow(vL,5))+(2.7*pow(vL,4))+(2.2*pow(vL,3))+(4.7*pow(vL,2))-(13*vL)+21;
    
    //Center Distance Calculation
    vC = (vC-0.56)/0.24;
    distanceC = (1.8*pow(vC,6))-(5.7*pow(vC,5))+(2.7*pow(vC,4))+(2.2*pow(vC,3))+(4.7*pow(vC,2))-(13*vC)+21;
    
    //Right Distance Calculation
    vR =(vR-0.56)/0.24;
    distanceR = (1.8*pow(vR,6))-(5.7*pow(vR,5))+(2.7*pow(vR,4))+(2.2*pow(vR,3))+(4.7*pow(vR,2))-(13*vR)+21;
  
    //Print distances of obstacle from robot in cm
    tft.printf ( "Dist. Left: %3.3f\n\r",distanceL);
    tft.printf ( "Dist. Cent.: %3.3f\n\r",distanceC);
    tft.printf ( "Dist. Right: %3.3f",distanceR);
          
    //Algorithm to determine obstacle avoidance, if an object comes within 75 cm then begin applying control procedures      
    if (distanceC < 75 || distanceR < 75 || distanceL < 75)
    {   
        //If object is closer on left then on right then turn right        
        if (distanceL < distanceR )
        { 
             distance_error = 1/distanceL;
             motors.motorR_fwd(V -(Kp*distance_error));
             motors.motorL_fwd(V +(Kp*distance_error)); 
        }
        //If object is closer on right then on left then turn left
        else if (distanceR < distanceL )
        {
             distance_error = 1/distanceR*(-1);  
             motors.motorR_fwd(V -(Kp*distance_error));
             motors.motorL_fwd(V +(Kp*distance_error)); 
        }      
    }
    else
    {
        motors.motorR_fwd(V);
        motors.motorL_fwd(V);
    } 
}
 
int main() 
{
    //Ticker to call the Obstacle avoidance function
    obstacleavoidance.attach(&obstavd,0.5);     
}
