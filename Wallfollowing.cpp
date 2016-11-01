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

float distance_error; // Used to calculate distance error to determine turn radius.
float  distance90, distance0; // Store distance of obeject from Robot
float V= 0.3; // Constant velocity in control loop
float vR,vC; // Read sensor values
double pi = 3.1415926535897;
double rad = pi/180;
float K = 1; // Proportionality constant for turning while wall following and when wall turns infront of robot
float Kw = 4; // Proportionality constant for turning when wall turns away from bot
int main() 
{ 
while(1)
{
 tft.setCursor(0,0);
 
// Taking readings from IR sensor 
 sweep.position(0);
 wait(0.2);
 vC = sweep.read() ;
 sweep.position(-90);
 wait(0.2);
 vR = sweep.read() ; 
 sweep.position(0);

// Calculating distance of wall  from IR sensor values
  vC = (vC-0.56)/0.24;
  distance0 = (1.8*pow(vC,6))-(5.7*pow(vC,5))+(2.7*pow(vC,4))+(2.2*pow(vC,3))+(4.7*pow(vC,2))-(13*vC)+21; // Forward distance from wall

  vR =(vR-0.56)/0.24;
  distance90 = (1.8*pow(vR,6))-(5.7*pow(vR,5))+(2.7*pow(vR,4))+(2.2*pow(vR,3))+(4.7*pow(vR,2))-(13*vR)+21; //Perpendicular distance from wall
  
  
  
// Print distances of wall from robot
 tft.printf ( "Perpendicular Dist: %3.3f\n\r",distance90);
 tft.printf ( "Forward Dist: %3.3f",distance0);

  
    
 // When no front wall is detected       
 if ( distance0 > 45 ) 
        {   
                 if (distance90 > 25 )
                        { 
                             distance_error = distance90*0.00075;
                             motors.motorR_fwd(V -(K*distance_error));
                             motors.motorL_fwd(V +(K*distance_error)); 
                        }
                else if (distance90 < 20)
                        {
                             distance_error = 1/distance90*(-1);  
                             motors.motorR_fwd(V -(K*distance_error));
                             motors.motorL_fwd(V +(K*distance_error)); 
                        }  
                
    
            
                else 
                        {
                           distance_error = 0; 
                           motors.motorR_fwd(V -(K*distance_error));
                           motors.motorL_fwd(V +(K*distance_error)); 
    
                        }   
        }
 // When front wall is detected
 else if (distance0 < 45)
        {                   
                        
                             distance_error = 1/distance0*(-1);
                             motors.motorR_fwd(V -(Kw*distance_error));
                             motors.motorL_fwd(V +(Kw*distance_error));
        }
        
}          
}
    
 

