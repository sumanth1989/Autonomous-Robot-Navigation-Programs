// Health Monitor for the Coylebot robot. Program checks the state of all sensors onboard and battery level along with camera and bluetooth status. This information is displayed on the  TFT screen.

#include "mbed.h"
#include "DRV8835.h"
#include "Adafruit_ST7735.h"
#include "QEI.h"
#include "LSM303D.h"
#include "L3GD20H.h"
#include "MSCFileSystem.h"
#include "MODSERIAL.h"
#include "GPS.h"
#include "Ticker.h"
#include "Adafruit_GFX.h"
#include "UCAMII.h"

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
Camera Cam(p9,p10,p28,p27,230400); 

//Creating namespace for flashdrive
#define FSNAME "usb"
MSCFileSystem usb(FSNAME);


int bluet; //Read bluetooth connection

int gyro1; //Read gyro connection
int compass1; //Ready compass connection
int gps1; //Read GPS connection
//Calculating robot heading
float magX, magY,magnetometervalue,robotorientation,robotheading;
float lat,lon;
//Calculating Wheel Speeds
int cidx, pidx;
double pi = 3.1415926535897;
double rad = pi/180;
int encodercount = 0;
float Time_values[2];  
float REncoder_Pulses[2], LEncoder_Pulses[2];
float REncoder_Pulse_Change[2], LEncoder_Pulse_Change[2]; 
float RWS[2], LWS[2];
float timeread;
float NPR = 909.72;
float radius = .021; 
float volts;
int percent;
int camcheck;
// Ticker functions 
Ticker healthmonitor; 

void monitor()
{   
    //Update the voltage
    volts=batt.read()*9.9;
    
    //Check bluetooth connection
    bluet = Bluetooth.readable();
    
    //Left and right wheel speed calculation
    timeread = 0.1;
    cidx= encodercount%2;
    pidx=(encodercount-1)%2;
    Time_values[cidx] = timeread;
    
    REncoder_Pulses[cidx] = Right.getPulses();
    REncoder_Pulse_Change[cidx] = REncoder_Pulses[cidx] - REncoder_Pulses[pidx];
    RWS[cidx] = ((REncoder_Pulse_Change[cidx]/timeread)/NPR)*(2*pi*radius); 
    
    LEncoder_Pulses[cidx] = Left.getPulses();
    LEncoder_Pulse_Change[cidx] = LEncoder_Pulses[cidx] - LEncoder_Pulses[pidx];
    LWS[cidx] = ((LEncoder_Pulse_Change[cidx]/timeread)/NPR)*(2*pi*radius);
    encodercount++;
    
    //Robot Heading Calculation
    comp.initialize();
    comp.setOffset(-245.5,139.5,0);
    comp.setScale(1,1.23,0); 
    
    //GPS check
    gps1 = gps.lock;
    
    //Gyro Check
    gyro1 = gyro.whoami();
    compass1 = comp.whoami();
    
    // Camera Check
    
    
    camcheck = Cam.connect();
} 
int main() 
{ 
    //Reset the screen
    tft.fillScreen(0x0000); 
    tft.setCursor(0,0);
    
    //Call the ticker function to update the parameters
    healthmonitor.attach(&monitor,0.1 );     
    
    //priniting the results to the tft screen
    while( true)
    { 
        //draws the battery symbol
        tft.drawRoundRect(0,0,12,8,0,0xFFFF);
        tft.drawRoundRect(12,2,3,4,0,0xFFFF);
        
        //used for assinging battery color based on amount of voltage
        uint16_t colorbattery;
        
        //Determines the amount the battery is filled based on voltage
        if (volts > 9)
        {
            percent = 9;
            colorbattery = ST7735_GREEN;
        }
        else if (volts > 8.5 && volts < 9)
        {
            percent = 8;
            colorbattery = ST7735_GREEN;
        }
        else if (volts > 8 && volts < 8.5)
        {
            percent = 7;
            colorbattery = ST7735_GREEN;
        }
        else if (volts > 7.5 && volts < 8)
        {
            percent = 6;
            colorbattery = ST7735_GREEN;
        }
        else if (volts > 7 && volts < 7.5)
        {
            percent = 5;
            colorbattery = ST7735_GREEN;
        }
        else if (volts > 6.5 && volts < 7)
        {
            percent = 4;
            colorbattery = ST7735_YELLOW;
        }
        else if (volts > 6 && volts < 6.5)
        {
            percent = 3;
            colorbattery = ST7735_YELLOW;
        }
        else if (volts > 5.5 && volts < 6)
        {
            percent = 2;
            colorbattery = ST7735_YELLOW;
        }
        else
        {
            percent = 1;
            colorbattery = ST7735_RED;
        }
        
        //Fills the battery icon with assigned percentage and color based on the amount of voltage
        for(int batx = 1; batx <= percent; batx++)
        {
            for (int baty = 1; baty <= 6; baty++)
            {
                tft.drawPixel(batx,baty,colorbattery);
            }
        }
        
        //Prints the Voltage reading
        tft.printf( "       %2.2f\n\r",volts);
        
        int down = 17;
        int up = 21;
        int diagnoaldown = 15;
        int diagnoalup = 23;
        uint16_t colorbluetooth;
        
        //Checks the bluetooth connection; white icon is no connection, blue is connected
        if (bluet == NULL)
        {
            tft.printf("\n\r   BT not connected\n\r");
            colorbluetooth = ST7735_WHITE;
        }
        else
        {   
            tft.printf(" \n\r  BT connected\n\r");
            colorbluetooth = ST7735_BLUE; 
        }
        
        //draws bluetooth icon
        for(int vert = 15; vert <= 23; vert++)
        {
            tft.drawPixel(5, vert, colorbluetooth);
        }     
    
        for(int cross = 3; cross <= 7; cross++)
        {
            tft.drawPixel(cross, down, colorbluetooth);
            tft.drawPixel(cross, up, colorbluetooth);
            down++;
            up--;                      
        }
       
        for(int diagnoal = 5; diagnoal <= 7; diagnoal++)
        {
            tft.drawPixel(diagnoal, diagnoaldown, colorbluetooth);
            tft.drawPixel(diagnoal, diagnoalup, colorbluetooth);
            diagnoaldown++;
            diagnoalup--;  
        }      
        
        //Checks the GPS connection status
        if ( gps1 == NULL)
        { 
            tft.printf("GPS not connected\n\r");
        }
        else
        { 
            //Print GPS coordinates after there is a gps lock
            lat = gps.latitude; 
            lon = gps.longitude;
            tft.printf("GPS connected\n\r");
            tft.printf("Lat: %f\n\rLong: %f\n\r",lat,lon);
            tft.printf("%d sats\n\r",gps.sats);
                //If the number of satellites falls under 3 then reset the gps lock back to 0 to redisplay gps not connected
                if( gps.sats < 3)
                {
                    gps.lock = 0;
                }
        }
        
        //Check the gyro connection
        if (compass1 == NULL ) 
        { 
            tft.printf("Compass not connected\n\r"); 
        }
        else
        { 
            tft.printf("Compass connected\n\r"); 
        }
        
        //Robot heading calculation
        magX = comp.magnetometer(XAXIS); 
        magY = comp.magnetometer(YAXIS);
        magnetometervalue = magX/magY;
        robotorientation = atan(magnetometervalue);
                   
        if ( magY > 0)
        {
            robotheading = 270 - (robotorientation/rad); 
            tft.printf("R_Heading: %3.2f\n\r", robotheading);
        }
        else if ( magY < 0)
        { 
            robotheading = 90 - (robotorientation/rad);
           tft.printf("R_Heading: %3.2f\n\r", robotheading);
        }
        else if ( magY == 0 & magX <0)
        {
            robotheading = 180; 
           tft.printf("R_Heading: %3.2f\n\r", robotheading);
        }
        else if ( magY == 0 & magX >0)
        { 
            robotheading = 0; 
           tft.printf("R_Heading: %3.2f\n\r", robotheading);
        }
        
        //Check the gyro connection
        if ( gyro1 == NULL ) 
        { 
            tft.printf("Gyro not connected\n\r"); 
        }
        else
        { 
            tft.printf("Gyro connected\n\r"); 
        }
        
        //No camera connection/not applicable since we currently do not have a camera or source file
        if (camcheck ==NULL)
        {
        tft.printf("Camera not connected\n\r"); }
        else
        {
        tft.printf("Camera is connected\n\r"); }
        //Print wheel velocity results
        tft.printf("RWS: %2.3f m/s\n\rLWS: %2.3f m/s\n\r", RWS[cidx], LWS[cidx]); 
        
        //clear the screen for update values and wait 2 milliseconds for screen to be cleared
        tft.fillScreen(0x0000);
        tft.setCursor(0,0);
        wait(.002); 
  
    }
}