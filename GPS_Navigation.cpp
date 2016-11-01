// This script uses the GPS sensor onboard a small differentially sterred robot. It takes GPS coordinate inputs from a text file and drives to these locations using the GPS sensor while calculating its wheel speed, battery voltage, current heading, orientation using the onboard IMU and displaying it on the robot's tft screen 
#include "mbed.h"
#include "DRV8835.h"
#include "Adafruit_ST7735.h"
#include "QEI.h"
#include "LSM303D.h"
#include "L3GD20H.h"
#include "MSCFileSystem.h"
#include "MODSERIAL.h"
#include "math.h"
#include "GPS.h"

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
GPS gps(p13,p14);

AnalogIn batt(p19);

// Timer for speed calculation
Timer t; 

//The command latitude and longitude values from the text file and conversion to radians
float cmd_latitude, cmd_longitude; 
double cmdr_latitude, cmdr_longitude; 

//Initial GPS latitude and longitude coordinates of the robot and conversion to radians
double init_latitude, init_longitude; 
double initr_latitude, initr_longitude; 

//Distance calculation values between two gps locations
double theta, dist;

//Variable initalizations for wheel calculations
int cidx, pidx;
int encodercount = 0;
float Time_values[2];  
float REncoder_Pulses[2], LEncoder_Pulses[2];
float REncoder_Pulse_Change[2], LEncoder_Pulse_Change[2]; 
float RWS[2], LWS[2];
float timeread;

// Magnetometer reading in X and Y Direction
double magX, magY; 

//Used to calculate robot heading
double magnetometervalue, robotorientation, X, Y; 

//Heading values
double gpsheading, robotheading, robotturnangle, headingerror, headingdifference;
double delta_lat; // DIfference in latitude, used to calculate heading

//initiates constant values
float K = .002;
double pi = 3.1415926535897;
double rad = pi/180;
int run = 1;
float Vconstant = .5;
int earthradius = 6371000;
//Number of pulses per revolution
float NPR = 909.72;
//Wheel radius of the robot 
float radius = .021; 

// Create the local filesystem under the name "local"
LocalFileSystem local("local");               

//function for calculating the heading error between GPS bearing heading and robot heading
double heading(double robot, double gps)
{   
    robotturnangle = robot - gps;
    if (robotturnangle < -180)
    {
        headingdifference = robotturnangle + 360;
    }
    else if (robotturnangle > 180)
    {
        headingdifference = robotturnangle - 360;
    }
    else
    {
        headingdifference = robotturnangle;
    }
    return headingdifference;      
}

//function for calculating the wheel speeds of the left and right wheels
void wheelspeeds(int currentindex, int previousindex, float timevalue)
{
    //Creates a file for recording the robots wheel speeds in m/s
    FILE * wheelspeed;
    wheelspeed = fopen("/local/wheelspeeds.txt", "a");
    
    Time_values[currentindex] = timevalue;
    
    //Calculate the robot wheel speeds from encoder pulses
    REncoder_Pulses[currentindex] = Right.getPulses();
    REncoder_Pulse_Change[currentindex] = REncoder_Pulses[currentindex] - REncoder_Pulses[previousindex];
    RWS[currentindex] = ((REncoder_Pulse_Change[currentindex]/(Time_values[currentindex]-Time_values[previousindex]))/NPR)*(2*pi*radius); 
    
    LEncoder_Pulses[currentindex] = Left.getPulses();
    LEncoder_Pulse_Change[currentindex] = LEncoder_Pulses[currentindex] - LEncoder_Pulses[previousindex];
    LWS[currentindex] = ((LEncoder_Pulse_Change[currentindex]/(Time_values[currentindex]-Time_values[previousindex]))/NPR)*(2*pi*radius);
    
    tft.printf("RWS: %2.3f m/s\n\rLWS: %2.3f m/s\n\r", RWS[currentindex], LWS[currentindex]);
    
    fprintf(wheelspeed," %f, %f, %f,\n\r",RWS[currentindex],LWS[currentindex],Time_values[currentindex]);
    fclose(wheelspeed); 
}

int main() 
{   
    tft.fillScreen(0x0000);
    
    // Open "assn.txt" on the local file system for reading the values in the text
    FILE *getvalues = fopen("/local/assn.txt", "r");  
    
    //Error message if file is not opened
    if ( getvalues == NULL )
    {
        tft.printf("Error: Could not open file for read operation.\r\n");     
        return -1;
    }
    
    //Creates a file for recording the robot's gps latitude and longitude
    FILE * robotgps;
    robotgps = fopen("/local/robotgps.txt", "a"); 
    fprintf(robotgps,"latitude, longitude,\n\r");
    
    //Run program for heading towards all GPS locations set by assn.txt
    while(true)
    {
        //run is initalized for gps way point following
        run = 1;
        
        //Scanning the commanded GPS Coordinates from text file until the end of file; if the scan retrieves less than 2
        //coordinates then the end of the file has been reached
        if(fscanf(getvalues, "%f %f ", &cmd_latitude, &cmd_longitude)<2)
        {
            tft.printf("End Run: No more gps locations in assn.txt.\r\n");
            tft.printf("Distance: %f", dist);
            motors.motorR_fwd(0);
            motors.motorL_fwd(0);
            fclose(getvalues);
            fclose(robotgps);
            return -2;
        }
        
        t.start();
        
        //After reading a commanded gps location from text file will perform maneuvers to reach gps location
        while(run == 1)
        {   
            //Allows for an initial 10 second calibration of robot gps location
            while(t.read() < 10)
            {
                init_latitude = gps.latitude;
                init_longitude = gps.longitude;
            }
            
            //Get robot gps locations
            init_latitude = gps.latitude;
            init_longitude = gps.longitude;
            
            //Prints initial robot GPS location and sends it to robotgps text file    
            tft.printf("I_Latitude: %f\n\rI_Longitude: %f\n\r",init_latitude,init_longitude);
            fprintf (robotgps," %f, %f,\n\r",init_latitude, init_longitude); 
             
            //convert current robot gps coordinates to radians
            initr_latitude = init_latitude*rad;
            initr_longitude = init_longitude*rad;
            
            //Convert text file gps coordinates to radians
            cmdr_latitude = cmd_latitude *rad;
            cmdr_longitude = cmd_longitude *rad; 
               
            //Calculation of the distance between the current robot gps and the commanded gps
            theta = cmdr_longitude - initr_longitude;
            dist = acos(sin(initr_latitude) * sin(cmdr_latitude) + cos(initr_latitude) * cos(cmdr_latitude) * cos(theta)) * earthradius;
            tft.printf("Distance:%f\n\r",dist);
            
            //Calculation of bearing heading between current robot gps and commanded gps; positive degrees is in clockwise rotation
            X = cos (initr_latitude)* sin(cmdr_latitude) - sin (initr_latitude)*cos (cmdr_latitude)* cos(theta);
            Y = cos (cmdr_latitude)*sin(theta);
            gpsheading = atan2(Y, X)/rad;
            tft.printf("GPS Heading %f\n\r",gpsheading);
            
            //Initialize Compass and calibration
            comp.initialize();
            comp.setOffset(-245.5,139.5,0);
            comp.setScale(1,1.23,0);
                  
            //Calculating the current robot heading using the compass with respect to true north; positive degrees is in clockwise rotation
            tft.printf("MagXYZ %d, %d, %d\n\r",comp.magnetometer(XAXIS),comp.magnetometer(YAXIS),comp.magnetometer(ZAXIS));
            magX = comp.magnetometer(XAXIS);
            magY = comp.magnetometer(YAXIS);
            magnetometervalue = magX/magY;
            robotorientation = atan(magnetometervalue);
                   
            if ( magY > 0)
            {
                robotheading = 270 - (robotorientation/rad); 
                tft.printf("Robot Heading: %f\n\r", robotheading);
            }
            else if ( magY < 0)
            { 
                robotheading = 90 - (robotorientation/rad);
                tft.printf("Robot Heading: %f\n\r", robotheading);
            }
            else if ( magY == 0 & magX <0)
            {
                robotheading = 180; 
                tft.printf("Robot Heading: %f\n\r", robotheading);
            }
            else if ( magY == 0 & magX >0)
            { 
                robotheading = 0; 
                tft.printf("Robot Heading: %f\n\r", robotheading);
            }
            
            //function call for the heading error to see how much the robot needs to turn to face the right direction
            //for the set gps waypoint
            headingerror = heading(robotheading, gpsheading);
            tft.printf("Heading Error: %f\n\r", headingerror);
            
            //If the heading error is off by more than 2 degrees in either direction adjust the left or right wheel speed to correct
            if (headingerror < 2 || headingerror > -2)
            {
                //K is a proportional gain of .002 for adjusting the left and right motor speeds depending on how large the
                //heading error is off between 180 and 0 or -180 and 0; Vconstant is a constant setting for the base wheel speeds
                //if there is no heading error
                motors.motorR_fwd(Vconstant + K*headingerror);
                motors.motorL_fwd(Vconstant - K*headingerror);
                
                //values for calulating wheel speeds
                ++encodercount;
                cidx=encodercount%2;
                pidx=(encodercount-1)%2;
                timeread = t.read();
                
                //function call for calculating wheelspeed
                wheelspeeds(cidx, pidx, timeread);   
            }
            
            //If heading error is less than 2 degrees than drive the robot straight
            else
            {
                motors.motorR_fwd(Vconstant);
                motors.motorL_fwd(Vconstant);
                
                ++encodercount;
                cidx=encodercount%2;
                pidx=(encodercount-1)%2;
                timeread =  t.read();
                
                wheelspeeds(cidx, pidx, timeread);     
            }
            
            //provides the break statement for exiting the current gps way point and going back to the text file to get the next way point
            //to signify that a waypoint has been reached motors will shut off for 3 seconds; due to robot gps location not being completely exact
            //if the robot is within 2 meters of the commanded gps way point then it has accomplished reaching the designated way point
            if(dist < 2)
            {
                run = 0;
                motors.motorR_fwd(0);
                motors.motorL_fwd(0);
                wait(3);     
            }  
            tft.fillScreen(0x0000); 
            tft.setCursor(0,0);
        }                       
    }      
}
