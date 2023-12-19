#include <Arduino.h>
#include <time.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
//#include <string.h>
#include <DFRobot_SIM808.h>

#define PIN_TX 3  
#define PIN_RX 4  

SoftwareSerial mySerial(PIN_TX, PIN_RX);

// ros::NodeHandle nh;
// sensor_msgs::NavSatFix gpsMsg;
// ros::Publisher gps("gps1", &gpsMsg);

// Publisher object for filtered IMU

double GPS_la = 0.1;
double GPS_lo = 0.1;
double GPS_ws = 0.1;
double GPS_alt = 0.1;
double GPS_heading = 0.1;
uint16_t GPS_year = 1;
uint8_t GPS_month = 1;
uint8_t GPS_day = 1;
uint8_t GPS_hour = 1;
uint8_t GPS_minute = 1;
uint8_t GPS_second = 1;
uint8_t GPS_centisecond = 1;
#define GPS_Sampling_Time_ms 100
unsigned long currentMillis_GPS = 0;
unsigned long previousMillis_GPS = 0;

bool state = LOW;
#define LED LED_BUILTIN

DFRobot_SIM808 sim808(&mySerial); //Connect RX,TX,PWR

void getGPS();

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(LED, OUTPUT); // Declare the LED as an output
  
  // initialize serial communication
  // mySerial.begin(9600);
  Serial.begin(9600);     //open serial and set the baudrate

  // ******** Initialize sim808 module *************
  while(!sim808.init())
  {
     //Serial.print("Sim808 init error\r\n");
     delay(100);
  }
  delay(300);

  if(sim808.attachGPS()){
     //Serial.println("Open the GPS power success");
  } else {
     //Serial.println("Open the GPS power failure");
  }

  //Connect to ROS
  // nh.initNode();

  //advertise GPS data
  // nh.advertise(gps);

  //while(!nh.connected()) {nh.spinOnce();}
}

void loop() {
  // //check the sampling time of acquising GPS data
  // currentMillis_GPS = millis();
  // if (currentMillis_GPS - previousMillis_GPS > GPS_Sampling_Time_ms) {
  //   getGPS();
    
  //   //************* Turn off the GPS power ************
  //   sim808.detachGPS();
  //   previousMillis_GPS = currentMillis_GPS;
  //   //publish GPS data
  //   gpsMsg.header.stamp = nh.now();
  //   gpsMsg.header.frame_id = "map";
  //   gpsMsg.latitude = GPS_la;
  //   gpsMsg.longitude = GPS_lo;
  //   gpsMsg.altitude = GPS_alt;
  //   gps.publish(&gpsMsg);
  // }
GPS_la = sim808.GPSdata.lat;
  GPS_lo = sim808.GPSdata.lon;
   Serial.print("Latitude: ");
  Serial.println(GPS_la, 6); 
  Serial.print("Longitude: ");
  Serial.println(GPS_lo, 6);
   // getGPS();
    
    // Turn off the GPS power
    sim808.detachGPS();
    previousMillis_GPS = currentMillis_GPS;
  // nh.spinOnce();
  // Serial.print("Latitude: ");
  // Serial.println(GPS_la, 6); 
  // Serial.print("Longitude: ");
  // Serial.println(GPS_lo, 6);
  // Serial.print("Altitude: ");
  // Serial.println(GPS_alt, 2); 
  // Serial.print("Speed: ");
  // Serial.println(GPS_ws, 2); 
  // Serial.print("Heading: ");
  // Serial.println(GPS_heading, 2); 
  // Serial.print("Year: ");
  // Serial.println(GPS_year);
  // Serial.print("Month: ");
  // Serial.println(GPS_month);
  // Serial.print("Day: ");
  // Serial.println(GPS_day);
  // Serial.print("Hour: ");
  // Serial.println(GPS_hour);
  // Serial.print("Minute: ");
  // Serial.println(GPS_minute);
  // Serial.print("Second: ");
  // Serial.println(GPS_second);
  // Serial.print("Centisecond: ");
  // Serial.println(GPS_centisecond);
  delay(200);

}

void getGPS(){ 
  // while(!sim808.attachGPS())
  // {
  //   //Serial.println("Open the GPS power failure");
  // }
  delay(80);

  //Serial.println("Open the GPS power success");
  digitalWrite(LED, HIGH);
    
  if(!sim808.getGPS())
  {
    //Serial.println("not getting anything");
    //stateChange();
  }

  GPS_la = sim808.GPSdata.lat;
  GPS_lo = sim808.GPSdata.lon;
  GPS_ws = sim808.GPSdata.speed_kph;
  GPS_alt = sim808.GPSdata.altitude;
  GPS_heading = sim808.GPSdata.heading;
  GPS_year = sim808.GPSdata.year;
  GPS_month = sim808.GPSdata.month;
  GPS_day = sim808.GPSdata.day;
  GPS_hour = sim808.GPSdata.hour;
  GPS_minute = sim808.GPSdata.minute;
  GPS_second = sim808.GPSdata.second;
  GPS_centisecond = sim808.GPSdata.centisecond;
}
