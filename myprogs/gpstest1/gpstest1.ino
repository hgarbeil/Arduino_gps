#include <Adafruit_GPS.h>
#if ARDUINO >= 100
 #include <SoftwareSerial.h>
 SoftwareSerial mySerial (3,2);
#else
  // Older Arduino IDE requires NewSoftSerial, download from:
  // http://arduiniana.org/libraries/newsoftserial/
// #include <NewSoftSerial.h>
 // DO NOT install NewSoftSerial if using Arduino 1.0 or later!
 NewSoftSerial mySerial (3,2);
#endif
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);




Adafruit_GPS GPS(&mySerial) ;
/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
int stringloc = 0;

#define GPSECHO false 
boolean usingInterrupt = true ;
void useInterrupt(boolean) ;


void setup() {
  

  Serial.begin (115200) ;
  GPS.begin (9600) ;
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF) ;
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  
  initSensors() ;
  
  useInterrupt(true) ;
  
  delay (1000) ;
  

}



void loop() {
  
  
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;  
  char gpstime[240], sensorstr[480] ;

  Serial.println("Hello") ;
  //char c = GPS.read() ;
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // put your main code here, to run repeatedly:
  
  //Serial.print(c) ;
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    //Serial.print(F("Heading: "));
    //Serial.print(orientation.heading);
    //Serial.print(F("; "));
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  /*
  if (bmp_event.pressure)
  {
    
    float temperature;
    bmp.getTemperature(&temperature);
    /// Convert atmospheric pressure, SLP and temp to altitude    
    Serial.print(F("Alt: "));
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature)); 
    Serial.print(F(" m; "));
    /// Display the temperature 
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.print(F(" C")); 
  }
  Serial.println(F("")) ;
  */
  sprintf (gpstime, "$ %02d:%02d:%02d  %d %d %f %f ", GPS.hour, GPS.minute, GPS.seconds, GPS.fixquality, GPS.satellites, GPS.latitude, GPS.longitude) ;
  sprintf (sensorstr, "%s %f %f &f", gpstime, orientation.heading, orientation.pitch, orientation.roll) ;
  //Serial.println (sensorstr) ;
}



// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;  

  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
  
  
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }

  
}
void initSensors()
{
  Serial.println ("in init sensors") ;
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
  return ;
}

