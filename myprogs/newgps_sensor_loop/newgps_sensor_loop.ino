#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_GPS.h>
#if ARDUINO >= 100
 #include <SoftwareSerial.h>
 SoftwareSerial mySerial (3,2);
#else
  // Older Arduino IDE requires NewSoftSerial, download from:
  // http://arduiniana.org/libraries/newsoftserial/
  #include <NewSoftSerial.h>
 // DO NOT install NewSoftSerial if using Arduino 1.0 or later!
 NewSoftSerial mySerial (3,2);
#endif
Adafruit_GPS GPS(&mySerial);

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false, GPSECHO=false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
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
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  GPS.begin (9600);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(false);

  delay(1000);
  
  /* Initialise the sensors */
  initSensors();
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  char outstr [240] ;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  readGPS () ;
   // if a sentence is received, we can check the checksum, parse it...
  //if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    //if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
    //  return;  // we can fail to parse a sentence in which case we should just wait for another
  //}
/*
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
*.
  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
  /*
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  */
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    /*
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
    */
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    /*
    Serial.print(F("Alt: "));
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature)); 
    Serial.print(F(" m; "));
    */
    ///* Display the temperature */
    /*
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.print(F(" C"));
    */
  }
  //Serial.print (int(orientation.pitch*10)) ;
  //GPS.latitude = 72. ;
  sprintf (outstr, "$ %02d:%02d:%02d.%03d %d %d %d %d %d %d %d %d BYE", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds, 
   int( GPS.latitude), int(GPS.longitude), int(GPS.altitude)*10, int(GPS.fixquality), GPS.satellites, int(orientation.heading*10), int(orientation.pitch*10), int(orientation.roll*10)) ;
  Serial.println (outstr) ;
  Serial.flush () ;
  //Serial.println(F(""));
  delay(10);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c ;
  
  c = GPS.read();
  
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
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


void readGPS () {
  char c ;
  clearGPS() ;
  while (!GPS.newNMEAreceived()) {
    c = GPS.read() ;
  }
  GPS.parse (GPS.lastNMEA()) ;
  while (!GPS.newNMEAreceived()) {
    c = GPS.read() ;
  }
  GPS.parse (GPS.lastNMEA()) ;
  
  
    
}
void clearGPS() {
  char c ;
  while (!GPS.newNMEAreceived()){
    c=GPS.read() ;
  }
  GPS.parse (GPS.lastNMEA()) ;
}


