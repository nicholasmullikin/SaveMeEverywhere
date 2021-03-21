// Satcomm
// Nicholas Mullikin & Yale Zhao





////Bluetooth
#include <time.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//// Accel
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <CircularBuffer.h>

//// GPS
#include <Adafruit_GPS.h>

//// Satcomm
#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include "wiring_private.h" // pinPeripheral() function

//// Bluetooth
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//// Accel
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

//// GPS
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


//// SatComm
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
int signalQuality = -1;
int err;
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
#define IridiumSerial Serial2
#define DIAGNOSTICS true // Change this to see diagnostics


int inDanger = 0;

CircularBuffer<uint8_t,100> stateBuffer; 

int lat;
int lon;

IridiumSBD modem(IridiumSerial);

void acceler() {
   sensors_event_t event;
  accel.getEvent(&event);

  float magnitude = sqrt(event.acceleration.x * event.acceleration.x +
                           event.acceleration.y * event.acceleration.y +
                           event.acceleration.z * event.acceleration.z);
  uint8_t state = 0;
  // state is 0 if accel is normal
  // state is 1 if accel is too fast
  // state is 2 if accel if very slow

  if (magnitude < 10.9) {
    state = 2;
  } else if (magnitude > 30) {
    state = 1;
  }

  inDanger = 0;

  if (stateBuffer.isFull()){
    stateBuffer.shift();
  } 
  stateBuffer.push(state);


  int countState1 = 0;
  int countState2 = 0;
  int arraySize = stateBuffer.size();
  int found2 = 0;
  for (int i = 0; i < arraySize; i++){
    if (stateBuffer[i] == 1){
      countState1++;
      found2 = 1;
    }
    if (stateBuffer[i] == 2 && found2 == 1){
      countState2++;
    }
  }
  // state is 0 if accel is normal
  // state is 1 if accel is too fast
  // state is 2 if accel if very slow
  if (countState1 > 2) {
    inDanger = countState2 - countState1 > arraySize/2;
  }

  
  ble.print("AT+BLEUARTTX=");
//  ble.print("X: ");
//  ble.print(event.acceleration.x);
//  ble.print("  ");
//  ble.print("Y: ");
//  ble.print(event.acceleration.y);
//  ble.print("  ");
//  ble.print("Z: ");
//  ble.print(event.acceleration.z);
//  ble.print("  ");
  ble.print("Magnitude: ");
  ble.print(magnitude);
  ble.print("  ");
//  ble.print("State: ");
//  ble.print(state);
//  ble.print("  ");
  ble.print("Danger: ");
  ble.print(inDanger);
  ble.print("  ");
  ble.print("CS1: ");
  ble.print(countState1);
  ble.print("  ");
  ble.print("CS2: ");
  ble.print(countState2);
  ble.print("  ");
  ble.println("\\n");
}


void findGPS(){
  int hasFix = 0;

    while(hasFix == 0){
      char c = GPS.read();
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another
      }
   
      // ble.print("AT+BLEUARTTX=");
      // ble.print("Fix: "); ble.print((int)GPS.fix);
      // ble.print(" quality: "); ble.print((int)GPS.fixquality);
      // ble.println("\\n");
      if (GPS.fix) {
        hasFix = 1;
        lat = GPS.latitude_fixed;
        if (GPS.lat == 'S') lat *= -1;
        lon = GPS.longitude_fixed;
        if (GPS.lon == 'W') lon *= -1;
        ble.print("AT+BLEUARTTX=");
        ble.print(lat);
        ble.print(", ");
        ble.print(lon);
        ble.print("\\n");
      }
    }
}


void sendSatComm(){
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    ble.print("AT+BLEUARTTX=");
    ble.print(F("SignalQuality failed: error "));
    ble.println(err);
    return;
  }
  
  ble.print("AT+BLEUARTTX=");
  ble.print(F("On a scale of 0 to 5, signal quality is currently "));
  ble.print(signalQuality);
  ble.println(F("."));

  // Send the message
  ble.print("AT+BLEUARTTX=");
  ble.println(F("Trying to send the message.  This might take several minutes."));
  
  char text[25];
  sprintf(text, "%i, %i", lat, lon);
  err = modem.sendSBDText(text);
  if (err != ISBD_SUCCESS) {
    ble.print("AT+BLEUARTTX=");
    ble.print(F("sendSBDText failed: error "));
    ble.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
    ble.print("AT+BLEUARTTX=");
      ble.println(F("Try again with a better view of the sky."));
  }
  else {
    ble.print("AT+BLEUARTTX=");
    ble.println(F("Hey, it worked!"));
  }

  // Clear the Mobile Originated message buffer
  ble.print("AT+BLEUARTTX=");
  ble.println(F("Clearing the MO buffer."));
  err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
  if (err != ISBD_SUCCESS) {
    ble.print("AT+BLEUARTTX=");
    ble.print(F("clearBuffers failed: error "));
    ble.println(err);
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(F("Done!"));
}


void sendLatLong(){
   ble.print("AT+BLEUARTTX=");
    ble.print(lat);
    ble.print(", ");
    ble.print(lon);
    ble.print("\\n");
    delay(500);
  
  Serial2.print(lat);
  Serial2.print(" ");
  Serial2.println(lon);
}

void setup()
{
  //Serial2.begin(9600);
  ////SatComm
  
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
 

  // Start the console serial port
  Serial.begin(115200);
  //while (!Serial);

  //// Bluetooth code
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }
  ble.echo(false);
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  ble.verbose(false);
  
  while (! ble.isConnected()) {
      delay(500);
  }
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ){
    // Change Mode LED Activity
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  
  //// Accel
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
  accel.setRange(LSM303_RANGE_2G);
  accel.setMode(LSM303_MODE_LOW_POWER);
  accel.setRange(LSM303_RANGE_4G);


  //// GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(250);
  GPSSerial.println(PMTK_Q_RELEASE);


  ////SatComm
 

  IridiumSerial.begin(19200);

  ble.println(F("Starting modem..."));
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    ble.print(F("Begin failed: error "));
    ble.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      ble.println(F("No modem detected: check wiring."));
    return;
  }
}

void loop()
{
  /* Get a new sensor event */
  acceler();

  if (inDanger == 1){
    findGPS();

     sendSatComm();
    //sendLatLong();
  }
  delay(100);
}




#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.print(c);
  ble.print("AT+BLEUARTTX=");
  ble.println(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.print(c);
  ble.print("AT+BLEUARTTX=");
  ble.println(c);
}
#endif
