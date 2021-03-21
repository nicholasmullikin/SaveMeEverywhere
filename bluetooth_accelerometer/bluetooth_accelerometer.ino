//#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <time.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <CircularBuffer.h>

#include <Adafruit_GPS.h>

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);


void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

int inDanger = 0;

CircularBuffer<uint8_t,100> stateBuffer; 

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

  if (magnitude < 9.9) {
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

  
//  ble.print("AT+BLEUARTTX=");
//  ble.print("X: ");
//  ble.print(event.acceleration.x);
//  ble.print("  ");
//  ble.print("Y: ");
//  ble.print(event.acceleration.y);
//  ble.print("  ");
//  ble.print("Z: ");
//  ble.print(event.acceleration.z);
//  ble.print("  ");
//  ble.print("Magnitude: ");
//  ble.print(magnitude);
//  ble.print("  ");
//  ble.print("State: ");
//  ble.print(state);
//  ble.print("  ");
//  ble.print("Danger: ");
//  ble.print(inDanger);
//  ble.print("  ");
//  ble.print("Danger: ");
//  ble.print(countState1);
//  ble.print("  ");
//  ble.print("Danger: ");
//  ble.print(countState2);
//  ble.print("  ");
//  ble.println("m/s^2 \\n");
}






void setup()
{
  int err;

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial);

  // Start the serial port connected to the satellite modem
 

  // If we're powering the device by USB, tell the library to
  // relax timing constraints waiting for the supercap to recharge.

  // Begin satellite modem operation


  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }
    // bluetooth code
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

  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }


 
  accel.setRange(LSM303_RANGE_2G);
  accel.setMode(LSM303_MODE_LOW_POWER);
  accel.setRange(LSM303_RANGE_4G);
}

void loop()
{

  /* Get a new sensor event */
  acceler();
  delay(150);
}
