#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <time.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"


#include "wiring_private.h" // pinPeripheral() function


#define IridiumSerial Serial2
#define DIAGNOSTICS false // Change this to see diagnostics
 
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
IridiumSBD modem(IridiumSerial);

void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

int signalQuality = -1;
int err;

void setup()
{
  int err;

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial);

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);

  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  // If we're powering the device by USB, tell the library to
  // relax timing constraints waiting for the supercap to recharge.
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }

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
}

void loop()
{
   struct tm t;
   int err = modem.getSystemTime(t);
   if (err == ISBD_SUCCESS)
   {
      char buf[32];
      sprintf(buf, "%d-%02d-%02d %02d:%02d:%02d\n",
         t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
      Serial.print(F("Iridium time/date is "));
      Serial.println(buf);

      ble.print("AT+BLEUARTTX=");
      ble.println(buf);
   }

   else if (err == ISBD_NO_NETWORK) // Did it fail because the transceiver has not yet seen the network?
   {
      Serial.println(F("No network detected.  Waiting 10 seconds."));
      ble.print("AT+BLEUARTTX=");
      ble.println(F("No network detected.  Waiting 10 seconds."));
   }

   else
   {
      Serial.print(F("Unexpected error "));
      Serial.println(err);
      return;
   }

  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send?"));
  }


   // Delay 1 seconds
   delay(1 * 1000UL);
}

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif
