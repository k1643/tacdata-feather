/*********************************************************************
  This Arduino Feather BLE program reads 16 inputs, and 
  translates those inputs into Bluetooth Low-Energy (BLE) Keyboard commands.
  
*********************************************************************/


#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined(ARDUINO_ARCH_SAMD)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/*
 Set up the HW and the BLE module. the setup function runs once when you press reset or power the board.
*/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("tacdata-feather HID Keyboard"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  
  Serial.println("Requesting Bluefruit info:");
  ble.info(); /* Print Bluefruit information */

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit Keyboard': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Bluefruit Keyboard" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
//  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
//  } else {
//    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
//      error(F("Could not enable Keyboard"));
//    }
//  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  enableInput(); // set input pins for digitalRead

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts keyboard input"));

  Serial.println();
  Serial.println(F("Enter the character(s) to send:"));
  Serial.println(F("- \\r for Enter"));
  Serial.println(F("- \\n for newline"));
  Serial.println(F("- \\t for tab"));
  Serial.println(F("- \\b for backspace"));

  Serial.println();
}

/* 
 * enable pull-up resistor for switch and invert polarity of signal on selected IO pins.
 * 
 * https://www.arduino.cc/en/Reference/HomePage
 */
void enableInput() {  
  // Using analog pins for digital input
  // https://www.arduino.cc/en/Tutorial/AnalogInputPins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  digitalWrite(A0, HIGH);  // set pullup on analog pin
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);
  
  // set digital pins to INPUT_PULLUP
  // https://www.arduino.cc/en/Reference/PinMode
  for (int pin = 0; pin <= 12; pin++) {
    // pin 4,7,8 control BLE
    if (pin == 4 || pin == 7 || pin == 8) {
      continue;
    }
    // do pins 2 (SDA) and3 (SCL) have pullup resistors?
    pinMode(pin, INPUT_PULLUP);
  }  
}

/*
 * show current state of input pins
 */
void showState() {
  char state[] = "state:_00000000_00000000";

  Serial.print("analog pins ");
  for (int i = 0; i < 6; i++) {
    Serial.print(A0+i, DEC);
    Serial.print(" ");
    state[7+i] = digitalRead(A0+i) == HIGH ? '0' : '1'; // using INPUT_PULLUP reverses LOW,HIGH
  }
  Serial.println();
  state[7+6] = digitalRead(0) == HIGH ? '0' : '1';
  state[7+7] = digitalRead(1) == HIGH ? '0' : '1';
  int stringi = 7+9;
  for (int pin = 2; pin <= 12; pin++) {
    // pin 4,7,8 control BLE
    if (pin == 4 || pin == 7 || pin == 8) {
      continue;
    }
    state[stringi++] = digitalRead(pin) == HIGH ? '0' : '1';
  }
  
  Serial.println(state);
}

int currKey = 0;
int keymap[] = {'a', 'b', 'c', 'd', -1, 'e', 'f', -1, -1, 'h', 'i', 'j', 'k'};
  
int getKey() {

  // pins 0-5 are low pin 6 is HIGH
  //
  for (int pin = 0; pin <= 12; pin++) {
    // pin 4,7,8 control BLE
    if (pin == 4 || pin == 7 || pin == 8) {
      continue;
    }
    if (!digitalRead(pin)) {
      return keymap[pin];
    }
  }
  return -1;
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  delay(500);
  showState();
  return;
  
  // Display prompt
  //Serial.print(F("keyboard > "));

  // Check for user input and echo it back if anything was found
  char keys[BUFSIZE+1];
  memset(keys, 0, BUFSIZE);
  //char keys[] = {'\0', '\n'};
  //getUserInput(keys, BUFSIZE);

  int c = getKey();
  
  if (c > -1) {
    keys[0] = (char) c;
    Serial.print("\nSending ");
    Serial.println(keys);
  
    ble.print("AT+BleKeyboard=");
    ble.println(keys);
  
    if (ble.waitForOK()) {
      Serial.println(F("OK!"));
    } else {
      Serial.println(F("FAILED!"));
    }
  }
  delay(500);
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );
}
