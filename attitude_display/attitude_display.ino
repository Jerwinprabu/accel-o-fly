/*
  LiquidCrystal Library - Custom Characters
 
 Demonstrates how to add custom characters on an LCD  display.  
 The LiquidCrystal library works with all LCD displays that are 
 compatible with the  Hitachi HD44780 driver. There are many of 
 them out there, and you can usually tell them by the 16-pin interface.
 
 This sketch prints "I <heart> Arduino!" and a little dancing man
 to the LCD.
 
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K potentiometer:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 * 10K poterntiometer on pin A0
 
 created21 Mar 2011
 by Tom Igoe
 Based on Adafruit's example at
 https://github.com/adafruit/SPI_VFD/blob/master/examples/createChar/createChar.pde
 
 This example code is in the public domain.
 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 
 Also useful:
 http://icontexto.com/charactercreator/
 
 */

// include the library code:
#include <LiquidCrystal.h>

int delayTime = 100;

// initialize the library with the numbers of the interface pins
// reversed with new breadboard 
// LiquidCrystal lcd(12, 11, 10, 9, 8, 7);
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// make some custom characters:
byte fullBlock[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte emptyBlock[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte rblock_45_3[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00001,
  0b00011,
  0b00111
};

byte rblock_45_4[8] = {
  0b00001,
  0b00011,
  0b00111,
  0b01111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte rblock_45_6[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte rblock_45_7[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte rblock_45_8[8] = {
  0b01111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};



void setup() {
  // create a new character

// setup chars for right 45 degree turn
// first row
  lcd.createChar(1, emptyBlock);
  lcd.createChar(2, emptyBlock);
  lcd.createChar(3, rblock_45_3);
  lcd.createChar(4, rblock_45_4);
  lcd.createChar(5, fullBlock);
  // second row
  lcd.createChar(6, emptyBlock);
  lcd.createChar(7, emptyBlock);
  lcd.createChar(8, emptyBlock);
//  lcd.createChar(9, fullBlock);
//  lcd.createChar(10,fullBlock);
  
  // set up the lcd's number of columns and rows: 
  lcd.begin(16, 2);
  // set the cursor to top left
  lcd.setCursor(0, 0);
  lcd.write(1);
  lcd.write(2);
  lcd.write(3);
  lcd.write(4);
  lcd.write(5);  
// goto 2nd row
  lcd.setCursor(0, 1);
  lcd.write(6);
  lcd.write(7);
  lcd.write(8);
  lcd.write(9);
  lcd.write(10);  

}

void loop() {}



