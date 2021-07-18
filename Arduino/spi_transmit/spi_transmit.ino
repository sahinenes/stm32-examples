#include <SPI.h>
#include "pins_arduino.h"
int a;
void setup (void)
{
  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  a=0;
  
}  // end of setup

void loop (void)
{

   byte c;

  // enable Slave Select
  digitalWrite(SS, LOW);    // SS is pin 10

  // send test string
  for (const char * p = "deneme"+a ; c = *p; p++)
    SPI.transfer (c);

  // disable Slave Select
  digitalWrite(SS, HIGH);
a++;
  delay (1000);  // 1 seconds delay 
}  // end of loop
