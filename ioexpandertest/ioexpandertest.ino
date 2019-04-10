#include <Wire.h>       

#define IO_ADDR 0x20
byte val = 1;
 
void setup()
{
  Wire.begin();   // Start de I2C
}


 
void loop()
{
  Wire.beginTransmission(32);    // Verzend naar adres #32 (0x20)
  Wire.write(15 - val);          // stuur de waarde naar de expander
  Wire.endTransmission();        // stop verzending
  
  val = val * 2;                 // verdubbel de waarde
  if(val > 8)                    // waarde groter dan 8 (binair 1000)
  {
    val = 1;                     // start weer bij 1 (binair 0001)
  }
  delay(1000);
}
