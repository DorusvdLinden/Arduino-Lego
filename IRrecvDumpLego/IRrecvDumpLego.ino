




/*
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Addapted from IRrecvDump 2009 Ken Shirriff
 */

#include <IRremote.h>

int RECV_PIN = 11;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}

void dump(decode_results *results) {
  int count = results->rawlen;
  char tmp[17];

  if (results->decode_type == UNKNOWN) {
    Serial.println("Could not decode message");
  } 
  else {
    if (results->decode_type == NEC) {
      Serial.print("Decoded NEC: ");
      Serial.print(results->value, HEX);
    } 
    else if (results->decode_type == SONY) {
      Serial.print("Decoded SONY: ");
      Serial.print(results->value, HEX);
    } 
    else if (results->decode_type == RC5) {
      Serial.print("Decoded RC5: ");
      Serial.print(results->value, HEX);
    } 
    else if (results->decode_type == RC6) {
      Serial.print("Decoded RC6: ");
      Serial.print(results->value, HEX);
    }
    else if (results->decode_type == LEGO) {
      sprintf(tmp, "%.4X",results->value);
      Serial.print("Decoded Lego: ");
      Serial.print(tmp);
      Serial.print(", dec: ");
      Serial.print(results->value);
    }
    Serial.println(".");
//    Serial.print(results->bits, DEC);
//    Serial.println(" bits)");
  }
//  Serial.print("Raw (");
//  Serial.print(count, DEC);
//  Serial.print("): ");

//  for (int i = 0; i < count; i++) {
//    if ((i % 2) == 1) {
//      Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
//    } 
//    else {
//      Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
//    }
//    Serial.print(" ");
//  }
//  Serial.println("");
}

void loop() {
  if (irrecv.decode(&results)) {
            
    dump(&results);
    irrecv.resume(); // Receive the next value
  }
}
