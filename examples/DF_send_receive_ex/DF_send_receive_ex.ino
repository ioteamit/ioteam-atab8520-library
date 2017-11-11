/*
    SmarteEveryting Fox ATAB8520 Library - DF_send_receive_ex

    This example shows how to send and receive a message using the Atmel ATAB8520 module

    created 25 Feb 2017
    by IoTeam (sw@ioteam.it)

    This example is in the public domain
    https://github.com/ioteamit/ioteam-atab8520-library

    More information on ATAB8520 available here:
    http://www.atmel.com/tools/ata8520-ek2-e.aspx

*/


#include <Sfx.h>

long timems=0;

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB) {
    ;
  }

  SigFoxObj.begin();
  SerialUSB.println("Exit setup");
}

void loop() {
  unsigned int error;
  long time_now = millis();
  if ((time_now - timems) > 60000) {
    timems = time_now;
    Serial.println("Send message ...");

    uint8_t msg[]={0xaa, 0xbb, 0xcc, 0xdd};
    uint8_t received[8];

    error = SigFoxObj.sendReceiveMessage(msg, 4, received);
    
    Serial.println("Received");
    for(int i=0; i<8; i++) {
      Serial.print(received[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");

    Serial.print("Error: ");
    Serial.println(error);
  }

}
