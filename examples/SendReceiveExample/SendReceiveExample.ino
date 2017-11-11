/*
    SmarteEveryting Fox ATAB8520 Library - SendReceiveExample

    This example shows how to send and receive a message using the Atmel ATAB8520 module

    created 25 Feb 2017
    by IoTeam (sw@ioteam.it)

    This example is in the public domain
    https://github.com/ioteamit/ioteam-atab8520-library

    More information on ATAB8520 available here:
    http://www.atmel.com/tools/ata8520-ek2-e.aspx

 */

#include <Sfx.h>

#define SFX_SS_PIN 45
#define SFX_RST_PIN 37
#define SFX_PWRON_PIN 39
#define SFX_EVENT_PIN 42

Sfx *sfx;
uint8_t txdata[]={0x43, 0x49, 0x41, 0x4F, 0x43};
uint8_t rxdata[9];

void setup() {
  sfx = new Sfx(SFX_SS_PIN, SFX_RST_PIN, SFX_PWRON_PIN, SFX_EVENT_PIN);

  uint8_t freq[]={0x33, 0xBE, 0x9C, 0xD0};
  
  sfx->startHomologation(freq);
  sfx->storeSystemConfig(0, 0, 3, true, false, true);
  
  sfx->systemReset();
  sfx->waitUntilEvent(30000);
  
  unsigned int error=sfx->sendReceiveMessage(txdata, 5, rxdata);
}

void loop() {
  sfx->systemReset();
  sfx->waitUntilEvent(30000);
  
  unsigned int error=sfx->sendReceiveMessage(txdata, 5, rxdata);
}
