/*--------------------------------------------------------------------------------------------------
  Name         :  SubaruDisplayEmulator_v_1_3.c
  Description  :  This program enables the displya on Subaru Tribeca.
  MCU          :  ATmega328P @ 16 MHz.
  Author       :  2022-03-29 - Panin Aleksandr
  Copyright    :  (c) 2022 SigmaObjects
                  2022-06-16 - v1.3 Production release.
--------------------------------------------------------------------------------------------------*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>
#include "Settings.h"

#if (USE_SOFTSERIAL)
  #include <SoftwareSerial.h>
  SoftwareSerial altSerial(PIN_SS_RX, PIN_SS_TX); // RX, TX
#endif

#include "IEBUS.h"


void setup() {
  
  pinMode(PIN_STB, OUTPUT);
  digitalWrite(PIN_STB, HIGH);
  
  Serial.begin(SERIAL_SPEED);

#if (USE_SOFTSERIAL)
  altSerial.begin(SS_SPEED);
#endif
  
  MCUSR = 0;
  wdt_disable();
  
  // Init LED port pin
  LED_DDR |= LEDOUT;
  LedOff();
  
  TCCR0B = (1<<CS01) | (1<<CS00);
  DDRD |= _BV(PIN_OUT);
  OUT_CLEAR;
    
  //  Enable watchdog @ ~2 sec.
  wdt_enable( WDTO_2S );
}


void loop() {
  
  // Reset watchdog.
  wdt_reset();

  // Read message from lan
  AvcReadMessage();

  // Check register session timeout & change register status to false
  if((lastRegistred + TIMEOUT_RECONNECT) < millis()){
    isRegistred = false;
  }

  // Init registration in timer
  if(!isRegistred){
    if((timerRegister + TIMEOUT_NETPING) < millis()){
      timerRegister = millis();
      AvcRegisterMe();
    }
  }

}
