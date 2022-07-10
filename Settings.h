/*--------------------------------------------------------------------------------------------------
  Name         :  Settings.h
  Description  :  IEBUS driver settings for Subaru devices
--------------------------------------------------------------------------------------------------*/
#ifndef _AVCLANDRV_H_
#define _AVCLANDRV_H_

/*--------------------------------------------------------------------------------------------------
                                       Default settings
--------------------------------------------------------------------------------------------------*/


#define FALSE                   0
#define TRUE                    (!FALSE)

// led port settings
#define LED_DDR                 DDRB
#define LED_PORT                PORTB
#define LEDOUT                  _BV(PORT5)

// serial settings
#define SERIAL_SPEED            115200

// softvare serial settings
#define USE_SOFTSERIAL          false     // Turn "true" for send data to software serial port
#define PIN_SS_RX               4
#define PIN_SS_TX               3
#define SS_SPEED                115200

// timout settings
#define TIMEOUT_RECONNECT       5000
#define TIMEOUT_NETPING         2000


/*--------------------------------------------------------------------------------------------------
                                       IE_BUS driver settings
--------------------------------------------------------------------------------------------------*/


#define PIN_STB                 8
#define PIN_ACC                 9


//////// for use outer comparator HA12187

#define DATA_PORT               PORTD
#define PIN_IN                  7
#define PIN_OUT                 6

#define INPUT_IS_SET            ( bit_is_set( PIND, PIN_IN ) )
#define INPUT_IS_CLEAR          ( bit_is_clear( PIND, PIN_IN ) )

#define OUT_SET                 ( bitSet(DATA_PORT, PIN_OUT) ) //( PORTD &= bit(PIN_OUT) ) //
#define OUT_CLEAR               ( bitClear(DATA_PORT, PIN_OUT) ) //( PORTD |= bit(PIN_OUT) ) //
  
//////// else for use inner comporator
  
//// AVC LAN bus directly connected to internal analog comparator (PD6/7)
//// PD6 AIN0 +
//// PD7 AIN1 -
//
//#define DATAIN_PIN              ACSR
//#define DATAIN                  ACO
//
//#define INPUT_IS_SET            ( bit_is_set( DATAIN_PIN, DATAIN ) )
//#define INPUT_IS_CLEAR          ( bit_is_clear( DATAIN_PIN, DATAIN ) )


/*--------------------------------------------------------------------------------------------------
                                       IE_BUS adress settings
--------------------------------------------------------------------------------------------------*/


#define HU_ADDRESS              0x130
#define MY_ADDRESS              0x140
#define BROADCAST_ADDRESS       0xFFF // All devices      //0x01FF // All audio devices
#define CONTROL_FLAGS           0xE


/*--------------------------------------------------------------------------------------------------
                                       Other settings
--------------------------------------------------------------------------------------------------*/


#define ONLY_MY                 true      // Recive all adres define this paremrte "false", or recive only for MY_ADDRESS or BROADCAST_ADDRESS define "ture"
#define SHOW_ERROR              false     // Turn "true" for print errors to serial port.

#define USART_BUFFER_SIZE       40

#if(!SHOW_ERROR)
  #define USART_BUFFER_SIZE     12
#endif

char UsartMsgBuffer[ USART_BUFFER_SIZE ];

unsigned long timerRegister = millis();


#endif // _AVCLANDRV_H_

/*--------------------------------------------------------------------------------------------------
                                         End of file.
--------------------------------------------------------------------------------------------------*/
