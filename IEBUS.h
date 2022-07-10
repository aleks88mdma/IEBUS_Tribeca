/*--------------------------------------------------------------------------------------------------
  Name         :  IEBUS.h
  Description  :  IE BUS driver for Subaru devices.
  Author       :  Panin Aleksandr
  Copyright    :  (c) 2022 aleksandrpanin.ru
  ----------------------------------------------------------------------------------------------------
                                          AVC LAN Theory

     The AVC bus is an implementation of the IEBus which is a differential line, floating on logical
     level '1' and driving on logical '0'. Floating level shall be below 20 mV whereas driving level
     shall be above 120 mV.
     

     The diagram below represents how things work from a logical perspective on the bus.

     A rising edge indicates a new bit. The duration of the high state tells whether it is a start
     bit (~165 us), a bit '0' (~30 us) or a bit '1' (~20 us). A normal bit length is close to 40 us.

                       |<---- Bit '0' ---->|<---- Bit '1' ---->|
     Physical '1'      ,---------------,   ,---------,         ,---------
                       ^               |   ^         |         ^
     Physical '0' -----'               '---'         '---------'--------- Idle low
                       |---- 32 us ----| 7 |- 20 us -|- 19 us -|

     A bit '1' is typically 20 us high followed by 19 us low.

     A bit '0' is typically 32 us high followed by 7 us low. A bit '0' is dominant i.e. it takes
     precedence over a '1' by extending the pulse. This is why lower addresses win on arbitration.

     A start bit is typically 165 us high followed by 30 us low.

                                  AVC LAN Frame Format
     Bits Description

      1   Start bit
      1   MSG_NORMAL
      12  Master address
      1   Parity
      12  Slave address
      1   Parity
      1   * Acknowledge * (read below)
      4   Control
      1   Parity
      1   * Acknowledge * (read below)
      8   Payload length (n)
      1   Parity
      1   * Acknowledge * (read below)
          8   Data
          1   Parity
          1   * Acknowledge * (read below)
      repeat 'n' times

     In point-to-point communication, sender issues an ack bit with value '1' (20 us). Receiver
     upon acking will extend the bit until it looks like a '0' (32 us) on the bus. In broadcast
     mode, receiver disregards the bit.

     An acknowledge bit of value '0' means OK, '1' means no ack.

  --------------------------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------------------------
                                       IE_BUS timing settings
--------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------

                       |<---- Bit '0' ---->|<---- Bit '1' ---->|
     Physical '1'      ,---------------,   ,---------,         ,---------
                       ^               |   ^         |         ^
     Physical '0' -----'               '---'         '---------'--------- Idle low
                       |---- 33 us ----| 7 |- 20 us -|- 20 us -|

     A bit '0' is typically 33 us high followed by 7 us low.
     A bit '1' is typically 20 us high followed by 20 us low.
     A start bit is typically 165 us high followed by 30 us low.

--------------------------------------------------------------------------------------------------*/

// Following multipliers result of Timer 0 prescaler having 2 counts/us
#define NORMAL_BIT_LENGTH           10  //37*2

#define BIT_1_HOLD_ON_LENGTH        5   //20*2
#define BIT_0_HOLD_ON_LENGTH        9   //33*2
#define BIT_HOLD_HALF_PERIOD        7   //26*2 // Compare half way between a '1' (20 us) and a '0' (32 us ): 32 - (32 - 20) /2 = 26 us

#define START_BIT_LENGTH            47  //186*2 372
#define START_BIT_HOLD_ON_LENGTH    42  //168*2 336





/*--------------------------------------------------------------------------------------------------
                                       Type definitions
--------------------------------------------------------------------------------------------------*/
typedef unsigned char           byte;
typedef unsigned int            word;

typedef enum
{   // No this is not a mistake, broadcast = 0!
    MSG_NORMAL      = 1,
    MSG_BCAST       = 0

} AvcTransmissionMode;

//typedef enum{
//    ACT_NONE,
//    HU_NET_SCAN,
//    HU_PING,
//    DSP_PING
//
//} AvcActionID;

//typedef struct{
//    AvcActionID         ActionID;           // Action to perform after receiving this message.
//    byte                DataSize;           // Payload data size (bytes).
//    byte                Data[ 22 ];         // Payload data.
//    char                Description[ 17 ];  // ASCII description of the command for terminal dump.
//
//} AvcIncomingMessageStruct;
//
//typedef const AvcIncomingMessageStruct AvcInMessage;

typedef struct{
    AvcTransmissionMode Mode;               // Transmission mode: normal (1) or broadcast (0).
    byte                DataSize;           // Payload data size (bytes).
    byte                Data[ 22 ];         // Payload data.
    char                Description[ 17 ];  // ASCII description of the command for terminal dump.

} AvcOutgoingMessageStruct;

typedef const AvcOutgoingMessageStruct AvcOutMessage;

/*--------------------------------------------------------------------------------------------------
                                         Prototypes
--------------------------------------------------------------------------------------------------*/

bool            AvcReadMessage ( void );

//bool            AvcProcessActionID ( AvcActionID actionID );
//void            AvcUpdateStatus ( void );

void            DumpRawMessage ( bool incoming );








/*--------------------------------------------------------------------------------------------------
                                      Local Functions
  --------------------------------------------------------------------------------------------------*/
static void         SendStartBit ( void );
static void         Send12BitWord ( word data );
static void         Send8BitWord ( byte data );
static void         Send4BitWord ( byte data );
static void         Send1BitWord ( bool data );
static bool         SendMessage ( void );

static word         ReadBits ( byte nbBits );

static bool         HandleAcknowledge ( void );
static void         SendAcknowledge ( void );
static bool         IsAvcBusFree ( void );

//static AvcActionID  GetActionID ( void );
static void         LoadDataInGlogalRegisters ( AvcOutMessage * msg );
static bool         getStartBit ( void );

static void LedOff( void );
static void LedOn( void );



/*--------------------------------------------------------------------------------------------------
                                      Global Variables
  --------------------------------------------------------------------------------------------------*/
// Message frame global registers
static const char * Description;
static bool         Broadcast;
static word         MasterAddress;
static word         SlaveAddress;
static word         SlaveAddress2 = 0x000;
static byte         Control;
static byte         DataSize;
static bool         ParityBit;
static byte         Data[ 32 ];

static bool         isRegistred = false;
//static bool         isRegistredTest = false;
static byte         emulatorHandleBite = 0x00;
static int         isRegistredTimer = 4;

//unsigned long timing;
unsigned long lastRegistred;
//static bool emulatrorRegistred = false;


//bool                inMessageComplite = true;

//AvcActionID  DeviceEnabled   = ACT_NONE; //casting possibly unneccesary
//
//static AvcInMessage MessageTable [] PROGMEM =
//{
//
//// HU messages
//  { HU_NET_SCAN,        3, {0x10, 0x00, 0x01}, "HU NET SCAN" },
//  { HU_PING,        1, {0x1F}, "HU HOLD PING" },
//  { DSP_PING,        1, {0x12}, "Display Register ping" },
//  
//  { (AvcActionID)FALSE } //possibly should be ACT_NONE
//};
//
//const byte MessageTableSize = sizeof( MessageTable ) / sizeof( AvcInMessage );

/*--------------------------------------------------------------------------------------------------
                                    Our (CD) Commands
  --------------------------------------------------------------------------------------------------*/

AvcOutMessage CmdHuPing PROGMEM = { MSG_NORMAL, 1, {0x1F}, "Display ping" };
AvcOutMessage CmdDdisplayReg PROGMEM = { MSG_BCAST, 1, {0x12}, "Display register" };
AvcOutMessage CmdDdisplayRegPing PROGMEM = { MSG_NORMAL, 1, {0x1f}, "Display register" };

AvcOutMessage CmdDdisplayAnsver PROGMEM = { MSG_NORMAL, 5, {0x11, 0x00, 0x01, 0x01, 0x85}, "Ddisplay ansver from register request" };

AvcOutMessage CmdDdisplayAnsver2 PROGMEM = { MSG_NORMAL, 6, {0x11, 0x00, 0x01, 0x02, 0x85, 0x93}, "Ddisplay ansver from register request" };

/*--------------------------------------------------------------------------------------------------
  Name         :  AvcRegisterMe
  Description  :  Sends registration message to master controller.
  Argument(s)  :  None.
  Return value :  (bool) -> TRUE if successful else FALSE.
  --------------------------------------------------------------------------------------------------*/
bool AvcRegisterMe ( void ) {
  Broadcast     = MSG_BCAST; //MSG_NORMAL;
  MasterAddress = MY_ADDRESS;
  SlaveAddress  = BROADCAST_ADDRESS; //HU_ADDRESS;
  Control       = CONTROL_FLAGS;
  DataSize      = 1;
  Data[0]       = 0x12; //0x1F;
  
  return SendMessage();
}

/*--------------------------------------------------------------------------------------------------
  Name         :  AvcReadMessage
  Description  :  Read incoming messages on the AVC LAN bus.
  Argument(s)  :  None.
  Return value :  bool.
  --------------------------------------------------------------------------------------------------*/
bool AvcReadMessage ( void ) {

  if(INPUT_IS_CLEAR){
    return false;
  }
  
// Start bit.
  if( !getStartBit() ){
    return false;
  }

  LedOn();
  
  Broadcast = ReadBits( 1 );

  MasterAddress = ReadBits( 12 );
  bool p = ParityBit;
  if ( p != ReadBits( 1 ) ) {
    
    if(SHOW_ERROR){
  //    UsartPutCStr( PSTR("AvcReadMessage: Parity error @ MasterAddress! \r\n") );
      sprintf( UsartMsgBuffer, "AvcReadMessage: Parity error @ MasterAddress! B:0x%X M:0x%X \r\n", Broadcast, MasterAddress );
      Serial.print( UsartMsgBuffer );
    }
    return false;
  }

  SlaveAddress = ReadBits( 12 );
  p = ParityBit;
  if ( p != ReadBits( 1 ) ) {
    
    if(SHOW_ERROR){
  //    UsartPutCStr( PSTR("AvcReadMessage: Parity error @ SlaveAddress!\r\n") );
      sprintf( UsartMsgBuffer, "AvcReadMessage: Parity error @ SlaveAddress! B:0x%X M:0x%X S:0x%X \r\n", Broadcast, MasterAddress, SlaveAddress );
      Serial.print( UsartMsgBuffer );
    }
    return false;
  }


  bool forMe = ( SlaveAddress == MY_ADDRESS );

  // In point-to-point communication, sender issues an ack bit with value '1' (20us). Receiver
  // upon acking will extend the bit until it looks like a '0' (32us) on the bus. In broadcast
  // mode, receiver disregards the bit.

  if ( forMe ) {
    // Send ACK.
    SendAcknowledge();
  }  else {
    ReadBits( 1 );
  }

  Control = ReadBits( 4 );
  p = ParityBit;
  if ( p != ReadBits( 1 ) )    {
    
    if(SHOW_ERROR){
  //    UsartPutCStr( PSTR("AvcReadMessage: Parity error @ Control!\r\n") );
      sprintf( UsartMsgBuffer, "AvcReadMessage: Parity error @ Control! B:0x%X M:0x%X S:0x%X, C:0x%X \r\n", Broadcast, MasterAddress, SlaveAddress, Control );
      Serial.print( UsartMsgBuffer );
    }
  
    if(isRegistred && forMe){
      isRegistred = false;
    }
    
    return false;
  }

  if ( forMe )    {
    // Send ACK.
    SendAcknowledge();
  }    else    {
    ReadBits( 1 );
  }

  DataSize = ReadBits( 8 );
  p = ParityBit;
  if ( p != ReadBits( 1 ) )    {
    
    if(SHOW_ERROR){
  //    UsartPutCStr( PSTR("AvcReadMessage: Parity error @ DataSize!\r\n") );
      sprintf( UsartMsgBuffer, "AvcReadMessage: Parity error @ DataSize! B:0x%X M:0x%X S:0x%X, C:0x%X, L:0x%X \r\n", Broadcast, MasterAddress, SlaveAddress, Control, DataSize );
      Serial.print( UsartMsgBuffer );
    }
  
    if(isRegistred && forMe){
      isRegistred = false;
    }
    
    return false;
  }

  if ( forMe )    {
    // Send ACK.
    SendAcknowledge();
  }    else    {
    ReadBits( 1 );
  }

  byte i;

  for ( i = 0; i < DataSize; i++ )    {
    Data[i] = ReadBits( 8 );
    p = ParityBit;
    if ( p != ReadBits( 1 ) )        {
      
      if(SHOW_ERROR){
        sprintf( UsartMsgBuffer, "AvcReadMessage: Parity error @ Data[%d]\r\n", i );
        Serial.print( UsartMsgBuffer );
      }
  
      if(isRegistred && forMe){
        isRegistred = false;
      }
    
      return false;
    }

    if ( forMe )        {
      // Send ACK.
      SendAcknowledge();
//      Send1BitWord( 0 );
    }        else        {
      ReadBits( 1 );
    }
  }

  // Dump message on terminal.
//  if ( forMe ) UsartPutCStr( PSTR("AvcReadMessage: This message is for me!\r\n") );
//  if ( forMe ){
//    emulatrorRegistred = true;
//    lastRegistred = millis();
//  }


  if ( forMe ){
    isRegistred = true;
    lastRegistred = millis();
  }

//  inMessageComplite = true;


  // ====== Start Handle ping request from HU ======= //
  if(DataSize == 0x3 && Data[0] == 0x10 && Data[2] == 0x1){
    emulatorHandleBite = Data[1];
//    LoadDataInGlogalRegisters ( &CmdDdisplayAnsver );
    LoadDataInGlogalRegisters ( &CmdDdisplayAnsver2 );
    
    Data[1] = emulatorHandleBite;
    SendMessage();

    
    isRegistred = true;
    lastRegistred = millis();
    
    LedOff();
    return false;
  }
  // ====== End Handle ping request from HU ======= //




//  // ====== Start Serial print message ======= //
//  Serial.print( DataSize, HEX );
//  for ( byte i = 0; i < DataSize; i++ ) {
//    Serial.print(" ");
//    Serial.print( Data[i], HEX );
//  }
//  Serial.println();
//  // ====== End Serial print message ======= //



  if(ONLY_MY){
    if(forMe || (!Broadcast && SlaveAddress == BROADCAST_ADDRESS )){
      DumpRawMessage( false );
    }
  }
  else{
    DumpRawMessage( false );
  }

  
  LedOff();

  return false;
}


///*--------------------------------------------------------------------------------------------------
//  Name         :  AvcProcessActionID
//  Description  :  Perform processing for given action ID.
//  Argument(s)  :  actionID (AvcActionID) -> Action ID to process.
//  Return value :  (bool) -> TRUE if action performed.
//  --------------------------------------------------------------------------------------------------*/
//bool AvcProcessActionID ( AvcActionID actionID ){
//  // This function relies on the last received message still being loaded in global registers.
//
//
//  switch ( actionID ){
//    case ACT_NONE:
////
////      if (emulatrorRegistred && millis() - lastRegistred > 5000){
////        emulatrorRegistred = false;
////      }
////          
////      if (millis() - timing > 2000){
////        timing = millis();
////    
////        if(!emulatrorRegistred){
////    //      AvcRegisterMe();
////
////    
////        
////          // display ping signal start
//////          LoadDataInGlogalRegisters ( &CmdDdisplayReg );
//////          SendMessage();
////          // display ping signal start
////
////    
////          
////        }
////      }
//        
//
//      return TRUE;
//
//      break;
//
//    
//    default:
//      return false;
//      
//  }
//      return false;
//      
//  // Nothing to do!
////  return FALSE;
//}

/*--------------------------------------------------------------------------------------------------
  Name         :  LoadDataInGlogalRegisters
  Description  :  Loads message data in global registers for given mesage ID.
  Argument(s)  :  msg (AvcOutMessage *) -> Message to load.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void LoadDataInGlogalRegisters ( AvcOutMessage * msg ) {
//  Description = msg->Description;
  Broadcast = pgm_read_byte_near( &msg->Mode );
  MasterAddress = MY_ADDRESS;

  if ( Broadcast == MSG_BCAST ) {
    SlaveAddress = BROADCAST_ADDRESS;
  } else {
    SlaveAddress = HU_ADDRESS;
  }

  Control = CONTROL_FLAGS;

  DataSize = pgm_read_byte_near( &msg->DataSize );

  for ( byte i = 0; i < DataSize; i++ ) {
    Data[i] = pgm_read_byte_near( &msg->Data[i] );
  }
}

///*--------------------------------------------------------------------------------------------------
//  Name         :  GetActionID
//  Description  :  Use the last received message to determine the corresponding action ID.
//  Argument(s)  :  None.
//  Return value :  (AvcActionID) -> Action ID corresponding to current message.
//  --------------------------------------------------------------------------------------------------*/
//AvcActionID GetActionID ( void ) {
//  Description = PSTR("Unknown message!");
//
//  // Iterate through all HU messages in table.
//  for ( byte msg = 0; msg < MessageTableSize; msg++ ) {
//    bool found = TRUE;
//
//    // Identify current message from it's payload data.
//    for ( byte i = 0; i < pgm_read_byte_near( &MessageTable[msg].DataSize ); i++ ) {
//      if ( Data[i] != pgm_read_byte_near( &MessageTable[msg].Data[i] ) )  {
//        found = FALSE;
//        break;
//      }
//    }
//
//    if ( found )    {
//      Description = MessageTable[msg].Description;
//      // Fetch action corresponding to the message.
//      AvcActionID actionID = pgm_read_byte_near( &MessageTable[msg].ActionID );
//      return actionID;
//    }
//  }
//
//  return ACT_NONE;
//}

/*--------------------------------------------------------------------------------------------------
  Name         :  Send12BitWord
  Description  :  Writes a 12 bit word on the AVC LAN bus.
  Argument(s)  :  data (word) -> Data to write.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void Send12BitWord ( word data ){
  ParityBit = 0;

  // Most significant bit out first.
  for ( char nbBits = 0; nbBits < 12; nbBits++ )  {
    // Reset timer to measure bit length.
    TCNT0 = 0;

    // Drive output to signal high.
    OUT_SET;

    if ( data & 0x0800 )    {
      // Adjust parity.
      ParityBit = ! ParityBit;
      while ( TCNT0 < BIT_1_HOLD_ON_LENGTH );
    }    else    {
      while ( TCNT0 < BIT_0_HOLD_ON_LENGTH );
    }

    // Release output.
    OUT_CLEAR;

    // Fetch next bit.
    data <<= 1;

    // Hold output low until end of bit.
    while ( TCNT0 < NORMAL_BIT_LENGTH );
  }
}

/*--------------------------------------------------------------------------------------------------
  Name         :  Send8BitWord
  Description  :  Writes an 8 bit word on the AVC LAN bus.
  Argument(s)  :  data (byte) -> Data to write.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void Send8BitWord ( byte data ){
  ParityBit = 0;

  // Most significant bit out first.
  for ( char nbBits = 0; nbBits < 8; nbBits++ )  {
    // Reset timer to measure bit length.
    TCNT0 = 0;

    // Drive output to signal high.
    OUT_SET;

    if ( data & 0x80 ) {
      // Adjust parity.
      ParityBit = ! ParityBit;
      while ( TCNT0 < BIT_1_HOLD_ON_LENGTH );
    } else {
      while ( TCNT0 < BIT_0_HOLD_ON_LENGTH );
    }

    // Release output.
    OUT_CLEAR;

    // Fetch next bit.
    data <<= 1;

    // Hold output low until end of bit.
    while ( TCNT0 < NORMAL_BIT_LENGTH );
  }
}

/*--------------------------------------------------------------------------------------------------
  Name         :  Send4BitWord
  Description  :  Writes a 4 bit word on the AVC LAN bus.
  Argument(s)  :  data (byte) -> Data to write.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void Send4BitWord ( byte data ){
  ParityBit = 0;

  // Most significant bit out first.
  for ( char nbBits = 0; nbBits < 4; nbBits++ )  {
    // Reset timer to measure bit length.
    TCNT0 = 0;

    // Drive output to signal high.
    OUT_SET;

    if ( data & 0x8 )  {
      // Adjust parity.
      ParityBit = ! ParityBit;
      while ( TCNT0 < BIT_1_HOLD_ON_LENGTH );
    }    else    {
      while ( TCNT0 < BIT_0_HOLD_ON_LENGTH );
    }

    // Release output.
    OUT_CLEAR;

    // Fetch next bit.
    data <<= 1;

    // Hold output low until end of bit.
    while ( TCNT0 < NORMAL_BIT_LENGTH );
  }
  
}

/*--------------------------------------------------------------------------------------------------
  Name         :  Send1BitWord
  Description  :  Writes a 1 bit word on the AVC LAN bus.
  Argument(s)  :  data (bool) -> Data to write.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void Send1BitWord ( bool data ){
  // Reset timer to measure bit length.
  TCNT0 = 0;
  // Drive output to signal high.
  OUT_SET;

  if ( data )  {
    while ( TCNT0 < BIT_1_HOLD_ON_LENGTH );
  }  else  {
    while ( TCNT0 < BIT_0_HOLD_ON_LENGTH );
  }

  // Release output.
  OUT_CLEAR;

  // Pulse level low duration until 40 us.
  while ( TCNT0 <  NORMAL_BIT_LENGTH );
}

/*--------------------------------------------------------------------------------------------------
  Name         :  SendStartBit
  Description  :  Writes a start bit on the AVC LAN bus.
  Argument(s)  :  None.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void SendStartBit ( void ){
  // Reset timer to measure bit length.
  TCNT0 = 0;

  // Drive output to signal high.
  OUT_SET;

  // Pulse level high duration.
  while ( TCNT0 < START_BIT_HOLD_ON_LENGTH );

  // Release output.
  OUT_CLEAR;

  // Pulse level low duration until ~185 us.
  while ( TCNT0 < START_BIT_LENGTH );
}

/*--------------------------------------------------------------------------------------------------
  Name         :  ReadBits
  Description  :  Reads specified number of bits from the AVC LAN bus.
  Argument(s)  :  nbBits (byte) -> Number of bits to read.
  Return value :  (word) -> Data value read.
                       |<---- Bit '0' ---->|<---- Bit '1' ---->|
     Physical '1'      ,---------------,   ,---------,         ,---------
                       ^               |   ^         |         ^
     Physical '0' -----'               '---'         '---------'--------- Idle low
                       |---- 32 us ----| 7 |- 20 us -|- 19 us -|
  --------------------------------------------------------------------------------------------------*/
word ReadBits ( byte nbBits ){
  word data = 0;

  ParityBit = 0;

  while ( nbBits-- > 0 )  {
    // Insert new bit
    data <<= 1;

    // Wait until rising edge of new bit.
    while ( INPUT_IS_CLEAR );

    // Reset timer to measure bit length.
    TCNT0 = 0;

    // Wait until falling edge.
    while ( INPUT_IS_SET );
    
    // Compare half way between a '1' (20 us) and a '0' (32 us ): 32 - (32 - 20) /2 = 26 us
    if ( TCNT0 < BIT_HOLD_HALF_PERIOD )    {
      // Set new bit.
      data |= 0x0001;

      // Adjust parity.
      ParityBit = ! ParityBit;
      
      while(TCNT0 < BIT_0_HOLD_ON_LENGTH);
    }
    
  }

  return data;
}


/*--------------------------------------------------------------------------------------------------
  Name         :  getStartBit
  Description  :  Reads specified number of bits from the AVC LAN bus.
  Argument(s)  :  nbBits (byte) -> Number of bits to read.
  Return value :  (word) -> Data value read.
                       |<---- Bit '0' ---->|<---- Bit '1' ---->|
     Physical '1'      ,---------------,   ,---------,         ,---------
                       ^               |   ^         |         ^
     Physical '0' -----'               '---'         '---------'--------- Idle low
                       |---- 32 us ----| 7 |- 20 us -|- 19 us -|
  --------------------------------------------------------------------------------------------------*/
bool getStartBit ( void ){
  
  // Wait until rising edge of new bit.
  while ( INPUT_IS_CLEAR ) {
    // Reset watchdog.
    wdt_reset();
  }

  // Reset timer to measure bit length.
  TCNT0 = 0;

  // Wait until falling edge.
  while ( INPUT_IS_SET );
  
  if ( TCNT0 > START_BIT_HOLD_ON_LENGTH - 2 && TCNT0 < START_BIT_LENGTH) {
    return true;
  }
  else{
    return false;
  }

}

/*--------------------------------------------------------------------------------------------------
  Name         :  SendMessage
  Description  :  Sends the message in global registers on the AVC LAN bus.
  Argument(s)  :  None.
  Return value :  (bool) -> TRUE if successful else FALSE.
  --------------------------------------------------------------------------------------------------*/
bool SendMessage ( void ){
  
  while ( ! IsAvcBusFree() );
  // At this point we know the bus is available.
  LedOn();

  // Send start bit.
  SendStartBit();
  

  // Broadcast bit.
  Send1BitWord( Broadcast );

  // Master address = me.
  Send12BitWord( MasterAddress );
  Send1BitWord( ParityBit );

  // Slave address = head unit (HU).
  Send12BitWord( SlaveAddress );
  Send1BitWord( ParityBit );
  

  if ( ! HandleAcknowledge() ) {
    
    if(SHOW_ERROR){
      DumpRawMessage( true );
      Serial.print( (char*)"SendMessage: No Ack @ Slave address\r\n" );
    }
    
    return false;
  }

  // Control flag + parity.
  Send4BitWord( Control );
  Send1BitWord( ParityBit );

  if ( ! HandleAcknowledge() ) {
    
    if(SHOW_ERROR){
      DumpRawMessage( true );
      Serial.print( (char*)"SendMessage: No Ack @ Control\r\n" );
    }
    
    return false;
  }

  // Data length + parity.
  Send8BitWord( DataSize );
  Send1BitWord( ParityBit );

  if ( ! HandleAcknowledge() ) {
    
    if(SHOW_ERROR){
      DumpRawMessage( true );
      Serial.print( (char*)"SendMessage: No Ack @ DataSize\r\n" );
    }
    
    return false;
  }

  for ( byte i = 0; i < DataSize; i++ )  {
    Send8BitWord( Data[i] );
    Send1BitWord( ParityBit );

    if ( ! HandleAcknowledge() )  {
    
      if(SHOW_ERROR){
        DumpRawMessage( true );
        sprintf( UsartMsgBuffer, "SendMessage: No Ack @ Data[%d]\r\n", i );
        Serial.print( UsartMsgBuffer );
      }
      
      return false;
    }
  }


      
  DumpRawMessage( true );

  LedOff();
  return true;
}



/*--------------------------------------------------------------------------------------------------
  Name         :  HandleAcknowledge
  Description  :  Sends ack bit if I am broadcasting otherwise wait and return received ack bit.
  Argument(s)  :  None.
  Return value :  (bool) -> FALSE if ack bit not detected.
  --------------------------------------------------------------------------------------------------*/
bool HandleAcknowledge ( void ){
  
  if ( Broadcast == MSG_BCAST )  {
    // Acknowledge.
    Send1BitWord( 0 );
    return true;
  }

  // The acknowledge pattern is very tricky: the sender shall drive the bus for the equivalent
  // of a bit '1' (20 us) then release the bus and listen. At this point the target shall have
  // taken over the bus maintaining the pulse until the equivalent of a bit '0' (32 us) is formed.

  // Reset timer to measure bit length.
  TCNT0 = 0;

  // Drive output to signal high.
  OUT_SET;

  // Generate bit '0'.
  while ( TCNT0 < BIT_1_HOLD_ON_LENGTH );

  // Release output.
  OUT_CLEAR;

  // Measure final resulting bit.
  while ( INPUT_IS_SET );

  // Sample half-way through bit '0' (26 us) to detect whether the target is acknowledging.
  if ( TCNT0 > BIT_HOLD_HALF_PERIOD ) {
    // Slave is acknowledging (ack = 0). Wait until end of ack bit.
    while ( TCNT0 <  NORMAL_BIT_LENGTH );
    return true;
  }
  else{
    while ( TCNT0 <  NORMAL_BIT_LENGTH );
    return false;
  }

}

/*--------------------------------------------------------------------------------------------------
  Name         :  SendAcknowledge
  Description  :  Sends ack bit if I am broadcasting otherwise wait and return received ack bit.
  Argument(s)  :  None.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void SendAcknowledge ( void ){
  
  while ( INPUT_IS_CLEAR )    {
    // Reset watchdog.
    wdt_reset();
  }
  
  // Reset timer to measure bit length.
  TCNT0 = 0;
  while ( TCNT0 < 1 );

  // Drive output to signal high.
  OUT_SET;

  // Generate bit '0'.
  while ( TCNT0 < BIT_0_HOLD_ON_LENGTH );

  // Release output.
  OUT_CLEAR;
  
}

/*--------------------------------------------------------------------------------------------------
  Name         :  IsAvcBusFree
  Description  :  Determine whether the bus is free (no tx/rx).
  Argument(s)  :  None.
  Return value :  (bool) -> TRUE is bus is free.
  --------------------------------------------------------------------------------------------------*/
bool IsAvcBusFree ( void ){
  // Reset timer.
  TCNT0 = 0;

  while ( INPUT_IS_CLEAR ) {
    // We assume the bus is free if anything happens for the length of 1 bit.
    if ( TCNT0 > NORMAL_BIT_LENGTH )
    {
      return true;
    }
  }
  return false;
}

/*--------------------------------------------------------------------------------------------------
  Name         :  DumpRawMessage
  Description  :  Dumps raw content of message registers on the terminal.
  Argument(s)  :  incoming (bool) -> TRUE means incoming data, FALSE means outgoing.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/
void DumpRawMessage ( bool incoming ){
  // Dump message on terminal.

//  if ( incoming )
//    Serial.print("I:OUT ");
//  else
//    Serial.print("I:IN  ");

  sprintf( UsartMsgBuffer, "B:%d ", Broadcast );
  Serial.print( UsartMsgBuffer );
  
  #if (USE_SOFTSERIAL)
    altSerial.print( UsartMsgBuffer );
  #endif

  sprintf( UsartMsgBuffer, "M:0X%0X ", MasterAddress );
  Serial.print( UsartMsgBuffer );
  
  #if (USE_SOFTSERIAL)
    altSerial.print( UsartMsgBuffer );
  #endif

  sprintf( UsartMsgBuffer, "S:0X%0X ", SlaveAddress );
  Serial.print( UsartMsgBuffer );
  
  #if (USE_SOFTSERIAL)
    altSerial.print( UsartMsgBuffer );
  #endif

  sprintf( UsartMsgBuffer, "CB:0X%0X ", Control );
  Serial.print( UsartMsgBuffer );
  
  #if (USE_SOFTSERIAL)
    altSerial.print( UsartMsgBuffer );
  #endif

  sprintf( UsartMsgBuffer, "L:%0d ", DataSize );
  Serial.print( UsartMsgBuffer );
  
  #if (USE_SOFTSERIAL)
    altSerial.print( UsartMsgBuffer );
  #endif

  sprintf( UsartMsgBuffer, "DATA: " );
  Serial.print( UsartMsgBuffer );
  
  #if (USE_SOFTSERIAL)
    altSerial.print( UsartMsgBuffer );
  #endif

  for ( byte i = 0; i < DataSize; i++ ) {
    sprintf( UsartMsgBuffer, "0X%0X ", Data[i] );
    Serial.print( UsartMsgBuffer );
  
    #if (USE_SOFTSERIAL)
      altSerial.print( UsartMsgBuffer );
    #endif
  }

  Serial.println();
  
  #if (USE_SOFTSERIAL)
    altSerial.println();
  #endif


//  if(incoming){
////    ctrlSerial.print("$");
//    for ( byte i = 0; i < DataSize; i++ ) {
//      if(i > 0)
//        ctrlSerial.print(",");
//      
//      sprintf( UsartMsgBuffer, "%0X ", Data[i] );
//      ctrlSerial.print(UsartMsgBuffer);
//    }
//    ctrlSerial.print(";");
//    ctrlSerial.println();
//  }

}

/*--------------------------------------------------------------------------------------------------
  Name         :  LedOn/LedOff
  Description  :  Toggle onboard Led.
  Argument(s)  :  None.
  Return value :  None.
  --------------------------------------------------------------------------------------------------*/

void LedOn ( void ){
    LED_PORT &= ~LEDOUT;
}

void LedOff ( void ){
    LED_PORT |= LEDOUT;
}

/*--------------------------------------------------------------------------------------------------
                                         End of file.
  --------------------------------------------------------------------------------------------------*/
