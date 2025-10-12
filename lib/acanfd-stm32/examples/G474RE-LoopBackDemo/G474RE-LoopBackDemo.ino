//-----------------------------------------------------------------
// This demo runs on NUCLEO_G474RE
// The can clock frequency is 168 MHz.
// The 3 FDCAN modules are configured in external loop back mode: it
// internally receives every CAN frame it sends, and emitted frames
// can be observed on TxCAN pins. No external hardware is required.
// Default TxCAN pins are used:
//   - fdcan1: PA_12
//   - fdcan2: PB_6
//   - fdcan3: PA_15
//-----------------------------------------------------------------

#ifndef ARDUINO_NUCLEO_G474RE
  #error This sketch runs on NUCLEO-G474RE Nucleo-64 board
#endif

//-----------------------------------------------------------------
// IMPORTANT:
//   <ACANFD_STM32.h> should be included only once in a sketch, generally from the .ino file
//   From an other file, include <ACANFD_STM32_from_cpp.h>
//-----------------------------------------------------------------

#include <ACANFD_STM32.h>

//-----------------------------------------------------------------

void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    delay (50) ;
  }

  Serial.print ("CPU frequency: ") ;
  Serial.print (F_CPU) ;
  Serial.println (" Hz") ;
  Serial.print ("PCLK1 frequency: ") ;
  Serial.print (HAL_RCC_GetPCLK1Freq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("PCLK2 frequency: ") ;
  Serial.print (HAL_RCC_GetPCLK2Freq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("HCLK frequency: ") ;
  Serial.print (HAL_RCC_GetHCLKFreq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("SysClock frequency: ") ;
  Serial.print (HAL_RCC_GetSysClockFreq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("FDCAN Clock: ") ;
  Serial.print (fdcanClock ()) ;
  Serial.println (" Hz") ;

  ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x2) ;
  settings.mModuleMode = ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK ;

  Serial.print ("Bit Rate prescaler: ") ;
  Serial.println (settings.mBitRatePrescaler) ;
  Serial.print ("Arbitration Phase segment 1: ") ;
  Serial.println (settings.mArbitrationPhaseSegment1) ;
  Serial.print ("Arbitration Phase segment 2: ") ;
  Serial.println (settings.mArbitrationPhaseSegment2) ;
  Serial.print ("Arbitration SJW: ") ;
  Serial.println (settings.mArbitrationSJW) ;
  Serial.print ("Actual Arbitration Bit Rate: ") ;
  Serial.print (settings.actualArbitrationBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Arbitration sample point: ") ;
  Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
  Serial.println ("%") ;
  Serial.print ("Exact Arbitration Bit Rate ? ") ;
  Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
  Serial.print ("Data Phase segment 1: ") ;
  Serial.println (settings.mDataPhaseSegment1) ;
  Serial.print ("Data Phase segment 2: ") ;
  Serial.println (settings.mDataPhaseSegment2) ;
  Serial.print ("Data SJW: ") ;
  Serial.println (settings.mDataSJW) ;
  Serial.print ("Actual Data Bit Rate: ") ;
  Serial.print (settings.actualDataBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Data sample point: ") ;
  Serial.print (settings.dataSamplePointFromBitStart ()) ;
  Serial.println ("%") ;
  Serial.print ("Exact Data Bit Rate ? ") ;
  Serial.println (settings.exactDataBitRate () ? "yes" : "no") ;

//--- beginFD is called without any receive filter, all sent frames are received
//    by receiveFD0 throught receiveFIFO0
  uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan1 configuration ok") ;
  }else{
    Serial.print ("Error fdcan1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }


  errorCode = fdcan2.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan2 configuration ok") ;
  }else{
    Serial.print ("Error fdcan2: 0x") ;
    Serial.println (errorCode, HEX) ;
  }


  errorCode = fdcan3.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan3 configuration ok") ;
  }else{
    Serial.print ("Error fdcan3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gSendDate = 0 ;
static uint32_t gSentCount1 = 0 ;
static uint32_t gReceivedCount1 = 0 ;
static uint32_t gSentCount2 = 0 ;
static uint32_t gReceivedCount2 = 0 ;
static uint32_t gSentCount3 = 0 ;
static uint32_t gReceivedCount3 = 0 ;

//-----------------------------------------------------------------

void loop () {
  if (gSendDate < millis ()) {
    gSendDate += 1000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    CANFDMessage message ;
    message.id = 0x7FF ;
    message.len = 8 ;
    message.data [0] = 0 ;
    message.data [1] = 1 ;
    message.data [2] = 2 ;
    message.data [3] = 3 ;
    message.data [4] = 4 ;
    message.data [5] = 5 ;
    message.data [6] = 6 ;
    message.data [7] = 7 ;
//--- tryToSendReturnStatusFD returns 0 if message has been successfully
//    appended in transmit buffer.
//  A not zero returned value contains an error code (see doc)
    uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      gSentCount1 += 1 ;
      Serial.print ("fdcan1 sent: ") ;
      Serial.println (gSentCount1) ;
    }
    sendStatus = fdcan2.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      gSentCount2 += 1 ;
      Serial.print ("fdcan2 sent: ") ;
      Serial.println (gSentCount2) ;
    }
    sendStatus = fdcan3.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      gSentCount3 += 1 ;
      Serial.print ("fdcan3 sent: ") ;
      Serial.println (gSentCount3) ;
    }
  }
  CANFDMessage messageFD ;
  if (fdcan1.receiveFD0 (messageFD)) {
    gReceivedCount1 += 1 ;
    Serial.print ("fdcan1 received: ") ;
    Serial.println (gReceivedCount1) ;
  }
  if (fdcan2.receiveFD0 (messageFD)) {
    gReceivedCount2 += 1 ;
    Serial.print ("fdcan2 received: ") ;
    Serial.println (gReceivedCount2) ;
  }
  if (fdcan3.receiveFD0 (messageFD)) {
    gReceivedCount3 += 1 ;
    Serial.print ("fdcan3 received: ") ;
    Serial.println (gReceivedCount3) ;
  }
}
