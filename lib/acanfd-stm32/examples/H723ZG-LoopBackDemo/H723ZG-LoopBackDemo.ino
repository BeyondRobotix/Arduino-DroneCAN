// LoopBackDemo

// This demo runs on NUCLEO_H723ZG
// The FDCAN1 module is configured in external loop back mode: it
// internally receives every CAN frame it sends, and emitted frames
// can be observed on TxCAN pin. No external hardware is required.

#ifndef ARDUINO_NUCLEO_H723ZG
  #error This sketch runs on NUCLEO-H723ZG Nucleo-144 board
#endif

//-----------------------------------------------------------------
// IMPORTANT:
//   <ACANFD_STM32.h> should be included only once, generally from the .ino file
//   From an other file, include <ACANFD_STM32_from_cpp.h>
//   Before including <ACANFD_STM32.h>, you should define
//   Message RAM size for FDCAN1 and Message RAM size for FDCAN2.
//   Maximum required size is 2,560 (2,560 32-bit words).
//   A 0 size means the FDCAN module is not configured; its TxCAN and RxCAN pins
//   can be freely used for an other function.
//   The begin method checks if actual size is greater or equal to required size.
//   Hint: if you do not want to compute required size, print
//   fdcan1.messageRamRequiredMinimumSize () for getting it.
//-----------------------------------------------------------------

static const uint32_t FDCAN1_MESSAGE_RAM_WORD_SIZE = 800 ;
static const uint32_t FDCAN2_MESSAGE_RAM_WORD_SIZE = 800 ;
static const uint32_t FDCAN3_MESSAGE_RAM_WORD_SIZE = 800 ;

#include <ACANFD_STM32.h>

//-----------------------------------------------------------------

void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
  Serial.begin (9600) ;
  while (!Serial) {
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    delay (50) ;
  }

  ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x4) ;

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

  settings.mModuleMode = ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK ;

  // settings.mTxPin = PB_9 ;
  // settings.mRxPin = PB_8 ;
  uint32_t errorCode = fdcan1.beginFD (settings) ;
  Serial.print ("FDCAN1 Message RAM required minimum size: ") ;
  Serial.print (fdcan1.messageRamRequiredMinimumSize ()) ;
  Serial.println (" words") ;
  if (0 == errorCode) {
    Serial.println ("fdcan1 ok") ;
  }else{
    Serial.print ("Error fdcan1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  // settings.mTxPin = 255 ;
  // settings.mRxPin = 255 ;
  errorCode = fdcan2.beginFD (settings) ;
  Serial.print ("FDCAN2 Message RAM required minimum size: ") ;
  Serial.print (fdcan2.messageRamRequiredMinimumSize ()) ;
  Serial.println (" words") ;
  if (0 == errorCode) {
    Serial.println ("fdcan2 ok") ;
  }else{
    Serial.print ("Error fdcan2: 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  // settings.mTxPin = 255 ;
  // settings.mRxPin = 255 ;
  errorCode = fdcan3.beginFD (settings) ;
  Serial.print ("FDCAN3 Message RAM required minimum size: ") ;
  Serial.print (fdcan3.messageRamRequiredMinimumSize ()) ;
  Serial.println (" words") ;
  if (0 == errorCode) {
    Serial.println ("fdcan3 ok") ;
  }else{
    Serial.print ("Error fdcan3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gSendDate1 = 0 ;
static uint32_t gSentCount1 = 0 ;
static uint32_t gReceivedCount1 = 0 ;

static uint32_t gSendDate2 = 0 ;
static uint32_t gSentCount2 = 0 ;
static uint32_t gReceivedCount2 = 0 ;

static uint32_t gSendDate3 = 0 ;
static uint32_t gSentCount3 = 0 ;
static uint32_t gReceivedCount3 = 0 ;

//-----------------------------------------------------------------

void loop () {
  if (gSendDate1 < millis ()) {
    // Serial.print ("CPU frequency: ") ;
    // Serial.print (F_CPU) ;
    // Serial.println (" Hz") ;
    // Serial.print ("PCLK1 frequency: ") ;
    // Serial.print (HAL_RCC_GetPCLK1Freq ()) ;
    // Serial.println (" Hz") ;
    // Serial.print ("PCLK2 frequency: ") ;
    // Serial.print (HAL_RCC_GetPCLK2Freq ()) ;
    // Serial.println (" Hz") ;
    // Serial.print ("HCLK frequency: ") ;
    // Serial.print (HAL_RCC_GetHCLKFreq ()) ;
    // Serial.println (" Hz") ;
    // Serial.print ("SysClock frequency: ") ;
    // Serial.print (HAL_RCC_GetSysClockFreq ()) ;
    // Serial.println (" Hz") ;
    // Serial.print ("CAN Clock: ") ;
    // Serial.print (fdcanClock ()) ;
    // Serial.println (" Hz") ;
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
    const uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
      gSendDate1 += 1000 ;
      gSentCount1 += 1 ;
      Serial.print ("FDCAN1 sent: ") ;
      Serial.println (gSentCount1) ;
    }
  }
  if (gSendDate2 < millis ()) {
    CANFDMessage message ;
    message.id = 0x7FE ;
    message.len = 8 ;
    message.data [0] = 0 ;
    message.data [1] = 1 ;
    message.data [2] = 2 ;
    message.data [3] = 3 ;
    message.data [4] = 4 ;
    message.data [5] = 5 ;
    message.data [6] = 6 ;
    message.data [7] = 7 ;
    const uint32_t sendStatus = fdcan2.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
      gSendDate2 += 999 ;
      gSentCount2 += 1 ;
      Serial.print ("FDCAN2 sent: ") ;
      Serial.println (gSentCount2) ;
    }
  }
if (gSendDate3 < millis ()) {
    CANFDMessage message ;
    message.id = 0x7FD ;
    message.len = 8 ;
    message.data [0] = 0 ;
    message.data [1] = 1 ;
    message.data [2] = 2 ;
    message.data [3] = 3 ;
    message.data [4] = 4 ;
    message.data [5] = 5 ;
    message.data [6] = 6 ;
    message.data [7] = 7 ;
    const uint32_t sendStatus = fdcan3.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
      gSendDate3 += 1001 ;
      gSentCount3 += 1 ;
      Serial.print ("FDCAN3 sent: ") ;
      Serial.println (gSentCount3) ;
    }
  }
  CANFDMessage messageFD ;
  if (fdcan1.receiveFD0 (messageFD)) {
    gReceivedCount1 += 1 ;
    Serial.print ("FDCAN1 received: ") ;
    Serial.println (gReceivedCount1) ;
  }
  if (fdcan2.receiveFD0 (messageFD)) {
    gReceivedCount2 += 1 ;
    Serial.print ("FDCAN2 received: ") ;
    Serial.println (gReceivedCount2) ;
  }
  if (fdcan3.receiveFD0 (messageFD)) {
    gReceivedCount3 += 1 ;
    Serial.print ("FDCAN3 received: ") ;
    Serial.println (gReceivedCount3) ;
  }
}
