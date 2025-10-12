// LoopBackDemo

// This demo runs on NUCLEO_G431KB
// The FDCAN module is configured in external loop back mode: it
// internally receives every CAN frame it sends, and emitted frames
// can be observed on TxCAN pin (PA12). No external hardware is required.

#ifndef ARDUINO_NUCLEO_G431KB
  #error This sketch runs on NUCLEO-G431KB Nucleo-32 board
#endif

//-----------------------------------------------------------------
// IMPORTANT:
//   <ACANFD_STM32.h> should be included only once, generally from the .ino file
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

  ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x5) ;

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

  uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan1 ok") ;
  }else{
    Serial.print ("Error can: 0x") ;
    Serial.println (errorCode, HEX) ;
    while (1) {
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
      delay (50) ;
    }
  }
  Serial.print ("Systick ") ;
  Serial.println (SysTick->LOAD + 1) ;
}

//-----------------------------------------------------------------

static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;

//-----------------------------------------------------------------

void loop () {
  if (gSendDate < millis ()) {
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
      // digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
      gSendDate += 1000 ;
      gSentCount += 1 ;
      Serial.print ("fdcan1 sent: ") ;
      Serial.println (gSentCount) ;
    }
  }
  CANFDMessage messageFD ;
  if (fdcan1.receiveFD0 (messageFD)) {
    gReceivedCount += 1 ;
    Serial.print ("fdcan1 received: ") ;
    Serial.println (gReceivedCount) ;
  }
}
