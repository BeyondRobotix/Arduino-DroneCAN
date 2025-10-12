// CAN1 external LoopBackDemo for NUCLEO_H743ZI2
// No external hardware required.
// You can observe emitted CANFD frames on CANH / CANL pins.
// This sketch is an example of extended filters.
//-----------------------------------------------------------------

#ifndef ARDUINO_NUCLEO_H743ZI2
  #error This sketch runs on NUCLEO-H743ZI2 Nucleo-144 board
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

static const uint32_t FDCAN1_MESSAGE_RAM_WORD_SIZE = 2560 ;
static const uint32_t FDCAN2_MESSAGE_RAM_WORD_SIZE = 0 ; // FDCAN2 not used

#include <ACANFD_STM32.h>

//-----------------------------------------------------------------

void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  Serial.println ("CAN1 CANFD loopback test") ;
  ACANFD_STM32_Settings settings (1000 * 1000, DataBitRateFactor::x8) ;

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
  Serial.print ("Arbitration Sample point: ") ;
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
  Serial.print ("Data Sample point: ") ;
  Serial.print (settings.dataSamplePointFromBitStart ()) ;
  Serial.println ("%") ;
  Serial.print ("Exact Data Bit Rate ? ") ;
  Serial.println (settings.exactDataBitRate () ? "yes" : "no") ;

  settings.mModuleMode = ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK ;

  ACANFD_STM32_ExtendedFilters extendedFilters ;
//--- Add single filter: identifier (1 matching identifier)
  extendedFilters.addSingle (0x5555, ACANFD_STM32_FilterAction::FIFO0) ;
//--- Add dual filter: identifier1, identifier2 (2 matching identifiers)
  extendedFilters.addDual (0x3333, 0x4444, ACANFD_STM32_FilterAction::FIFO0) ;
//--- Add range filter: low bound, high bound (565 matching identifiers)
  extendedFilters.addRange (0x1000, 0x1234, ACANFD_STM32_FilterAction::FIFO1) ;
//--- Add classic filter: identifier and mask (32 matching identifiers)
  extendedFilters.addClassic (0x6789, 0x1FFF67BD, ACANFD_STM32_FilterAction::FIFO0) ;

//--- Reject extended frames that do not match any filter
  settings.mNonMatchingExtendedFrameReception = ACANFD_STM32_FilterAction::REJECT ;

// Therefore FIFO0 receives 1 + 2 + 32 = 35 frames, FIFO1 receives 565 frames.

//--- Allocate FIFO 1
  settings.mHardwareRxFIFO1Size = 10 ; // By default, 0
  settings.mDriverReceiveFIFO1Size = 10 ; // By default, 0

  const uint32_t errorCode = fdcan1.beginFD (settings, extendedFilters) ;

  Serial.print ("Message RAM required minimum size: ") ;
  Serial.print (fdcan1.messageRamRequiredMinimumSize ()) ;
  Serial.println (" words") ;

  if (0 == errorCode) {
    Serial.println ("can configuration ok") ;
  }else{
    Serial.print ("Error can configuration: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static const uint32_t PERIOD = 1000 ;
static uint32_t gBlinkDate = PERIOD ;
static uint32_t gSentIdentifier = 0 ;
static uint32_t gReceiveCountFIFO0 = 0 ;
static uint32_t gReceiveCountFIFO1 = 0 ;
static bool gOk = true ;

//-----------------------------------------------------------------

static void printCount (const uint32_t inActualCount, const uint32_t inExpectedCount) {
  Serial.print (", ") ;
  if (inActualCount == inExpectedCount) {
    Serial.print ("ok") ;
  }else{
    Serial.print (inActualCount) ;
    Serial.print ("/") ;
    Serial.print (inExpectedCount) ;
  }
}

//-----------------------------------------------------------------

void loop () {
  if (gOk && (gSentIdentifier <= 0x1FFFFFFF) && fdcan1.sendBufferNotFullForIndex (0)) {
    CANFDMessage frame ;
    frame.id = gSentIdentifier ;
    frame.ext = true ;
    gSentIdentifier += 1 ;
    const uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD (frame) ;
    if (sendStatus != 0) {
      gOk = false ;
      Serial.print ("Sent error 0x") ;
      Serial.println (sendStatus) ;
    }
  }
//--- Receive frame
  CANFDMessage frame ;
  if (gOk && fdcan1.receiveFD0 (frame)) {
    gReceiveCountFIFO0 += 1 ;
  }
  if (gOk && fdcan1.receiveFD1 (frame)) {
    gReceiveCountFIFO1 += 1 ;
  }
//--- Blink led and display
  if (gBlinkDate <= millis ()) {
    gBlinkDate += PERIOD ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Sent: ") ;
    Serial.print (gSentIdentifier) ;
    printCount (gReceiveCountFIFO0, 35) ;
    printCount (gReceiveCountFIFO1, 565) ;
    Serial.println () ;
  }
}

//-----------------------------------------------------------------
