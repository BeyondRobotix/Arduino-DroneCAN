//-----------------------------------------------------------------
// This demo runs on NUCLEO_G474RE
// It is similar to G474RE-Intensive, but use canfd alternate pins
// The can clock frequency is 168 MHz.
// The 3 FDCAN modules are configured in normal mode.
// Default TxCAN pins are used:
//   - fdcan1: PB_9
//   - fdcan2: PB_13
//   - fdcan3: PB_4
// Default RxCAN pins are used:
//   - fdcan1: PB_8
//   - fdcan2: PB_12
//   - fdcan3: PB_3
// External hardware: connect together all 6 pins: it realizes a can network without any transceiver.
// Constraints for this network:
//   - the TxPins should be configured as open collector outputs (otherwise we have short-circuits);
//   - the RxPins should be configured as pullup inputs (otherwise we can't have recessive high level);
//   - the data bit rate should be lower or equal than 500 kbit/s (no transceiver, the loop delay is shorter)
//   - do not send canfd frames with bit rate switch; only allowed:
//        * CAN 2.0B remote frames (message.type = CANFDMessage::CAN_REMOTE ;)
//        * CAN 2.0B data frames (message.type = CANFDMessage::CAN_DATA ;)
//        * CANFD frames without bit rate switch (message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ;)
//-----------------------------------------------------------------
// Each CANFD controller sends MESSAGE_COUNT frames (see definition below), and then stops sending
// So 3 * MESSAGE_COUNT frames pass through the network
// As in normal mode a canfd controller does not receive the frame it sends, each CANFD controller receives 2 * MESSAGE_COUNT frames.
// For avoiding dead-lock, two or more CANFD controller should not try to send frames with the same identifier.
// So:
//  - fdcan1 sends frames with identifier value equal to a (multiple of 3)
//  - fdcan2 sends frames with identifier value equal to a (multiple of 3) + 1
//  - fdcan3 sends frames with identifier value equal to a (multiple of 3) + 2
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

  ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x1) ;
  // settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD ; // Useless, as it is default value
  settings.mOpenCollectorOutput = true ; // Required, as all TxCAN pins are connected together
  settings.mInputPullup = true ; // Required, otherwise the recessive level cannot be ensured

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
  settings.mTxPin = PB_9 ; // DO NOT USE Pxy, Px_y
  settings.mRxPin = PB_8 ; // DO NOT USE Pxy, Px_y
  uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan1 configuration ok") ;
  }else{
    Serial.print ("Error fdcan1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }


  settings.mTxPin = PB_13 ; // DO NOT USE Pxy, Px_y
  settings.mRxPin = PB_12 ; // DO NOT USE Pxy, Px_y
  errorCode = fdcan2.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan2 configuration ok") ;
  }else{
    Serial.print ("Error fdcan2: 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  settings.mTxPin = PB_4 ; // DO NOT USE Pxy, Px_y
  settings.mRxPin = PB_3 ; // DO NOT USE Pxy, Px_y
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

static const uint32_t MESSAGE_COUNT = 100 * 1000 ;

//-----------------------------------------------------------------

void loop () {
  if (gSendDate < millis ()) {
    gSendDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
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
    Serial.print ("CAN Clock: ") ;
    Serial.print (fdcanClock ()) ;
    Serial.println (" Hz") ;
    Serial.print ("fdcan1 sent: ") ;
    Serial.print (gSentCount1) ;
    Serial.print (", received: ") ;
    Serial.println (gReceivedCount1) ;
    Serial.print ("fdcan2 sent: ") ;
    Serial.print (gSentCount2) ;
    Serial.print (", received: ") ;
    Serial.println (gReceivedCount2) ;
    Serial.print ("fdcan3 sent: ") ;
    Serial.print (gSentCount3) ;
    Serial.print (", received: ") ;
    Serial.println (gReceivedCount3) ;
  }
  CANFDMessage message ;
  message.ext = true ; // Extended identifier
  message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ; // DO NOT USE CANFD_WITH_BIT_RATE_SWITCH
  message.len = 5 ;
  for (uint8_t i=0 ; i<message.len ; i++) {
    message.data [i] = i ;
  }
//--- tryToSendReturnStatusFD returns 0 if message has been successfully
//    appended in transmit buffer.
//  A not zero returned value contains an error code (see doc)
  if (gSentCount1 < MESSAGE_COUNT) {
    message.id = gSentCount1 * 3 ;
    const uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      gSentCount1 += 1 ;
    }
  }
  if (gSentCount2 < MESSAGE_COUNT) {
    message.id = gSentCount2 * 3 + 1 ;
    const uint32_t sendStatus = fdcan2.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      gSentCount2 += 1 ;
    }
  }
  if (gSentCount3 < MESSAGE_COUNT) {
    message.id = gSentCount3 * 3 + 2 ;
    const uint32_t sendStatus = fdcan3.tryToSendReturnStatusFD (message) ;
    if (sendStatus == 0) {
      gSentCount3 += 1 ;
    }
  }
  CANFDMessage messageFD ;
  if (fdcan1.receiveFD0 (messageFD)) {
    gReceivedCount1 += 1 ;
  }
  if (fdcan2.receiveFD0 (messageFD)) {
    gReceivedCount2 += 1 ;
  }
  if (fdcan3.receiveFD0 (messageFD)) {
    gReceivedCount3 += 1 ;
  }
}
