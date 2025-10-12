//-----------------------------------------------------------------
// This demo runs on NUCLEO_G474RE
// The system clock are defined for getting a 160 MHz can clock
// The 3 FDCAN modules is configured in external loop back mode: it
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
//   WHY DEFINE CUSTOM SYSTEM CLOCK CONFIGURATION?
//-----------------------------------------------------------------
// In short: because default fdcan clock (168 MHz) can make a given bit rate unavailable.
// For example, I want 5 Mbit/s a data bit rate (arbitration bit rate is not significant for this problem)
// The 5 Mbit/s data bit rate is not available because 5 is not a divisor of 168.
// The only solution is to change the fdcan clock.
// We need a fdcan clock frequency multiple of 5 MHz, minimum 10 times 5 MHz, for allowing correct bit timing.
// Note setting a custom fdcan clock frequency affect also CPU speed. Note max CPU frequency is 170 MHz.
// A valid frequency is 160 MHz, but it would be valid to choose 170 MHz or 165 MHz.
//-----------------------------------------------------------------
//   DEFINE CUSTOM SYSTEM CLOCK CONFIGURATION
//-----------------------------------------------------------------
// For any board, System Clock can be overriden. I think it is valid, as it is described in:
//   https://github.com/stm32duino/Arduino_Core_STM32/wiki/Custom-definitions#systemclock_config
// For the NUCLEO_G474RE the SystemClock_Config function file is defined in STM32duino package. On my Mac, it is located at:
//    ~/Library/Arduino15/packages/STMicroelectronics/hardware/stm32/2.6.0/variants/STM32G4xx/G473R\(B-C-E\)T_G474R\(B-C-E\)T_G483RET_G484RET/variant_NUCLEO_G474RE.cpp
// The SystemClock_Config function is duplicated below and modified.
// For a full understanding of the clock tree, consider using STM32CubeMX
// The very very simplified explaination of the clock tree is:
//    - fdcan clock is PCLK1
//    - PCLK1 = HSE_CLOCK * PLLN / PLLM / PLLP
// HSE_CLOCK = 24 MHz, we cannot change that, it is given by STLink
// In the original file, PLLN=28, PLLM=2, PLLP=2 --> PCLK1 = 24 MHz * 28 / 2 / 2 = 168 MHz
// Caution: all values are not valid, I strongly suggest using STM32CubeMX for checking a given setting.
// For a PCLK1=160 MHz, it is valid to select PLLM=3 and PLLN=40: PCLK1 = 24 MHz * 40 / 3 / 2 = 160 MHz
// Note: for PLLM=3, use the RCC_PLLM_DIV3 symbol.
// For a PCLK1=170 MHz, it is valid to select PLLM=6 and PLLN=85: PCLK1 = 24 MHz * 85 / 6 / 2 = 170 MHz
// Note: for PLLM=6, use the RCC_PLLM_DIV6 symbol.
// For a PCLK1=165 MHz, it is valid to select PLLM=8 and PLLN=110: PCLK1 = 24 MHz * 110 / 8 / 2 = 165 MHz
// Note: for PLLM=8, use the RCC_PLLM_DIV8 symbol.
//
// Always validate your setting by checking actual CAN clock (call fdcanClock function, see below),
// and examine actual Data Bit Rate (call settings.actualDataBitRate function, see below)
//-----------------------------------------------------------------

extern "C" void SystemClock_Config (void) { // extern "C" IS REQUIRED!
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
#ifdef USBCON
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
#endif

  /* Configure the main internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV8 ; // Original value: RCC_PLLM_DIV2
  RCC_OscInitStruct.PLL.PLLN = 110 ; // Original value: 28
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK) {
    Error_Handler();
  }

#ifdef USBCON
  /* Initializes the peripherals clocks */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
#endif
}

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

  ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x10) ;
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
