#pragma once

//------------------------------------------------------------------------------

#include <ACANFD_STM32_from_cpp.h>

//------------------------------------------------------------------------------
//  NUCLEO-H743ZI2: STM32H743 (DS13313 Rev 3, Dec 2021)
//    FDCAN1_RX : PA11, PD0 (ZIO D67), PB8
//    FDCAN1_TX : PA12, PD1 (ZIO D66), PB9
//    FDCAN2_RX : PB12 (ZIO D19), PB5 (ZIO D11 or D22 ?)
//    FDCAN2_TX : PB13 (ZIO D18), PB6 (ZIO D1)
//    FDCAN3_RX : PF6, PD12, PG10
//    FDCAN3_TX : PF7, PD13, PG9
//------------------------------------------------------------------------------
//   FDCAN1
//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan1_tx_pin_array {
   ACANFD_STM32::PinPort (PD_1,  9), // Tx Pin: PD_1, AF9
   ACANFD_STM32::PinPort (PB_9,  9), // Tx Pin: PB_9, AF9
   ACANFD_STM32::PinPort (PA_12, 9)  // Tx Pin: PA_12, AF9
} ;

//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan1_rx_pin_array {
   ACANFD_STM32::PinPort (PD_0,  9), // Rx Pin: PD_0, AF9
   ACANFD_STM32::PinPort (PB_8,  9), // Rx Pin: PB_9, AF9
   ACANFD_STM32::PinPort (PA_11, 9)  // Rx Pin: PA_11, AF9
} ;

//------------------------------------------------------------------------------

ACANFD_STM32 fdcan1 (
  FDCAN1, // CAN Peripheral base address
  0, // Message RAM start word offset
  FDCAN1_MESSAGE_RAM_WORD_SIZE, // Message RAM word size
  ACANFD_STM32::IRQs (FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn), // Interrupts
  fdcan1_tx_pin_array,
  fdcan1_rx_pin_array
) ;

//------------------------------------------------------------------------------

extern "C" void FDCAN1_IT0_IRQHandler (void) ;
extern "C" void FDCAN1_IT1_IRQHandler (void) ;

//------------------------------------------------------------------------------

void FDCAN1_IT0_IRQHandler (void) {
  fdcan1.isr0 () ;
}

//------------------------------------------------------------------------------

void FDCAN1_IT1_IRQHandler (void){
  fdcan1.isr1 () ;
}

//------------------------------------------------------------------------------
//   FDCAN2
//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan2_tx_pin_array {
   ACANFD_STM32::PinPort (PB_6,  9), // Tx Pin: PB_6, AF9
   ACANFD_STM32::PinPort (PB_13, 9)  // Tx Pin: PB_13, AF9
} ;

//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan2_rx_pin_array {
   ACANFD_STM32::PinPort (PB_5,  9), // Rx Pin: PB_5, AF9
   ACANFD_STM32::PinPort (PB_12, 9)  // Rx Pin: PB_12, AF9
} ;

//------------------------------------------------------------------------------

ACANFD_STM32 fdcan2 (
  FDCAN2, // CAN Peripheral base address
  FDCAN1_MESSAGE_RAM_WORD_SIZE, // Message RAM start word offset
  FDCAN2_MESSAGE_RAM_WORD_SIZE, // Message RAM word size
  ACANFD_STM32::IRQs (FDCAN2_IT0_IRQn, FDCAN2_IT1_IRQn), // Interrupts
  fdcan2_tx_pin_array,
  fdcan2_rx_pin_array
) ;

//------------------------------------------------------------------------------

extern "C" void FDCAN2_IT0_IRQHandler (void) ;
extern "C" void FDCAN2_IT1_IRQHandler (void) ;

//------------------------------------------------------------------------------

void FDCAN2_IT0_IRQHandler (void) {
  fdcan2.isr0 () ;
}

//------------------------------------------------------------------------------

void FDCAN2_IT1_IRQHandler (void){
  fdcan2.isr1 () ;
}

//------------------------------------------------------------------------------
//   FDCAN3
//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan3_tx_pin_array {
   ACANFD_STM32::PinPort (PF_7,  3), // Tx Pin: PF_7, AF3
   ACANFD_STM32::PinPort (PD_13, 3), // Tx Pin: PD_13, AF3
   ACANFD_STM32::PinPort (PG_9,  2)  // Tx Pin: PG_9, AF2
} ;

//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan3_rx_pin_array {
   ACANFD_STM32::PinPort (PF_6,  2), // Rx Pin: PF_6, AF2
   ACANFD_STM32::PinPort (PD_12, 3), // Rx Pin: PD_12, AF3
   ACANFD_STM32::PinPort (PG_10, 2)  // Tx Pin: PG_10, AF2
} ;

//------------------------------------------------------------------------------

ACANFD_STM32 fdcan3 (
  FDCAN3, // CAN Peripheral base address
  FDCAN1_MESSAGE_RAM_WORD_SIZE + FDCAN2_MESSAGE_RAM_WORD_SIZE, // Message RAM start word offset
  FDCAN3_MESSAGE_RAM_WORD_SIZE, // Message RAM word size
  ACANFD_STM32::IRQs (FDCAN3_IT0_IRQn, FDCAN3_IT1_IRQn), // Interrupts
  fdcan3_tx_pin_array,
  fdcan3_rx_pin_array
) ;

//------------------------------------------------------------------------------

extern "C" void FDCAN3_IT0_IRQHandler (void) ;
extern "C" void FDCAN3_IT1_IRQHandler (void) ;

//------------------------------------------------------------------------------

void FDCAN3_IT0_IRQHandler (void) {
  fdcan3.isr0 () ;
}

//------------------------------------------------------------------------------

void FDCAN3_IT1_IRQHandler (void){
  fdcan3.isr1 () ;
}

//------------------------------------------------------------------------------
