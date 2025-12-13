/*
 *******************************************************************************
 * Copyright (c) 2020, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
// #if defined(ARDUINO_GENERIC_H742XGHX) || defined(ARDUINO_GENERIC_H742XIHX) ||\
//     defined(ARDUINO_GENERIC_H743XGHX) || defined(ARDUINO_GENERIC_H743XIHX) ||\
//     defined(ARDUINO_GENERIC_H745XGHX) || defined(ARDUINO_GENERIC_H745XIHX) ||\
//     defined(ARDUINO_GENERIC_H747XGHX) || defined(ARDUINO_GENERIC_H747XIHX) ||\
//     defined(ARDUINO_GENERIC_H750XBHX) || defined(ARDUINO_GENERIC_H753XIHX) ||\
//     defined(ARDUINO_GENERIC_H755XIHX) || defined(ARDUINO_GENERIC_H757XIHX)
#include "pins_arduino.h"

// Digital PinName array
const PinName digitalPin[] = {
    PA_0,   // D0/A0
    PA_1,   // D1/A1
    PA_2,   // D2/A2
    PA_3,   // D3/A3
    PA_4,   // D4/A4
    PA_5,   // D5/A5
    PA_6,   // D6/A6
    PA_7,   // D7/A7
    PA_8,   // D8
    PA_9,   // D9
    PA_10,  // D10
    PA_11,  // D11
    PA_12,  // D12
    PA_13,  // D13
    PA_14,  // D14
    PA_15,  // D15
    PB_0,   // D16/A8
    PB_1,   // D17/A9
    PB_2,   // D18
    PB_3,   // D19
    PB_4,   // D20
    PB_5,   // D21
    PB_6,   // D22
    PB_7,   // D23
    PB_8,   // D24
    PB_9,   // D25
    PB_10,  // D26
    PB_11,  // D27
    PB_12,  // D28
    PB_13,  // D29
    PB_14,  // D30
    PB_15,  // D31
    PC_0,   // D32/A10
    PC_1,   // D33/A11
    PC_2,   // D34/A12
    PC_3,   // D35/A13
    PC_4,   // D36/A14
    PC_5,   // D37/A15
    PC_6,   // D38
    PC_7,   // D39
    PC_8,   // D40
    PC_9,   // D41
    PC_10,  // D42
    PC_11,  // D43
    PC_12,  // D44
    PC_13,  // D45
    PC_14,  // D46
    PC_15,  // D47
    PD_0,   // D48
    PD_1,   // D49
    PD_2,   // D50
    PD_3,   // D51
    PD_4,   // D52
    PD_5,   // D53
    PD_6,   // D54
    PD_7,   // D55
    PD_8,   // D56
    PD_9,   // D57
    PD_10,  // D58
    PD_11,  // D59
    PD_12,  // D60
    PD_13,  // D61
    PD_14,  // D62
    PD_15,  // D63
    PE_0,   // D64
    PE_1,   // D65
    PE_2,   // D66
    PE_3,   // D67
    PE_4,   // D68
    PE_5,   // D69
    PE_6,   // D70
    PE_7,   // D71
    PE_8,   // D72
    PE_9,   // D73
    PE_10,  // D74
    PE_11,  // D75
    PE_12,  // D76
    PE_13,  // D77
    PE_14,  // D78
    PE_15,  // D79
    PF_0,   // D80
    PF_1,   // D81
    PF_2,   // D82
    PF_3,   // D83/A16
    PF_4,   // D84/A17
    PF_5,   // D85/A18
    PF_6,   // D86/A19
    PF_7,   // D87/A20
    PF_8,   // D88/A21
    PF_9,   // D89/A22
    PF_10,  // D90/A23
    PF_11,  // D91/A24
    PF_12,  // D92/A25
    PF_13,  // D93/A26
    PF_14,  // D94/A27
    PF_15,  // D95
    PG_0,   // D96
    PG_1,   // D97
    PG_2,   // D98
    PG_3,   // D99
    PG_4,   // D100
    PG_5,   // D101
    PG_6,   // D102
    PG_7,   // D103
    PG_8,   // D104
    PG_9,   // D105
    PG_10,  // D106
    PG_11,  // D107
    PG_12,  // D108
    PG_13,  // D109
    PG_14,  // D110
    PG_15,  // D111
    PH_0,   // D112
    PH_1,   // D113
    PH_2,   // D114/A28
    PH_3,   // D115/A29
    PH_4,   // D116/A30
    PH_5,   // D117/A31
    PH_6,   // D118
    PH_7,   // D119
    PH_8,   // D120
    PH_9,   // D121
    PH_10,  // D122
    PH_11,  // D123
    PH_12,  // D124
    PH_13,  // D125
    PH_14,  // D126
    PH_15,  // D127
    PI_0,   // D128
    PI_1,   // D129
    PI_2,   // D130
    PI_3,   // D131
    PI_4,   // D132
    PI_5,   // D133
    PI_6,   // D134
    PI_7,   // D135
    PI_8,   // D136
    PI_9,   // D137
    PI_10,  // D138
    PI_11,  // D139
    PI_12,  // D140
    PI_13,  // D141
    PI_14,  // D142
    PI_15,  // D143
    PJ_0,   // D144
    PJ_1,   // D145
    PJ_2,   // D146
    PJ_3,   // D147
    PJ_4,   // D148
    PJ_5,   // D149
    PJ_6,   // D150
    PJ_7,   // D151
    PJ_8,   // D152
    PJ_9,   // D153
    PJ_10,  // D154
    PJ_11,  // D155
    PJ_12,  // D156
    PJ_13,  // D157
    PJ_14,  // D158
    PJ_15,  // D159
    PK_0,   // D160
    PK_1,   // D161
    PK_2,   // D162
    PK_3,   // D163
    PK_4,   // D164
    PK_5,   // D165
    PK_6,   // D166
    PK_7,   // D167
    PA_0_C, // D168/A32
    PA_1_C, // D169/A33
    PC_2_C, // D170/A34
    PC_3_C  // D171/A35
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
    0,   // A0,  PA0
    1,   // A1,  PA1
    2,   // A2,  PA2
    3,   // A3,  PA3
    4,   // A4,  PA4
    5,   // A5,  PA5
    6,   // A6,  PA6
    7,   // A7,  PA7
    16,  // A8,  PB0
    17,  // A9,  PB1
    32,  // A10, PC0
    33,  // A11, PC1
    34,  // A12, PC2
    35,  // A13, PC3
    36,  // A14, PC4
    37,  // A15, PC5
    83,  // A16, PF3
    84,  // A17, PF4
    85,  // A18, PF5
    86,  // A19, PF6
    87,  // A20, PF7
    88,  // A21, PF8
    89,  // A22, PF9
    90,  // A23, PF10
    91,  // A24, PF11
    92,  // A25, PF12
    93,  // A26, PF13
    94,  // A27, PF14
    114, // A28, PH2
    115, // A29, PH3
    116, // A30, PH4
    117, // A31, PH5
    168, // A32, PA0_C
    169, // A33, PA1_C
    170, // A34, PC2_C
    171  // A35, PC3_C
};

// #endif /* ARDUINO_GENERIC_* */
/**
 * @brief  System Clock Configuration
 * @param  None
 * @retval None
 */
WEAK void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  /** Supply configuration update enable
   */
#if defined(SMPS)
  /** If SMPS is available on this MCU, assume that the MCU's board is
   *  built to power the MCU using the SMPS since it's more efficient.
   *  In this case, we must configure the MCU to use DIRECT_SMPS_SUPPLY.
   *
   *  N.B.: if the hardware configuration does not match the argument to
   *  HAL_PWREx_ConfigSupply(), the board will deadlock at this function call.
   *  This can manifest immediately or after a RESET/power cycle.
   *
   *  Trying to flash the board at this point will result in errors such as
   *  "No STM32 target found". To overcome this problem, erase the MCU's flash.
   *
   *  The following settings in STM32CubeProgrammer appear to work for this purpose:
   *   - Mode: Power down
   *   - Reset mode: Hardware reset
   */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
#else
  /** No SMPS available: use the internal voltage regulator (LDO).
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
#endif

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_CSI | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_CEC | RCC_PERIPHCLK_CKPER | RCC_PERIPHCLK_HRTIM1 | RCC_PERIPHCLK_I2C123 | RCC_PERIPHCLK_I2C4 | RCC_PERIPHCLK_LPTIM1 | RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_LPTIM3 | RCC_PERIPHCLK_QSPI | RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_SPI123 | RCC_PERIPHCLK_SPI45 | RCC_PERIPHCLK_SPI6 | RCC_PERIPHCLK_LPUART1 | RCC_PERIPHCLK_USART16 | RCC_PERIPHCLK_USART234578 | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_FDCAN;

  /** Initializes the CANBUS peripherals clock
   */

  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  PeriphClkInitStruct.CecClockSelection = RCC_CECCLKSOURCE_CSI;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Hrtim1ClockSelection = RCC_HRTIM1CLK_CPUCLK;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.Lptim345ClockSelection = RCC_LPTIM345CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
