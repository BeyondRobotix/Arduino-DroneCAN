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
// #if defined(ARDUINO_GENERIC_L431CBTX) || defined(ARDUINO_GENERIC_L431CBUX) ||\
//     defined(ARDUINO_GENERIC_L431CCTX) || defined(ARDUINO_GENERIC_L431CCUX)
#include "pins_arduino.h"

// Digital PinName array
const PinName digitalPin[] = {
    PA_0,  // D0/A0
    PA_1,  // D1/A1
    PA_2,  // D2/A2
    PA_3,  // D3/A3
    PA_4,  // D4/A4
    PA_5,  // D5/A5
    PA_6,  // D6/A6
    PA_7,  // D7/A7
    PA_8,  // D8
    PA_9,  // D9
    PA_10, // D10
    PA_11, // D11
    PA_12, // D12
    PA_13, // D13
    PA_14, // D14
    PA_15, // D15
    PB_0,  // D16/A8
    PB_1,  // D17/A9
    PB_2,  // D18
    PB_3,  // D19
    PB_4,  // D20
    PB_5,  // D21
    PB_6,  // D22
    PB_7,  // D23
    PB_8,  // D24
    PB_9,  // D25
    PB_10, // D26
    PB_11, // D27
    PB_12, // D28
    PB_13, // D29
    PB_14, // D30
    PB_15, // D31
    PC_13, // D32
    PC_14, // D33
    PC_15, // D34
    PH_0,  // D35
    PH_1,  // D36
    PH_3   // D37
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
    0,  // A0,  PA0
    1,  // A1,  PA1
    2,  // A2,  PA2
    3,  // A3,  PA3
    4,  // A4,  PA4
    5,  // A5,  PA5
    6,  // A6,  PA6
    7,  // A7,  PA7
    16, // A8,  PB0
    17  // A9,  PB1
};

// #endif /* ARDUINO_GENERIC_* */
#ifdef __cplusplus
extern "C"
{
#endif
    WEAK void SystemClock_Config(void)
    {
        RCC_OscInitTypeDef RCC_OscInitStruct = {};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

        /** Configure the main internal regulator output voltage
         */
        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        {
            Error_Handler();
        }

        /** Initializes the RCC Oscillators according to the specified parameters
         * in the RCC_OscInitTypeDef structure.
         */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM = 1;
        RCC_OscInitStruct.PLL.PLLN = 10;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
        RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /** Initializes the CPU, AHB and APB buses clocks
         */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        {
            Error_Handler();
        }

        // Reinitialize SysTick to enable the delay() function
        // Configure SysTick to generate an interrupt every 1ms (SystemCoreClock / 1000)
        if (SysTick_Config(SystemCoreClock / 1000))
        {
            // If SysTick configuration fails, handle the error
            Error_Handler();
        }
        NVIC_EnableIRQ(SysTick_IRQn);
    }

#ifdef __cplusplus
}
#endif