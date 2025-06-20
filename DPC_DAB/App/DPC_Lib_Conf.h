/**
******************************************************************************
* @file    DPC_Lib_Conf.h
* @brief   This file contains the DPC Library Configuration.
******************************************************************************
*
* COPYRIGHT(c) 2020 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DPC_LIB_CONF_H
#define __DPC_LIB_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal_conf.h"
#include "DPC_Application_Conf.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define DEBUG_DAC
#define NO_____LUT_GRID
#define ADC_GRID
#define DISPLAY_USE
#define STDES_DABBIDIR
#define TELEMETRY_OFF
#define __TELEMETRY_ON
#define TIMEOUT_USED
#define ___USE_ADVTIM
#define USE_HRTIM
#define ____DPC_ARM_MATH               //

//PWM Generation Timer configuration Section
// Tipology: - DPC_PWM_1ADVTIM_3CH, use 1 Advanced Control Timer with 3 channel.
//           - DPC_PWM_2ADVTIM_3CH_3CHX, use 1 Advanced Control Timer with 3 channel + complementary channels.

///__Start_________________________________________________________________STDES_DABBIDIR_______________________________________________________________________
#ifdef STDES_DABBIDIR

#ifdef USE_ADVTIM                                               //ifdef
#define DPC_PWM_2ADVTIM_2CH_2CHX                                                ///Use 2 Adv. Tim. 3 channel + 3 complementary channel
#define PWM_Tim1                        htim1                                 /*!<PWM TIMER - PWM group 1*/
#define PWM_Tim2                        htim8                                   /*!<PWM TIMER - PWM group 2*/
#define PWM_CHANNEL_1                   TIM_CHANNEL_1                           /*!<PWM Channel Phase A (Power Board)*/
#define PWM_CHANNEL_1N                  TIM_CHANNEL_1                           /*!<PWM Channel Phase A (Power Board)*/
#define PWM_CHANNEL_2                   TIM_CHANNEL_2                           /*!<PWM Channel Phase B (Power Board)*/
#define PWM_CHANNEL_2N                  TIM_CHANNEL_2                           /*!<PWM Channel Phase B (Power Board)*/
#endif                                                          //andif

#ifdef USE_HRTIM                                                //ifdef
#define DPC_PWM_1HRTIM_4CH_4CHX                                                 ///Use 1 HR. Tim. 4 channel + 4 complementary channel
#define PWM_Tim1                        hhrtim1                                   /*!<PWM TIMER - PWM group 1*/
#define PWM_CHANNEL_1                   HRTIM_OUTPUT_TA1                        /*!<PWM Channel HighVoltage SX High Side "M1" (PowerBoard)*/
#define PWM_CHANNEL_1N                  HRTIM_OUTPUT_TA2                        /*!<PWM Channel HighVoltage SX Low Side "M2" (Power Board)*/
#define PWM_CHANNEL_2                   HRTIM_OUTPUT_TB1                        /*!<PWM Channel HighVoltage DX High Side "M3" (PowerBoard)*/
#define PWM_CHANNEL_2N                  HRTIM_OUTPUT_TB2                        /*!<PWM Channel HighVoltage DX Low Side "M4" (Power Board)*/
#define PWM_CHANNEL_3                   HRTIM_OUTPUT_TC1                        /*!<PWM Channel LowVoltage SX High Side "M5" (Power Board)*/
#define PWM_CHANNEL_3N                  HRTIM_OUTPUT_TC2                        /*!<PWM Channel LowVoltage SX Low Side "M6" (Power Board)*/
#define PWM_CHANNEL_4                   HRTIM_OUTPUT_TD1                        /*!<PWM Channel LowVoltage DX High Side "M7" (Power Board)*/
#define PWM_CHANNEL_4N                  HRTIM_OUTPUT_TD2                        /*!<PWM Channel LowVoltage DX Low Side "M8" (Power Board)*/
#define PWM_CHANNEL_5                   HRTIM_OUTPUT_TE1                        /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_CHANNEL_5N                  HRTIM_OUTPUT_TE2                        /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_CHANNEL_6                   HRTIM_OUTPUT_TF1                        /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_CHANNEL_6N                  HRTIM_OUTPUT_TF2                        /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_IDX_CH_1                    HRTIM_TIMERINDEX_TIMER_A                /*!<PWM Index Channel HighVoltage SX (Power Board)*/
#define PWM_IDX_CH_2                    HRTIM_TIMERINDEX_TIMER_B                /*!<PWM Index Channel HighVoltage DX (Power Board)*/
#define PWM_IDX_CH_3                    HRTIM_TIMERINDEX_TIMER_C                /*!<PWM Index Channel LowVoltage SX (Power Board)*/
#define PWM_IDX_CH_4                    HRTIM_TIMERINDEX_TIMER_D                /*!<PWM Index Channel LowVoltage DX (Power Board)*/
#define PWM_IDX_CH_5                    HRTIM_TIMERINDEX_MASTER                 /*!<PWM Index MASTER Sync (Power Board)*/
//#define PWM_IDX_CH_6                    HRTIM_TIMERINDEX_TIMER_D                /*!<PWM Index Channel NOT USED (Not used in DAB)*/
#define PWM_LL_CHANNEL_1                LL_HRTIM_OUTPUT_TA1                     /*!<PWM Channel HighVoltage SX High Side "M1"  (PowerBoard)*/
#define PWM_LL_CHANNEL_1N               LL_HRTIM_OUTPUT_TA2                     /*!<PWM Channel HighVoltage SX Low Side "M2"  (Power Board)*/
#define PWM_LL_CHANNEL_2                LL_HRTIM_OUTPUT_TB1                     /*!<PWM Channel HighVoltage DX High Side "M3"  (PowerBoard)*/
#define PWM_LL_CHANNEL_2N               LL_HRTIM_OUTPUT_TB2                     /*!<PWM Channel HighVoltage DX Low Side "M4"  (Power Board)*/
#define PWM_LL_CHANNEL_3                LL_HRTIM_OUTPUT_TC1                     /*!<PWM Channel LowVoltage SX High Side "M5"  (Power Board)*/
#define PWM_LL_CHANNEL_3N               LL_HRTIM_OUTPUT_TC2                     /*!<PWM Channel LowVoltage SX Low Side "M6"  (Power Board)*/
#define PWM_LL_CHANNEL_4                LL_HRTIM_OUTPUT_TD1                     /*!<PWM Channel LowVoltage DX High Side "M7"  (Power Board)*/
#define PWM_LL_CHANNEL_4N               LL_HRTIM_OUTPUT_TD2                     /*!<PWM Channel LowVoltage DX Low Side "M8"  (Power Board)*/
#define PWM_LL_CHANNEL_5                LL_HRTIM_OUTPUT_TE1                     /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_LL_CHANNEL_5N               LL_HRTIM_OUTPUT_TE2                     /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_LL_CHANNEL_6                LL_HRTIM_OUTPUT_TF1                     /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_LL_CHANNEL_6N               LL_HRTIM_OUTPUT_TF2                     /*!<PWM Channel NOT USED (Not used in DAB)*/
#define PWM_LL_IDX_CH_1                 LL_HRTIM_TIMER_A                        /*!<PWM Index Channel HighVoltage SX (Power Board)*/
#define PWM_LL_IDX_CH_2                 LL_HRTIM_TIMER_B                        /*!<PWM Index Channel HighVoltage DX (Power Board)*/
#define PWM_LL_IDX_CH_3                 LL_HRTIM_TIMER_C                        /*!<PWM Index Channel LowVoltage SX (Power Board)*/
#define PWM_LL_IDX_CH_4                 LL_HRTIM_TIMER_D                        /*!<PWM Index Channel LowVoltage DX (Power Board)*/
#define PWM_LL_IDX_CH_5                 LL_HRTIM_TIMER_E                        /*!<PWM Index Channel NOT USED (Not used in DAB)*/
#define PWM_LL_IDX_CH_6                 LL_HRTIM_TIMER_F                        /*!<PWM Index Channel NOT USED (Not used in DAB)*/
#define PWM_ID_CHANNEL_1                HRTIM_TIMERID_TIMER_A
#define PWM_ID_CHANNEL_2                HRTIM_TIMERID_TIMER_B
#define PWM_ID_CHANNEL_3                HRTIM_TIMERID_TIMER_C
#define PWM_ID_CHANNEL_4                HRTIM_TIMERID_TIMER_D
#endif                                                          //andif

#define APPL_Tim1                       htim2                                   /*!< Application TIMER - High Frequency Tasks*/
#define APPL_Tim2                       htim3                                   /*!< Application TIMER - Low Frequency Tasks*/
#define APPL_Tim3                       htim6                                   /*!< Application TIMER - Very Low Frequency Tasks*/
//#define APPL_Tim4                       htim1                                  /*!< Application TIMER - BICOLOR LED managment*/
#define APPL_Tim5                       htim7                                   /*!< Application TIMER - Very Low Frequency Tasks*/

///DPC LED Define of STDESVIENNARECT_REV2
#define DPC_BLED_TIM                    htim1                                   /*!<*/
#define APPL_TimBLED                    htim1                                   /*!<*/
#define DPC_BLED_CH                     TIM_CHANNEL_3                           /*!<*/
#define TIM_BLED_CHANNEL_1              TIM_CHANNEL_3                           /*!<*/

#define USE_RELAY_FAN                                                           /*!<*/
#define RELAY_FAN_PORT                  FAN_GPIO_Port                           /*!<*/
#define RELAY_FAN_PIN                   FAN_Pin                                 /*!<*/

///DPC TIMEOUTs Define of STDESPFCBIDIR
#define DPC_TO_INRUSH                   DPC_TO_INDX1                                /*!< */
#define DPC_TO_OTHER                    DPC_TO_INDX2                                /*!< */
#define DPC_TO_IDLE                     DPC_TO_INDX3                                /*!< Timeout Index - TimeOut_IDLE [IDX]*/
#define DPC_TO_INIT                     DPC_TO_INDX4                                /*!< Timeout Index - TimeOut_INIT [IDX]*/
#define DPC_TO_START                    DPC_TO_INDX5                                /*!< Timeout Index - TimeOut_INIT [IDX]*/

///DPC ADCs Define of STDESPFCBIDIR
#define ACQ_ADC_1                       hadc1                                   /*!< */
#define ACQ_ADC_2                       hadc2                                   /*!< */
#define ADC1_CHs                        1                                       /*!< */
#define ADC2_CHs                        5                                       /*!< */

#define ADC_DMA_RANK_VDAB1              3                                       /*!<DAB 1 Voltage DC Sensing*/
#define ADC_DMA_RANK_VDAB2              2                                       /*!<DAB 2 Voltage DC Sensing*/
#define ADC_DMA_RANK_IDAB1              1                                       /*!<DAB 1 Current DC Sensing*/
#define ADC_DMA_RANK_IDAB2              0                                       /*!<DAB 2 Current DC Sensing*/
#define ADC_DMA_RANK_ILK                0                                       /*!<Leakage Current AC Sensing */
//#define ADC_DMA_RANK_UCTEMP             5                                       /*!<MCU Temperature ADC Sensing */
#define ADC_DMA_RANK_EXTTEMP            4                                       /*!<EXT Temperature ADC Sensing */

#define ADC_DMA_IDX_VDAB1               p_ADC2_Data                             /*!<DAB 1 Voltage DC Sensing*/
#define ADC_DMA_IDX_VDAB2               p_ADC2_Data                             /*!<DAB 2 Voltage DC Sensing*/
#define ADC_DMA_IDX_IDAB1               p_ADC2_Data                             /*!<DAB 1 Current DC Sensing*/
#define ADC_DMA_IDX_IDAB2               p_ADC2_Data                             /*!<DAB 2 Current DC Sensing*/
#define ADC_DMA_IDX_ILK                 p_ADC1_Data                             /*!<Leakage Current AC Sensing */
//#define ADC_DMA_IDX_UCTEMP              p_ADC1_Data                             /*!<MCU Temperature ADC Sensing */
#define ADC_DMA_IDX_EXTTEMP             p_ADC1_Data                             /*!<EXT Temperature ADC Sensing */

///__End_________________________________________________________________STDES_DCDCDAB_______________________________________________________________________

#else
SELECT DEFINE
#endif

/* Exported functions ------------------------------------------------------- */

#endif //__DPC_LIB_CONF_H
