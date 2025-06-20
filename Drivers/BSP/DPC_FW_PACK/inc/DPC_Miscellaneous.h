/**
  ******************************************************************************
  * @file    : DPC_Miscellaneous.h
  * @brief   : Miscellaneous fuction management
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
#ifndef __DPC_MISCELLANEOUS_H
#define __DPC_MISCELLANEOUS_H


/* Includes ------------------------------------------------------------------*/
#include "DPC_Lib_Conf.h"
#include "DPC_CommonData.h"
#include "DPC_ADC.h"


/* Exported types ------------------------------------------------------------*/


/** 
 *@brief AC Source: Variables & Limit
 */
typedef struct {
uint16_t V_ac_pos_Limit;                /*!< */
uint16_t V_ac_neg_Limit;                /*!< */
uint16_t V_ac_pos_UVLO_Limit;           /*!< */
uint16_t V_ac_neg_UVLO_Limit;           /*!< */
uint16_t V_ac_pos_UV_Limit;             /*!< */
uint16_t V_ac_neg_UV_Limit;             /*!< */
uint16_t V_ac_pos_Low_Limit;            /*!< */
uint16_t V_ac_neg_Low_Limit;            /*!< */
uint16_t I_ac_pos_Limit;                /*!< */
uint16_t I_ac_neg_Limit;                /*!< */
}DPC_Source_Limit_TypeDef;








/* Exported constants --------------------------------------------------------*/

#define  RELAY_OFF GPIO_PIN_RESET
#define  RELAY_ON  GPIO_PIN_SET
#define  DPC_Relay_State_TypeDef GPIO_PinState

/* Exported macro ------------------------------------------------------------*/

#define DPC_LED1_ON    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);             /*!< SET ON DPC LED1 */
#define DPC_LED1_OFF   HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);           /*!< SET OFF DPC LED1*/
#define DPC_LED2_ON    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);             /*!< SET ON DPC LED2 */
#define DPC_LED2_OFF   HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);           /*!< SET OFF DPC LED2*/
#define DPC_GPIO1_ON   HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, GPIO_PIN_SET);           /*!< SET ON DPC LED1 */
#define DPC_GPIO1_OFF  HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, GPIO_PIN_RESET);         /*!< SET OFF DPC LED1*/
#define DPC_GPIO2_ON   HAL_GPIO_WritePin(GPIO_2_GPIO_Port, GPIO_2_Pin, GPIO_PIN_SET);           /*!< SET ON DPC LED2 */
#define DPC_GPIO2_OFF  HAL_GPIO_WritePin(GPIO_2_GPIO_Port, GPIO_2_Pin, GPIO_PIN_RESET);         /*!< SET OFF DPC LED2*/


/* Exported functions ------------------------------------------------------- */
DPC_MISC_DCLoadStatus_t DPC_MISC_DCLoadCheck(DPC_MISC_DCLoad_t *DPC_Load_loc,DPC_MISC_DCLoadLimit_t DC_Load_Limit_sub);
DPC_MISC_DCSourceStatus_t DPC_MISC_DAB_HVDCSource_Check(DPC_MISC_DCSource_t  *tDC_Source,DPC_MISC_DCSourceLimit_t tDC_SourceLimit,DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw);
DPC_MISC_DCLoadStatus_t DPC_MISC_DABLoadCheck(DPC_MISC_DCLoad_t *DPC_Load_loc,DPC_MISC_DCLoadLimit_t DC_Load_Limit_sub);
void DPC_MISC_DCSource_Init(DPC_MISC_DCSourceLimit_t *tDC_SourceLimit,uint16_t VDCsrc_OVP_V,uint16_t VDCsrc_UV_V,uint16_t VDCsrc_UVLO_V,uint16_t VDCsrc_LOW_V,uint16_t IDCsrc_OCP_A,DPC_ADC_Conf_t *pDPC_ADC_Conf);
DPC_MISC_PlugDCSourceStatus_t DPC_MISC_DC_SOURCE_Plugged(DPC_MISC_DCSourceLimit_t tDC_SourceLimit);
DPC_MISC_PlugDCSourceStatus_t DPC_MISC_DAB_HVDC_SOURCE_Plugged(DPC_MISC_DCSourceLimit_t tDC_SourceLimit,DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw);
void DPC_MISC_DCLoad_Init(DPC_MISC_DCLoadLimit_t *DC_Load_Limit_sub,uint16_t V_dc_Limit_VOLT,uint16_t V_cap_Limit_VOLT,float I_dc_NO_LOAD_Limit_AMP,float I_dc_LOW_LOAD_Limit_AMP,float I_dc_OVER_LOAD_Limit_AMP,DPC_ADC_Conf_t *pDPC_ADC_Conf);
void DPC_MISC_DAB_DCLoad_Init(DPC_MISC_DCLoadLimit_t *DC_Load_Limit_sub,uint16_t V_dc_Limit_VOLT,float I_dc_NO_LOAD_Limit_AMP,float I_dc_LOW_LOAD_Limit_AMP,float I_dc_OVER_LOAD_Limit_AMP,DPC_ADC_Conf_t *pDPC_ADC_Conf);
void DPC_MISC_APPL_Timer_Init(TIM_HandleTypeDef AppTIM, uint32_t  APPL_Freq_Desidered);
void DPC_MISC_Appl_Timer_Start(void);
void DPC_MISC_BLED_Set(TIM_HandleTypeDef *htim_bled,uint32_t TIM_CHANNEL_BLED,DPC_BLED_TypeDef State_BLED);
void DPC_DAC_Init(DAC_Channel_STRUCT *DAC_CH_Sub, uint8_t DAC_CH1_INIT_loc, uint8_t DAC_CH2_INIT_loc, uint8_t DAC_CH3_INIT_loc, uint16_t DAC_G_CH1_Init_loc,uint16_t DAC_G_CH2_Init_loc , uint16_t DAC_G_CH3_Init_loc, uint16_t DAC_B_CH1_Init_loc,uint16_t DAC_B_CH2_Init_loc , uint16_t DAC_B_CH3_Init_loc);
void DPC_MISC_OB_Init(void);
#endif /* __DPC_MISCELLANEOUS_H */