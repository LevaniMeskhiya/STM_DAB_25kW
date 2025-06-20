/**
  ******************************************************************************
  * @file    DPC_ADC.h
  * @brief   This file contains the headers of the transform module.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
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
#ifndef __ADC_CONVERTER_H
#define __ADC_CONVERTER_H

/* Includes ------------------------------------------------------------------*/
#include "DPC_CommonData.h"
/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Digital Power Converter - ADC handler
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void ADC2Phy_DAB_ProcessData(DPC_ADC_Conf_t *DPC_ADC_Conf,uint32_t* p_Data_Sub, DAB_ADC_PHY_Struct_t* DAB_ADC_DC_PHY_Sub);
void ADC2RAW_DAB_ProcessData(uint32_t* p_Data_Sub, DAB_ADC_RAW_Struct_t* DAB_ADC_DC_RAW_Sub);
void DPC_DAB_ADC_Init(DPC_ADC_Conf_t *DPC_ADC_Conf,float fGVac,float fBVac,float fGIac,float fBIac,float fGVdc,float fBVdc,float fGIdc,float fBIdc);
void DPC_ADC_AWDInit(void);
int16_t DPC_ADC_LowPassFilter(int16_t hNoFilteredValue, int32_t *phLastFilteredValue, uint8_t bDigitShift);
void DPC_ADC_TrigSet();
#endif