/**
  ******************************************************************************
  * @file           : Data.c
  * @brief          : Data Collectiona and access interfaces
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
/* Includes ------------------------------------------------------------------*/
#include "DPC_CommonData.h"
#include "adc.h"
    

/* Private variables ---------------------------------------------------------*/
float DATA_theta_PLL;
DPC_CDT_VOLT_DC_ADC_t VDC_ADC;
DPC_CDT_CURR_DC_ADC_t IDC_ADC;
DAB_ADC_Value_Struct DAB_ADC;
RAW_ADC_UNION RAW_ADC;
//External temoerature
uint16_t T_ext;
uint16_t T_int;

 
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Init the ADC DMA 
  * @param  None
  *         
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void InitDMA_ADC_CONV(void){
HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&RAW_ADC,9);  //HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* p_ADC1_Data, uint32_t Length)
}



/**
  * @brief  Read the ADC data raw 
  * @param  None
  *         
  * @retval RAW_ADC_UNION, pointer to Data struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
RAW_ADC_UNION* DATA_Read_ADC_Raw(void){
  
  return &RAW_ADC; 
  
}




/**
* @brief  Acquisition Data From a ADC with DMA.
* @param  p_ADC1_Data.
*         Pointer to ADC Data struct:
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data,uint32_t* p_ADC2_Data)                            
{   
  DAB_ADC.VDAB1=ADC_DMA_IDX_VDAB1[ADC_DMA_RANK_VDAB1];             ///
  DAB_ADC.VDAB2=ADC_DMA_IDX_VDAB2[ADC_DMA_RANK_VDAB2];             ///
  DAB_ADC.IDAB1=ADC_DMA_IDX_IDAB1[ADC_DMA_RANK_IDAB1];             ///
  DAB_ADC.IDAB2=ADC_DMA_IDX_IDAB2[ADC_DMA_RANK_IDAB2];             ///
  DAB_ADC.ILK=ADC_DMA_IDX_ILK[ADC_DMA_RANK_ILK];                 ///
  T_ext=ADC_DMA_IDX_EXTTEMP[ADC_DMA_RANK_EXTTEMP];              ///EXT Temperature ADC Sensing   
} 



/**
  * @brief  Read the DC Current.
  * @param  None.
  * 
  * @retval IAC_ADC, pointer to CurrentDC_ADC struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
DPC_CDT_CURR_DC_ADC_t* Read_Curr_DC(void){
  return &IDC_ADC;
}

/**
  * @brief  Read the DC Voltage.
  * @param  None.
  * 
  * @retval VDC_ADC, pointer to VoltageDC_ADC struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */
DPC_CDT_VOLT_DC_ADC_t* Read_Volt_DC(void){
  return &VDC_ADC;
}



/**
  * @brief  Read the DAB variables.
  * @param  None.
  * 
  * @retval DAB_ADC, pointer to DAB_ADC_Value_Struct
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
DAB_ADC_Value_Struct* Read_DAB(void){
  return &DAB_ADC;
}