/**
  ******************************************************************************
  * @file           : adc_convert.c
  * @brief          : manage the adc converter
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

#include "DPC_ADC.h"
#include "adc.h"
#include "hrtim.h"

/* Private variables ---------------------------------------------------------*/  
/* Private typedef -----------------------------------------------------------*/    
/* Private function prototypes -----------------------------------------------*/    
    

/**
  * @brief  ADC_DAB_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  DAB_ADC_RAW_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC2Phy_DAB_ProcessData(DPC_ADC_Conf_t *DPC_ADC_Conf,uint32_t* p_Data_Sub, DAB_ADC_PHY_Struct_t* DAB_ADC_DC_PHY_Sub){
  
  //HV side measurements Physical value scaling terms
  float fBVdc1=DPC_ADC_Conf->fBVac;
  float invG_Vdc1=DPC_ADC_Conf->fInvGVac;
  float fBIdc1=DPC_ADC_Conf->fBIac;
  float fInvGIdc1=DPC_ADC_Conf->fInvGIac;    
  //HV side measurements Physical value scaling terms
  float fBVdc2=DPC_ADC_Conf->fBVdc;
  float invG_Vdc2=DPC_ADC_Conf->fInvGVdc;
  float fBIdc2=DPC_ADC_Conf->fBIdc;
  float fInvGIdc2=DPC_ADC_Conf->fInvGIdc;
  //AV section measurements Physical value scaling terms    
  float fBILk=0;
  float fInvGILk=0;   
  
  

  DAB_ADC_DC_PHY_Sub->VDAB1=((float)((int16_t)p_Data_Sub[0]-fBVdc1)*(float)(invG_Vdc1));        //HV Voltage measurements Physical - value scaling
  DAB_ADC_DC_PHY_Sub->VDAB2=((float)((int16_t)p_Data_Sub[1]-fBVdc2)*(float)(invG_Vdc2));        //LV Voltage measurements Physical - value scaling
  DAB_ADC_DC_PHY_Sub->IDAB1=((float)((int16_t)p_Data_Sub[2]-fBIdc1)*(float)(fInvGIdc1));        //HV Current measurements Physical - value scaling 
  DAB_ADC_DC_PHY_Sub->IDAB2=((float)((int16_t)p_Data_Sub[3]-fBIdc2)*(float)(fInvGIdc2));        //LV Current measurements Physical - value scaling
  DAB_ADC_DC_PHY_Sub->ILK=((float)((int16_t)p_Data_Sub[4]-fBILk)*(float)(fInvGILk));            //Primary side Current measurements - Physical value scaling
}

/**
  * @brief  ADC_DAB_ProcessData
  * @param  p_Data_Sub ROW DATA
  * @param  DAB_ADC_RAW_Sub, cooked Data
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */     
void ADC2RAW_DAB_ProcessData(uint32_t* p_Data_Sub, DAB_ADC_RAW_Struct_t* DAB_ADC_DC_RAW_Sub){
  
  DAB_ADC_DC_RAW_Sub->VDAB1=(int16_t)p_Data_Sub[0];        //HV Voltage measurements - Raw acquisition
  DAB_ADC_DC_RAW_Sub->VDAB2=(int16_t)p_Data_Sub[1];        //LV Voltage measurements - Raw acquisition
  DAB_ADC_DC_RAW_Sub->IDAB1=(int16_t)p_Data_Sub[2];        //HV Current measurements - Raw acquisition
  DAB_ADC_DC_RAW_Sub->IDAB2=(int16_t)p_Data_Sub[3];        //LV Current measurements - Raw acquisition
  DAB_ADC_DC_RAW_Sub->ILK=(int16_t)p_Data_Sub[4];          //Primary side Current measurements - Raw acquisition
}




/**
  * @brief  DPC_DAB_ADC_Init - Configure the  DPC_ADC_Conf_t struct according with the sensing parameters
  * @param  DPC_ADC_Conf_t
  * @param  fGVac,fBVac,fGIac,fBIac,fGVdc,fBVdc,fGIdc,fBIdc - Gain & Bias of the sensing
  * @retval None
  *
  * @note Function valid for STM32G4xx microconroller family  
  */    

void DPC_DAB_ADC_Init(DPC_ADC_Conf_t *DPC_ADC_Conf,float fGVdcHV,float fBVdcHV,float fGIdcHV,float fBIdcHV,float fGVdcLV,float fBVdcLV,float fGIdcLV,float fBIdcLV){
  

/* DC High Voltage Sensing */  
DPC_ADC_Conf->fBVac=fBVdcHV;
DPC_ADC_Conf->fGVac=fGVdcHV;
DPC_ADC_Conf->fInvGVac=(float)(1.0f/fGVdcHV);

/* DC Low Voltage Sensing */
DPC_ADC_Conf->fBVdc=fBVdcLV;
DPC_ADC_Conf->fGVdc=fGVdcLV;
DPC_ADC_Conf->fInvGVdc=(float)(1.0f/fGVdcLV);

/* DC HV Current Sensing */
DPC_ADC_Conf->fBIac=fBIdcHV;
DPC_ADC_Conf->fGIac=fGIdcHV;
DPC_ADC_Conf->fInvGIac=(float)(1.0f/fGIdcHV);

/* DC LV Current Sensing */
DPC_ADC_Conf->fBIdc=fBIdcLV;
DPC_ADC_Conf->fGIdc=fGIdcLV;
DPC_ADC_Conf->fInvGIdc=(float)(1.0f/fGIdcLV);


DPC_ADC_Conf->DPC_ADC_Conf_Complete=SET;                /*! Set when configuration is completed */
}


/**
  * @brief  DPC_ADC_TrigSet - 
  * @param  TBD
  * @param  TBD
  * @retval TBD
  *
  * @note Function valid for STM32G4xx microconroller family  
  */    

void DPC_ADC_TrigSet(uint32_t TriggerTimeEvent){
  
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER,ADC_TRG_CU, (uint32_t)(TriggerTimeEvent));          
  
}
                                                                                                                   
