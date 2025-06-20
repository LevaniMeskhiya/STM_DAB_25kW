/**
******************************************************************************
* @file           : DPC_Miscellaneous.c
* @brief          : Miscellaneous fuction management
******************************************************************************
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether 
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
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
/* Includes ------------------------------------------------------------------*/

#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

/* Private includes ---------------------------------------------------------*/
#include "tim.h"
#include "stm32g4xx_ll_tim.h"
#include "gpio.h"

#include "DPC_Miscellaneous.h"
#include "DPC_Faulterror.h"



/* external variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @brief  DPC_MISC_DCLoadCheck: Check Load in DC Output systems 
*         
* @param p_ADC_Data_Sub: pointer to read output DC current value  
* @param tDC_LoadLimit: Converter limit data structure  
*
* @retval DPC_MISC_DCLoadStatus_t: Load status, NO_LOAD, LOW_LOAD, ON_LOAD, 
*             OVERVOLTAGE_LOAD, OVERCURRENT_LOAD.
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/

DPC_MISC_DCLoadStatus_t DPC_MISC_DCLoadCheck(DPC_MISC_DCLoad_t *DPC_Load_loc,DPC_MISC_DCLoadLimit_t DC_Load_Limit_sub)
{

  uint16_t VDC;
  DPC_CDT_VOLT_DC_ADC_t* DATA_VDC;  
  DPC_CDT_CURR_DC_ADC_t* DATA_IDC;
  
  DATA_VDC =  Read_Volt_DC(); 
  DATA_IDC = Read_Curr_DC(); 
  
  VDC=DATA_VDC->Vdc_pos+DATA_VDC->Vdc_neg;
  
  DPC_MISC_DCLoadStatus_t Load_Status;
 
  
  if((DATA_VDC->Vdc_pos > DC_Load_Limit_sub.V_cap_Limit) || (DATA_VDC->Vdc_neg > DC_Load_Limit_sub.V_cap_Limit)){  
    DPC_ACT_OutDisable();                                                                   ///Safe: Disable PWM outputs if enabled
    Load_Status=OVERVOLTAGE_CAP;
    DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OVC);    
  }
  else if (VDC>DC_Load_Limit_sub.V_dc_Limit){
    DPC_ACT_OutDisable();                                                                   ///Safe: Disable PWM outputs if enabled
    Load_Status=OVERVOLTAGE_LOAD;
    DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OVL);
  }
  else {
    if(DATA_IDC->IDC_adc>=DC_Load_Limit_sub.I_Over_load_Threshold)
    {
      Load_Status=OVERCURRENT_LOAD;
      DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OCL);       
    }
    else if(DATA_IDC->IDC_adc>=DC_Load_Limit_sub.I_Low_load_Max_Threshold && (DPC_Load_loc->DPC_Load_Status==LOW_LOAD || DPC_Load_loc->DPC_Load_Status==NO_LOAD))
    {
      Load_Status=ON_LOAD;
    }
    else if(DATA_IDC->IDC_adc<=DC_Load_Limit_sub.I_Low_load_Min_Threshold && (DPC_Load_loc->DPC_Load_Status==ON_LOAD))
    {
      Load_Status=LOW_LOAD;
    }
    else if(DATA_IDC->IDC_adc>=DC_Load_Limit_sub.I_No_load_Max_Threshold && (DPC_Load_loc->DPC_Load_Status==NO_LOAD))
    {
      Load_Status=LOW_LOAD;
    }
    else if(DATA_IDC->IDC_adc<=DC_Load_Limit_sub.I_No_load_Min_Threshold)
    {
      Load_Status=NO_LOAD;
    }
    else
    {
      Load_Status=DPC_Load_loc->DPC_Load_Status;
    }
  }
  
  DPC_Load_loc->DPC_Load_Status=Load_Status;
  return Load_Status;
}



/* external variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @brief  DPC_MISC_DCLoadCheck: Check Load in DC Autput sistems 
*         
* @param p_ADC_Data_Sub: pointer to readed output DC current value  
* @param DC_Load_Limit_sub: Converter limit data structure  
*
* @retval DPC_MISC_DCLoadStatus_t: Load status, NO_LOAD, LOW_LOAD, ON_LOAD, 
*             OVERVOLTAGE_LOAD, OVERCURRENT_LOAD.
*
* @note Function valid for STM32G4xx and STM32F74x microconroller family   
*/

DPC_MISC_DCLoadStatus_t DPC_MISC_DABLoadCheck(DPC_MISC_DCLoad_t *DPC_Load_loc,DPC_MISC_DCLoadLimit_t DC_Load_Limit_sub)
{

  uint16_t VDC_Load;
  uint16_t IDC_Load;
  DAB_ADC_Value_Struct* DATA_DAB;  
  
  DATA_DAB =  Read_DAB(); 
  
  VDC_Load=DATA_DAB->VDAB2;
  IDC_Load=DATA_DAB->IDAB2;
  
  DPC_MISC_DCLoadStatus_t Load_Status;
 
  
  if (VDC_Load>DC_Load_Limit_sub.V_dc_Limit){
    DPC_ACT_OutDisable();                                                                   ///Safe: Disable PWM outputs if enabled
    Load_Status=OVERVOLTAGE_LOAD;
    DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OVL);
  }
  else {
    if(IDC_Load>=DC_Load_Limit_sub.I_Over_load_Threshold)
    {
      Load_Status=OVERCURRENT_LOAD;
      DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OCL);       
    }
    else if(IDC_Load>=DC_Load_Limit_sub.I_Low_load_Max_Threshold && (DPC_Load_loc->DPC_Load_Status==LOW_LOAD || DPC_Load_loc->DPC_Load_Status==NO_LOAD))
    {
      Load_Status=ON_LOAD;
    }
    else if(IDC_Load<=DC_Load_Limit_sub.I_Low_load_Min_Threshold && (DPC_Load_loc->DPC_Load_Status==ON_LOAD))
    {
      Load_Status=LOW_LOAD;
    }
    else if(IDC_Load>=DC_Load_Limit_sub.I_No_load_Max_Threshold && (DPC_Load_loc->DPC_Load_Status==NO_LOAD))
    {
      Load_Status=LOW_LOAD;
    }
    else if(IDC_Load<=DC_Load_Limit_sub.I_No_load_Min_Threshold)
    {
      Load_Status=NO_LOAD;
    }
    else
    {
      Load_Status=DPC_Load_loc->DPC_Load_Status;
    }
  }
  
  DPC_Load_loc->DPC_Load_Status=Load_Status;
  return Load_Status;
}



/**
* @brief  DPC_MISC_DCLoad_Init: Check Load in DC output systems 
*         
* @param V_CapLimit_V DC voltage limit expressed in Peak voltage
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/

void DPC_MISC_DCLoad_Init(DPC_MISC_DCLoadLimit_t *DC_Load_Limit_sub,uint16_t V_dc_Limit_VOLT,uint16_t V_cap_Limit_VOLT,float I_dc_NO_LOAD_Limit_AMP,float I_dc_LOW_LOAD_Limit_AMP,float I_dc_OVER_LOAD_Limit_AMP,DPC_ADC_Conf_t *pDPC_ADC_Conf)
{
  
 
  uint16_t V_cap_Limit_loc;                                                                             /// Local variable to pass Output capacitors voltage limit theshold (Expressed in VOLT) 
  uint16_t V_dc_Limit_loc;                                                                              /// Local variable to pass Output voltage limit theshold (Expressed in VOLT)   
  uint16_t I_dc_NoLoad_Limit;                                                                      /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_NO_LOAD_Delta_Limit_loc;                                                                /// 
  uint16_t I_dc_NO_LOAD_Max_Limit_loc;                                                                  /// Local variable to pass Output current Max theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_NO_LOAD_Min_Limit_loc;                                                                  /// Local variable to pass Output current Min theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_LOW_LOAD_Limit_loc;                                                                     /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_LOW_LOAD_Delta_Limit_loc;                                                                /// 
  uint16_t I_dc_LOW_LOAD_Max_Limit_loc;                                                                 /// Local variable to pass Output current Max theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_LOW_LOAD_Min_Limit_loc;                                                                 /// Local variable to pass Output current Min theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_OVER_LOAD_Limit_loc;                                                                    /// Local variable to pass Output current theshold (Expressed in AMPs)to determinate Over Load Condition
  

  
  V_cap_Limit_loc=(uint16_t)(((float)V_cap_Limit_VOLT*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);                                        /// (Vcap_limit_Threshold [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias 
  V_dc_Limit_loc=(uint16_t)(((float)V_dc_Limit_VOLT*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);                                          /// (Vdc_limit_Threshold [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias

  I_dc_NoLoad_Limit=(uint16_t)(((float)I_dc_NO_LOAD_Limit_AMP*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);                           /// (IDC_No_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_NO_LOAD_Delta_Limit_loc=(uint16_t)((I_dc_NoLoad_Limit - pDPC_ADC_Conf->fBIdc)*((float)DPC_NO_LOAD_DELTA_CURR*0.01f));                     ///
  I_dc_NO_LOAD_Max_Limit_loc=(uint16_t)(I_dc_NoLoad_Limit + I_dc_NO_LOAD_Delta_Limit_loc);                                                 /// 
  I_dc_NO_LOAD_Min_Limit_loc=(uint16_t)(I_dc_NoLoad_Limit - I_dc_NO_LOAD_Delta_Limit_loc);                                                 /// 

  I_dc_LOW_LOAD_Limit_loc=(uint16_t)(((float)I_dc_LOW_LOAD_Limit_AMP*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);                         /// (IDC_Light_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_LOW_LOAD_Delta_Limit_loc=(uint16_t)((I_dc_LOW_LOAD_Limit_loc - pDPC_ADC_Conf->fBIdc)*((float)DPC_LOW_LOAD_DELTA_CURR*0.01f));                   ///
  I_dc_LOW_LOAD_Max_Limit_loc=(uint16_t)(I_dc_LOW_LOAD_Limit_loc + I_dc_LOW_LOAD_Delta_Limit_loc);                                               /// 
  I_dc_LOW_LOAD_Min_Limit_loc=(uint16_t)(I_dc_LOW_LOAD_Limit_loc - I_dc_LOW_LOAD_Delta_Limit_loc);                                               /// 

  I_dc_OVER_LOAD_Limit_loc=(uint16_t)(((float)I_dc_OVER_LOAD_Limit_AMP*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);                       /// (IDC_Over_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias   
  
  DC_Load_Limit_sub->V_cap_Limit=V_cap_Limit_loc;  
  DC_Load_Limit_sub->V_dc_Limit=V_dc_Limit_loc;  
  DC_Load_Limit_sub->I_No_load_Threshold=I_dc_NoLoad_Limit;
  DC_Load_Limit_sub->I_No_load_Max_Threshold=I_dc_NO_LOAD_Max_Limit_loc;
  DC_Load_Limit_sub->I_No_load_Min_Threshold=I_dc_NO_LOAD_Min_Limit_loc;  
  DC_Load_Limit_sub->I_Low_load_Threshold=I_dc_LOW_LOAD_Limit_loc;
  DC_Load_Limit_sub->I_Low_load_Max_Threshold=I_dc_LOW_LOAD_Max_Limit_loc;
  DC_Load_Limit_sub->I_Low_load_Min_Threshold=I_dc_LOW_LOAD_Min_Limit_loc;
  DC_Load_Limit_sub->I_Over_load_Threshold=I_dc_OVER_LOAD_Limit_loc;
  
  
}


/**
* @brief  DPC_MISC_DAB_DCLoad_Init: Check Load in DC output systems 
*         
* @param V_CapLimit_V DC voltage limit expressed in Peak voltage
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/

void DPC_MISC_DAB_DCLoad_Init(DPC_MISC_DCLoadLimit_t *DC_Load_Limit_sub,uint16_t V_dc_Limit_VOLT,float I_dc_NO_LOAD_Limit_AMP,float I_dc_LOW_LOAD_Limit_AMP,float I_dc_OVER_LOAD_Limit_AMP,DPC_ADC_Conf_t *pDPC_ADC_Conf)
{
  
 
  uint16_t V_dc_Limit_loc;                                                                              /// Local variable to pass Output voltage limit theshold (Expressed in VOLT)   
  uint16_t I_dc_NoLoad_Limit;                                                                      /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_NO_LOAD_Delta_Limit_loc;                                                                /// 
  uint16_t I_dc_NO_LOAD_Max_Limit_loc;                                                                  /// Local variable to pass Output current Max theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_NO_LOAD_Min_Limit_loc;                                                                  /// Local variable to pass Output current Min theshold (Expressed in AMPs) to determinate No Load Condition  
  uint16_t I_dc_LOW_LOAD_Limit_loc;                                                                     /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_LOW_LOAD_Delta_Limit_loc;                                                                /// 
  uint16_t I_dc_LOW_LOAD_Max_Limit_loc;                                                                 /// Local variable to pass Output current Max theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_LOW_LOAD_Min_Limit_loc;                                                                 /// Local variable to pass Output current Min theshold (Expressed in AMPs) to determinate Light Load Condition  
  uint16_t I_dc_OVER_LOAD_Limit_loc;                                                                    /// Local variable to pass Output current theshold (Expressed in AMPs)to determinate Over Load Condition
  

  
  V_dc_Limit_loc=(uint16_t)(((float)V_dc_Limit_VOLT*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);                                          /// (Vdc_limit_Threshold [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias

  I_dc_NoLoad_Limit=(uint16_t)(((float)I_dc_NO_LOAD_Limit_AMP*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);                           /// (IDC_No_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_NO_LOAD_Delta_Limit_loc=(uint16_t)((I_dc_NoLoad_Limit - pDPC_ADC_Conf->fBIdc)*((float)DPC_NO_LOAD_DELTA_CURR*0.01f));                     ///
  I_dc_NO_LOAD_Max_Limit_loc=(uint16_t)(I_dc_NoLoad_Limit + I_dc_NO_LOAD_Delta_Limit_loc);                                                 /// 
  I_dc_NO_LOAD_Min_Limit_loc=(uint16_t)(I_dc_NoLoad_Limit - I_dc_NO_LOAD_Delta_Limit_loc);                                                 /// 

  I_dc_LOW_LOAD_Limit_loc=(uint16_t)(((float)I_dc_LOW_LOAD_Limit_AMP*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);                         /// (IDC_Light_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  I_dc_LOW_LOAD_Delta_Limit_loc=(uint16_t)((I_dc_LOW_LOAD_Limit_loc - pDPC_ADC_Conf->fBIdc)*((float)DPC_LOW_LOAD_DELTA_CURR*0.01f));                   ///
  I_dc_LOW_LOAD_Max_Limit_loc=(uint16_t)(I_dc_LOW_LOAD_Limit_loc + I_dc_LOW_LOAD_Delta_Limit_loc);                                               /// 
  I_dc_LOW_LOAD_Min_Limit_loc=(uint16_t)(I_dc_LOW_LOAD_Limit_loc - I_dc_LOW_LOAD_Delta_Limit_loc);                                               /// 

  I_dc_OVER_LOAD_Limit_loc=(uint16_t)(((float)I_dc_OVER_LOAD_Limit_AMP*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);                       /// (IDC_Over_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias   
  
  DC_Load_Limit_sub->V_dc_Limit=V_dc_Limit_loc;  
  DC_Load_Limit_sub->I_No_load_Threshold=I_dc_NoLoad_Limit;
  DC_Load_Limit_sub->I_No_load_Max_Threshold=I_dc_NO_LOAD_Max_Limit_loc;
  DC_Load_Limit_sub->I_No_load_Min_Threshold=I_dc_NO_LOAD_Min_Limit_loc;  
  DC_Load_Limit_sub->I_Low_load_Threshold=I_dc_LOW_LOAD_Limit_loc;
  DC_Load_Limit_sub->I_Low_load_Max_Threshold=I_dc_LOW_LOAD_Max_Limit_loc;
  DC_Load_Limit_sub->I_Low_load_Min_Threshold=I_dc_LOW_LOAD_Min_Limit_loc;
  DC_Load_Limit_sub->I_Over_load_Threshold=I_dc_OVER_LOAD_Limit_loc;
  
  
}




/**
* @brief  DPC_MISC_DCSource_Init: This function is used to define DC source operation thresholds
*         
* @param VDCsrc_OVP_V - OverVoltageProtection "OVP" DC voltage limit expressed in volt
* @param VDCsrc_UV_V - UnderVoltage "UV" DC voltage limit expressed in volt (Recoverable)
* @param VDCsrc_UVLO_V - UnderVoltageLockOut "UVLO" DC voltage limit expressed in volt (UnRecoverable)
* @param VDCsrc_LOW_V - Min Voltage DC voltage limit expressed in volt (Start-up)
* @param IDCsrc_OCP_A - Over Current Protection "OCP" DC current limit expressed amp
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/

void DPC_MISC_DCSource_Init(DPC_MISC_DCSourceLimit_t *tDC_SourceLimit,uint16_t VDCsrc_OVP_V,uint16_t VDCsrc_UV_V,uint16_t VDCsrc_UVLO_V,uint16_t VDCsrc_LOW_V,uint16_t IDCsrc_OCP_A,DPC_ADC_Conf_t *pDPC_ADC_Conf)
{
 
  uint16_t V_dc_Limit_loc;                                                                                                          /*!< Minimum DC Voltage FSM start-up - DC/AC operation [Expressed in Bits]*/
  uint16_t V_dc_UV_Limit_loc;                                                                                                       /*!< >*/
  uint16_t V_dc_UVLO_Limit_loc;                                                                                                     /*!< UnderVoltage Lock-out protection for DC/AC operation [Expressed in Bits]>*/ 
  uint16_t V_dc_Low_Limit_loc;                                                                                                      /*!< >*/ 
  uint16_t I_dc_Limit_loc;                                                                                                          /*!< >*/
   
  V_dc_Limit_loc=(uint16_t)(((float)VDCsrc_OVP_V*pDPC_ADC_Conf->fGVac)+pDPC_ADC_Conf->fBVac);                                       /*!< >*/
  V_dc_UV_Limit_loc=(uint16_t)(((float)VDCsrc_UV_V*pDPC_ADC_Conf->fGVac)+pDPC_ADC_Conf->fBVac);                                     /*!< >*/
  V_dc_UVLO_Limit_loc=(uint16_t)(((float)VDCsrc_UVLO_V*pDPC_ADC_Conf->fGVac)+pDPC_ADC_Conf->fBVac);                                 /*!< >*/
  V_dc_Low_Limit_loc=(uint16_t)(((float)VDCsrc_LOW_V*pDPC_ADC_Conf->fGVac)+pDPC_ADC_Conf->fBVac);                                   /*!< >*/
  I_dc_Limit_loc=(uint16_t)(((float)IDCsrc_OCP_A*pDPC_ADC_Conf->fGIac)+pDPC_ADC_Conf->fBIac);                                       /*!< >*/
  
  
  tDC_SourceLimit->V_dc_pos_Limit=V_dc_Limit_loc;                                                                                   /*!< >*/
//  tDC_SourceLimit->V_dc_neg_Limit=V_dc_neg_Limit_loc;                                                                               /*!< >*/
  tDC_SourceLimit->V_dc_pos_UV_Limit=V_dc_UV_Limit_loc;                                                                             /*!< >*/
//  tDC_SourceLimit->V_dc_neg_UV_Limit=V_dc_neg_UV_Limit_loc;                                                                         /*!< >*/  
  tDC_SourceLimit->V_dc_pos_UVLO_Limit=V_dc_UVLO_Limit_loc;                                                                         /*!< >*/
//  tDC_SourceLimit->V_dc_neg_UVLO_Limit=V_dc_neg_UVLO_Limit_loc;                                                                     /*!< >*/    
  tDC_SourceLimit->V_dc_pos_Low_Limit=V_dc_Low_Limit_loc;                                                                           /*!< >*/
//  tDC_SourceLimit->V_dc_neg_Low_Limit=V_dc_neg_Low_Limit_loc;                                                                       /*!< >*/
  tDC_SourceLimit->I_dc_pos_Limit=I_dc_Limit_loc;
//  tDC_SourceLimit->I_dc_neg_Limit=I_dc_neg_Limit_loc;
  
}

 




/**
* @brief  DPC_MISC_APPL_Timer_Init: Initialize the Application timer to the desidered period
*         
* @param AppTIM: Application Timer to initialize.
* @param APPL_Freq_Desidered: Value of frequency desidered [Expressed in Hz]
* @retval None
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/
void DPC_MISC_APPL_Timer_Init(TIM_HandleTypeDef AppTIM, uint32_t  APPL_Freq_Desidered)
{
  uint32_t Timers_Clock;                                                                /*!< Local variable used to contain Appl Timer Clock*/
  uint32_t Timers_PSC;                                                                  /*!< Local variable used to contain Appl Timer Prescaler*/
  uint32_t Timers_ClockPSCed;                                                           /*!< Local variable used to contain Appl Timer Precaled Clock*/
  uint32_t Timers_ClockARR;                                                             /*!< Local variable used to contain Appl Timer AutoReloadRegister Value*/
  
  Timers_PSC=LL_TIM_GetPrescaler(AppTIM.Instance);                                      /// Get Appl Tim Prescaler
  Timers_Clock=HAL_RCC_GetPCLK2Freq();                                                  /// Get Appl Tim Clock  
  
  Timers_ClockPSCed=(Timers_Clock/(Timers_PSC+1));                                      /// Determines the Prescaled Clock    
  Timers_ClockARR = ((Timers_ClockPSCed/APPL_Freq_Desidered) - 1);                      /// Set Timers_ClockARR in ARR Register of Appl Timer
  
  AppTIM.Init.Period = Timers_ClockARR;                                                 ///
  if (HAL_TIM_Base_Init(&AppTIM) != HAL_OK){Error_Handler();}

//  LL_TIM_SetAutoReload(AppTIM.Instance,Timers_ClockARR);                                /// Set Timers_ClockARR in ARR Register of Appl Timer

} 




/**
  * @brief  DPC_MISC_ApplTimer_Start: Start the application timers and the BLED timer if used
  * @param  None.
  * @retval None. 
  *
  * @note Function valid for STM32G4xx microcontroller family  
  */
void DPC_MISC_Appl_Timer_Start(void)
{
#ifdef APPL_Tim1
  HAL_TIM_Base_Start_IT(&APPL_Tim1);  //Start Appl Timer 1
#endif // APPL_Tim1
#ifdef APPL_Tim2
  HAL_TIM_Base_Start_IT(&APPL_Tim2);  //Start Appl Timer 2
#endif // APPL_Tim2
#ifdef APPL_Tim3
  HAL_TIM_Base_Start_IT(&APPL_Tim3);  //Start Appl Timer 3 
#endif // APPL_Tim3
#ifdef APPL_Tim4
  HAL_TIM_Base_Start_IT(&APPL_Tim4);  //Start Appl Timer 4 
#endif // APPL_Tim4  
#ifdef APPL_Tim5
  HAL_TIM_Base_Start_IT(&APPL_Tim5);  //Start Appl Timer 5 
#endif // APPL_Tim5    
#ifdef APPL_TimBLED    
  HAL_TIM_PWM_Start(&APPL_TimBLED, TIM_BLED_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&APPL_TimBLED, TIM_BLED_CHANNEL_1);
#endif // APPL_TimBLED  
    
}


/**
* @brief  DPC_MISC_BLED_Set: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/



void DPC_MISC_BLED_Set(TIM_HandleTypeDef *htim_bled,uint32_t TIM_CHANNEL_BLED,DPC_BLED_TypeDef State_BLED){
  switch ( State_BLED){
  case BLED_Idle:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_Idle);                // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    break;
  case BLED_StartUp_inrush:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_StartUp_inrush);      // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    break;
  case BLED_Fault:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_Fault);               // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);        
    break;
  case BLED_Error:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_Error);               // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);      
    break;
  case BLED_Run:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_Run);                 // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);         
    break; 
  case BLED_StartUp_burst: 
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_StartUp_burst);       // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);       
    break; 
  case BLED_Stop:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_Stop);                // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);             
    break;
  case BLED_Debug:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_Debug);               // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);             
    break; 
  case BLED_Wait:
    __HAL_TIM_SET_COMPARE(htim_bled, TIM_CHANNEL_BLED, BLED_Pulse_Wait);                // 0xB000 = Orange 0x0000 = Red  0xFFFF = Green//      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);             
    break;     
  }  
}



/**
* @brief  DPC_MISC_DAB_HVDC_SOURCE_Plugged: Check AC Source connection
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/

DPC_MISC_PlugDCSourceStatus_t DPC_MISC_DAB_HVDC_SOURCE_Plugged(DPC_MISC_DCSourceLimit_t tDC_SourceLimit,DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw){
  
  DPC_MISC_PlugDCSourceStatus_t Status_DAB_Plug_HVDCSource; 
  
  
  uint16_t V_dc_Plug_Limit_local;                                                                   /*!< >*/
  
  V_dc_Plug_Limit_local=tDC_SourceLimit.V_dc_pos_Low_Limit;                                         /*!< >*/
  
  
 if(DAB_ADC_DCraw->VDAB1 == 0x00){  
    Status_DAB_Plug_HVDCSource=WAIT_Plug_DCSource;
  }
  else{
    if(DAB_ADC_DCraw->VDAB1 < V_dc_Plug_Limit_local){ 
      Status_DAB_Plug_HVDCSource=NO_Plug_DCSource;
    }
    else
    {
      Status_DAB_Plug_HVDCSource=OK_Plug_DCSource;        
    }
  }
  return Status_DAB_Plug_HVDCSource;
}



/**
* @brief  DPC_MISC_DCSource_Check: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/

DPC_MISC_DCSourceStatus_t DPC_MISC_DCSource_Check(DPC_MISC_DCSource_t  *tDC_Source,DPC_MISC_DCSourceLimit_t tDC_SourceLimit){
  
  DPC_MISC_DCSourceStatus_t Status_DCSource = tDC_Source->Status_DCSource; 
  
  
  DPC_CDT_VOLT_DC_ADC_t* DATA_VDC;                                                                       /*!< >*/
  DPC_CDT_CURR_DC_ADC_t* DATA_IDC;                                                                       /*!< >*/
  
  uint16_t V_dc_Limit_local;                                                                             /*!< >*/
  uint16_t V_dc_UVLO_Limit_local;                                                                        /*!< UnderVoltage Lock-out protection for DC/AC operation [Expressed in Bits]*/
  uint16_t V_dc_UV_Limit_local;                                                                          /*!< UnderVoltage protection for DC/AC operation [Expressed in Bits]*/
  uint16_t V_dc_Low_Limit_local;                                                                         /*!< Minimum DC Voltage F - DC/AC operation [Expressed in Bits]*/ 
  uint16_t I_dc_Limit_local;                                                                             /*!< Overcurrent DC protection for DC/AC operation [Expressed in Bits]*/
  
  
  V_dc_Limit_local=tDC_SourceLimit.V_dc_pos_Limit;                                                       /*!< >*/
  V_dc_UVLO_Limit_local=tDC_SourceLimit.V_dc_pos_UVLO_Limit;                                             /*!< >*/
  V_dc_UV_Limit_local=tDC_SourceLimit.V_dc_pos_UV_Limit;                                                 /*!< >*/
  V_dc_Low_Limit_local=tDC_SourceLimit.V_dc_pos_Low_Limit;                                               /*!< >*/
  I_dc_Limit_local=tDC_SourceLimit.I_dc_pos_Limit;                                                       /*!< >*/
  
  
  DATA_VDC = Read_Volt_DC();                                                                    /*!< Read DC Voltage ADC raw data>*/
  DATA_IDC = Read_Curr_DC();                                                                    /*!< Read DC Current ADC raw data>*/
  
  
  if(DATA_IDC->IDC_adc > I_dc_Limit_local)
  {
    Status_DCSource=OVERCURRENT_DC_SOURCE;
    DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OCS);      
  }
  else
  {
    if((DATA_VDC->Vdc_pos + DATA_VDC->Vdc_neg) > V_dc_Limit_local)
    {  
      Status_DCSource=OVERVOLTAGE_DC_SOURCE;
      DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OVS);  
    }
    else{  
      
      if((DATA_VDC->Vdc_pos + DATA_VDC->Vdc_neg) == 0){ 
        Status_DCSource=WAIT_DC_SOURCE;
      }
      else{
        if((DATA_VDC->Vdc_pos + DATA_VDC->Vdc_neg) > V_dc_Limit_local){
          Status_DCSource=OVERVOLTAGE_DC_SOURCE;
          DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OVS);
        } 
        else if((DATA_VDC->Vdc_pos + DATA_VDC->Vdc_neg) < V_dc_Low_Limit_local){
          Status_DCSource=NO_DC_SOURCE;
          //          DPC_FLT_Faulterror_Set(DPC_FLT_ERROR_DC_OFF);
        }
        else if((DATA_VDC->Vdc_pos + DATA_VDC->Vdc_neg) < V_dc_UV_Limit_local){
          Status_DCSource=UV_DC_SOURCE;
          //          DPC_FLT_Faulterror_Set(DPC_FLT_ERROR_DC_UV); 
        }      
        else {
          Status_DCSource=OK_DC_SOURCE;
          if((DATA_VDC->Vdc_pos + DATA_VDC->Vdc_neg) < V_dc_UVLO_Limit_local){
            DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_PFC_UVLO);
          }            
        }
      }
    }
  }  
  tDC_Source->Status_DCSource=Status_DCSource;
  return Status_DCSource;
}



/**
* @brief  DPC_MISC_DAB_HVDCSource_Check: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/

DPC_MISC_DCSourceStatus_t DPC_MISC_DAB_HVDCSource_Check(DPC_MISC_DCSource_t  *tDC_Source,DPC_MISC_DCSourceLimit_t tDC_SourceLimit,DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw){
  
  DPC_MISC_DCSourceStatus_t Status_DCSource = tDC_Source->Status_DCSource; 
  
  uint32_t DATA_VDC;                                                                       /*!< >*/
  uint32_t DATA_IDC;                                                                       /*!< >*/
  
  
  uint16_t V_dc_Limit_local;                                                                             /*!< >*/
  uint16_t V_dc_UVLO_Limit_local;                                                                        /*!< UnderVoltage Lock-out protection for DC/AC operation [Expressed in Bits]*/
  uint16_t V_dc_UV_Limit_local;                                                                          /*!< UnderVoltage protection for DC/AC operation [Expressed in Bits]*/
  uint16_t V_dc_Low_Limit_local;                                                                         /*!< Minimum DC Voltage F - DC/AC operation [Expressed in Bits]*/ 
  uint16_t I_dc_Limit_local;                                                                             /*!< Overcurrent DC protection for DC/AC operation [Expressed in Bits]*/
  
  
  V_dc_Limit_local=tDC_SourceLimit.V_dc_pos_Limit;                                                       /*!< >*/
  V_dc_UVLO_Limit_local=tDC_SourceLimit.V_dc_pos_UVLO_Limit;                                             /*!< >*/
  V_dc_UV_Limit_local=tDC_SourceLimit.V_dc_pos_UV_Limit;                                                 /*!< >*/
  V_dc_Low_Limit_local=tDC_SourceLimit.V_dc_pos_Low_Limit;                                               /*!< >*/
  I_dc_Limit_local=tDC_SourceLimit.I_dc_pos_Limit;                                                       /*!< >*/
    
  DATA_VDC = DAB_ADC_DCraw->VDAB1;                                                                    /*!< Read DC Voltage ADC raw data>*/
  DATA_IDC = DAB_ADC_DCraw->IDAB1;                                                                    /*!< Read DC Current ADC raw data>*/
  
  
  if(DATA_IDC > I_dc_Limit_local)
  {
    Status_DCSource=OVERCURRENT_DC_SOURCE;
    DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OCS);      
  }
  else
  {
    if(DATA_VDC > V_dc_Limit_local)
    {  
      Status_DCSource=OVERVOLTAGE_DC_SOURCE;
      DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OVS);  
    }
    else{  
      
      if(DATA_VDC == 0){ 
        Status_DCSource=WAIT_DC_SOURCE;
      }
      else{
        if(DATA_VDC > V_dc_Limit_local){
          Status_DCSource=OVERVOLTAGE_DC_SOURCE;
          DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_OVS);
        } 
        else if(DATA_VDC < V_dc_Low_Limit_local){
          Status_DCSource=NO_DC_SOURCE;
          //          DPC_FLT_Faulterror_Set(DPC_FLT_ERROR_DC_OFF);
        }
        else if(DATA_VDC < V_dc_UV_Limit_local){
          Status_DCSource=UV_DC_SOURCE;
          //          DPC_FLT_Faulterror_Set(DPC_FLT_ERROR_DC_UV); 
        }      
        else {
          Status_DCSource=OK_DC_SOURCE;
          if(DATA_VDC < V_dc_UVLO_Limit_local){
            DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_PFC_UVLO);
          }            
        }
      }
    }
  }  
  tDC_Source->Status_DCSource=Status_DCSource;
  return Status_DCSource;
}

/**
* @brief  DPC_MISC_DAC_Init: 
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx and STM32F74x microcontroller family   
*/
  void DPC_DAC_Init(DAC_Channel_STRUCT *DAC_CH_Sub, uint8_t DAC_CH1_INIT_loc, uint8_t DAC_CH2_INIT_loc, uint8_t DAC_CH3_INIT_loc, uint16_t DAC_G_CH1_Init_loc,uint16_t DAC_G_CH2_Init_loc , uint16_t DAC_G_CH3_Init_loc, uint16_t DAC_B_CH1_Init_loc,uint16_t DAC_B_CH2_Init_loc , uint16_t DAC_B_CH3_Init_loc){
  
  DAC_CH_Sub->CH1=DAC_CH1_INIT_loc;
  DAC_CH_Sub->CH2=DAC_CH2_INIT_loc;
  DAC_CH_Sub->CH3=DAC_CH3_INIT_loc;
  DAC_CH_Sub->Gain_CH1=DAC_G_CH1_Init_loc;
  DAC_CH_Sub->Gain_CH2=DAC_G_CH2_Init_loc;
  DAC_CH_Sub->Gain_CH3=DAC_G_CH3_Init_loc;
  DAC_CH_Sub->Bias_CH1=DAC_G_CH1_Init_loc;
  DAC_CH_Sub->Bias_CH2=DAC_G_CH2_Init_loc;
  DAC_CH_Sub->Bias_CH3=DAC_G_CH3_Init_loc;  
  
}




/**
* @brief  DPC_MISC_OB_Init: Allow to define the Option Bytes related to the power converter application
*         
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_MISC_OB_Init(void){
  
FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  OptionsBytesStruct.OptionType=OPTIONBYTE_BOOT_LOCK;
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
  
  if (OptionsBytesStruct.BootEntryPoint!=OB_BOOT_LOCK_ENABLE){
    
    HAL_FLASH_Unlock();  
    HAL_FLASH_OB_Unlock();
    
    OptionsBytesStruct.BootEntryPoint=OB_BOOT_LOCK_ENABLE;
    if (HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
    {
      Error_Handler();
    }  
    HAL_FLASH_OB_Lock();
    HAL_FLASH_OB_Launch();  
    
    HAL_FLASH_Lock();
  }    
} 
