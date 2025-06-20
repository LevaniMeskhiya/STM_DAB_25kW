/**
  ******************************************************************************
  * @file           : DPC_Loopctrl.c
  * @brief          : Loop Control  Module
  ******************************************************************************
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
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

#include "DPC_Loopctrl.h"
#include "DPC_Pid.h"
#include "DPC_Math.h"

/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  DPC_LCT_VoltageDC_Control: DPC_LCT_VoltageDC_Control .
  * @param  VOLTAGECTRL_sub: Voltage control handler
  * @param  Id_ctrl_sub: d-axis output 
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microcontroller family  
  */  
void DPC_LCT_VoltageDC_Control(DPC_LCT_VDC_Ctrl_t *VOLTAGECTRL_sub, float *Id_ctrl_sub){
  float Vdc_ref_sub=VOLTAGECTRL_sub->Vdc_ref;
  float Vdc_feed_sub=VOLTAGECTRL_sub->Vdc_feed;
  
  DPC_PI(Vdc_ref_sub, Vdc_feed_sub , &VOLTAGECTRL_sub->pPI_VDC_CTRL);
  VOLTAGECTRL_sub->Id_ctrl=VOLTAGECTRL_sub->pPI_VDC_CTRL.PIout_sat;
  *Id_ctrl_sub=VOLTAGECTRL_sub->pPI_VDC_CTRL.PIout_sat;
}

/**
  * @brief  DPC_LCT_CurrentDC_Control: DPC_LCT_CurrentDC_Control .
  * @param  VOLTAGECTRL_sub: Current control handler
  * @param  Out_ctrl_sub: 
  * 
  * @retval None
  *
  * @note Function valid for STM32G4xx microcontroller family  
  */  
void DPC_LCT_CurrentDC_Control(DPC_LCT_IDC_Ctrl_t *CURRENTCTRL_sub, float *Out_ctrl_sub){
  float Vdc_ref_sub=CURRENTCTRL_sub->Idc_ref;
  float Vdc_feed_sub=CURRENTCTRL_sub->Idc_feed;
  
  DPC_PI(Vdc_ref_sub, Vdc_feed_sub , &CURRENTCTRL_sub->pPI_IDC_CTRL);
  CURRENTCTRL_sub->Out_ctrl=CURRENTCTRL_sub->pPI_IDC_CTRL.PIout_sat;
  *Out_ctrl_sub=CURRENTCTRL_sub->pPI_IDC_CTRL.PIout_sat;
}





/**
  * @brief  DPC_LCT_DAB_Ctrl_Reset: 
  * 
  * @retval null 
  *
  * @note Function valid for STM32G4xx microcontroller family  
  */ 
void DPC_LCT_DAB_Ctrl_Reset(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc)
{
        pDAB_CTRL_loc->VOLTAGECTRL.pPI_VDC_CTRL.resetPI=SET;        ///Mantein Not used regulator in reset mode
}





/**
* @brief  DPC_LCT_BurstCtrl_Init: Burst mode init function
* @param  pBURST: Burst mode handler
* @param  BurstEnable: Burst mode Enable
* @param  Vref_hist_VOLT: Hysteresis set-point
* @param  delta_Vref_hist_VOLT: Hysteresis window
* @param  I_DC_NoLoadLimit_A: No Load threshond
* @param  I_dc_LOW_LOAD_Limit_AMP_loc: Low Load threshond
* @param  duty_no_load_local: No load duty
* @param  duty_low_load_local: Low load duty
* @param  pDPC_ADC_Conf: ADC handler
* 
* @retval none 
*
* @note Function valid for STM32G4xx microcontroller family  
*/ 
void DPC_LCT_BurstCtrl_Init(DPC_ACT_Burst_t *pBURST,FlagStatus Burst_Enable_loc,uint16_t Vref_hist_VOLT,uint16_t delta_Vref_hist_VOLT,float I_DC_NoLoadLimit_A,float I_dc_LOW_LOAD_Limit_AMP_loc,float duty_no_load_local,float duty_low_load_local,DPC_ADC_Conf_t *pDPC_ADC_Conf){
  
  
  uint16_t Vref_Hist;
  uint16_t delta_Vref_hist_loc;
  uint16_t Vout_load_max;                                                                       /*!< Local hysteresis higher ouput DC voltage threshold expressed in Bits */
  uint16_t Vout_load_min;                                                                       /*!< Local hysteresis lower ouput DC voltage threshold expressed in Bits */  
  uint16_t I_dc_NoLoad_Limit;                                                                   /// Local variable to pass Output current threshold (Expressed in AMPs) to determinate No Load Condition 
  uint16_t I_dc_LOW_LOAD_Limit_loc;                                                             /// Local variable to pass Output current theshold (Expressed in AMPs) to determinate Low Load Condition 

    /* configuration init */   
  Vref_Hist=(uint16_t)(((float)Vref_hist_VOLT*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);
  delta_Vref_hist_loc=(uint16_t)(((float)delta_Vref_hist_VOLT*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);
  Vout_load_max=Vref_Hist+delta_Vref_hist_loc;                                              /*!< Obtain and set higher output voltage term*/
  Vout_load_min=Vref_Hist-delta_Vref_hist_loc;                                              /*!< Obtain and set lower output voltage term*/ 
  
  I_dc_NoLoad_Limit=(uint16_t)(((float)I_DC_NoLoadLimit_A*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);       /// (IDC_No_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias  
  I_dc_LOW_LOAD_Limit_loc=(uint16_t)(((float)I_dc_LOW_LOAD_Limit_AMP_loc*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);     /// (IDC_Low_LOAD_Threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias  
  

      /* configuration output */   
  pBURST->Vref_hist=Vref_Hist;
  pBURST->delta_Vref_hist=delta_Vref_hist_loc;
  pBURST->Vout_max=Vout_load_max;
  pBURST->Vout_min=Vout_load_min;   
  pBURST->Duty_Limit=0.5;
  pBURST->Duty_noload=duty_no_load_local;
  pBURST->Duty_lowload=duty_low_load_local;  
  pBURST->Burst_Enable=Burst_Enable_loc;
  if (pBURST->Burst_Enable!=SET) {
  pBURST->BURST_Status=BURST_Disable;
  }
  pBURST->Iout_no_load_threshold=I_dc_NoLoad_Limit;
  pBURST->Iout_low_load_threshold=I_dc_LOW_LOAD_Limit_loc;
  
}




/**
* @brief  DPC_LCT_DAB_BurstCheck: Function used to obtain FSM operation during the BURST
* @param  p_Data_Sub:  DC voltage feedback data
* @param  iDC_Data_Sub: DC current feedback data
* @param  BURST_CTRL_f:  Burst mode handler
* 
* @retval BURST_Status: Burst mode status 
*
* @note Function valid for STM32G4xx microcontroller family  
*/ 
DPC_ACT_BurstStatus_t DPC_LCT_DAB_BurstCheck(DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw,DPC_ACT_Burst_t *BURST_CTRL_f){ 
  
  
  
  DPC_ACT_BurstStatus_t BURST_Status; 
  uint16_t Vout_load_max;                                                               /*!< Local hysteresis higher ouput DC voltage threshold expressed in Bits */
  uint16_t Vout_load_min;                                                               /*!< Local hysteresis lower ouput DC voltage threshold expressed in Bits */
//  uint16_t I_load_Burst=0;                                                                /*!< */
  
    /* Burst check task */    
  if(BURST_CTRL_f->Burst_Enable==SET){                                                  /** If Burst_Enable is SET */
        
    BURST_CTRL_f->Vout_load=DAB_ADC_DCraw->VDAB2;                                              /*!< Pass voltages data in local terms */
    Vout_load_max=BURST_CTRL_f->Vout_max;                                               /*!< Set higher output voltage term*/
    Vout_load_min=BURST_CTRL_f->Vout_min;                                               /*!< Set lower output voltage term*/          
//    I_load_Burst=DAB_ADC_DCraw->IDAB2;                                                      /*!< Pass current data in local terms [0]=Iload */  
    
//    if(I_load_Burst<=(BURST_CTRL_f->Iout_no_load_threshold)){                           ///NO_LOAD  Check        
      if (BURST_CTRL_f->Vout_load>Vout_load_max) 
      {
        BURST_Status=BURST_Complete;
      }
      else if (BURST_CTRL_f->Vout_load<Vout_load_min)   
      {   
        BURST_Status=BURST_Progress;
      }
      else
      {
        BURST_Status=BURST_Complete;   
      }     
//    }
//    else
//    {
//      BURST_Status=BURST_Error;   
//    }    
  }
  else{                                                                                 /** If Burst_Enable is RESET */
    BURST_Status=BURST_Disable; 
  }
  
    /* Output */     
  BURST_CTRL_f->BURST_Status=BURST_Status;
//  BURST_CTRL_f->uI_load_Burst=I_load_Burst;
  
  return BURST_Status; 
}




/**
  * @brief  DPC_LCT_DAB_InrushCheck: Function used to obtain FSM operation during the INRUSH
  * @param  INRUSH_CTRL_f: Inrush mode handler
  * 
  * @retval INRUSH_StatusTypeDef Inrush mode status
  *
  * @note Function valid for STM32G4xx microcontroller family  
  */ 
DPC_ACT_InrushStatus_t DPC_LCT_DAB_InrushCheck(DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw,DPC_LCT_InrushCtrl_t *INRUSH_CTRL_f){
  
  DPC_ACT_InrushStatus_t  INRUSH_Status;                                                /*!< Local INRUSH_StatusTypeDef */
  uint16_t Vout_load;                                                                   /*!< Local actual DPC output voltage expressed in Bits*/
  uint16_t I_load_Inrush=0;                                                             /*!< Local actual DPC output current expressed in Bits*/
  uint16_t Vout_load_max;                                                               /*!< Local hysteresis higher ouput DC voltage threshold expressed in Bits */
  uint16_t Vout_load_min;                                                               /*!< Local hysteresis lower ouput DC voltage threshold expressed in Bits */
//  uint16_t Iout_load_threshold;                                                         /*!< Local ouput DC current threshold expressed in Bits */
   
    /* Inrush check task */     
  if (INRUSH_CTRL_f->InrushEnable==SET){                                                /** If InrushEnable is SET */ 
    
    Vout_load=DAB_ADC_DCraw->VDAB2;                                                     /*!< Pass voltages data in local terms ([0]=VDC_upper  [1]=VDC_lower) */
    I_load_Inrush=DAB_ADC_DCraw->IDAB2;                                                 /*!< Pass current data in local terms [0]=Iload */    
    INRUSH_CTRL_f->Vout_load=Vout_load;                                                 /*!< Store output voltage in "INRUSH" struct */
    Vout_load_max=INRUSH_CTRL_f->Vout_max;                                              /*!< Set higher output voltage term*/
    Vout_load_min=INRUSH_CTRL_f->Vout_min;                                              /*!< Set lower output voltage term*/
//    Iout_load_threshold=INRUSH_CTRL_f->Iout_load_threshold;                             /*!< Set output current term*/
    
//    if(I_load_Inrush<=Iout_load_threshold){                                             /*!< NO_LOAD  Check */
      if (Vout_load>Vout_load_max)                                                      /*!< ERROR Check - If occur AC OVERVOLTAGE or 3W-4W are not properly configurated*/
      {  
        INRUSH_Status=INRUSH_Error;   
      }
      else if (Vout_load<Vout_load_min)                                                 /*!< Inrush Check - If NOT occured AC UNDERVOLTAGE or 3W-4W are not properly configurated*/
      {   
        INRUSH_Status=INRUSH_Progress;     
      }
      else                                                                              /** InrushEnable is SET and completed*/
      {
        INRUSH_Status=INRUSH_Complete;  
      }
//    }///  END NO_LOAD  Check                                                                                    
//    else                                                                                /** InrushEnable is SET but DC current is present during the inrush (ERROR) */ 
//    {
//      INRUSH_Status=INRUSH_Error;   
//    } 
  }
  else                                                                                  /** If InrushEnable is RESET */ 
  {
    INRUSH_Status=INRUSH_Disable;
  }  
  
    /* Output */    
  INRUSH_CTRL_f->INRUSH_Status=INRUSH_Status;
  INRUSH_CTRL_f->I_load_Inrush=I_load_Inrush;  
  
  return INRUSH_Status;
}



/**
  * @brief  DPC_LCT_Inrush_Init:  Inrush mode init function
  * @param  INRUSH_CTRL_f:  Inrush mode handler
  * @param  Vref_Hist_V: Hysteresis reference voltage
  * @param  DeltaHigh_VrefHist_V: Hysteresis High threshold window voltage
  * @param  DeltaLow_VrefHist_V: Hysteresis Low threshold window voltage
  * @param  I_DC_NoLoadLimit_A: DC current No load limit 
  * @param  InrushEnable_loc: Inrush enable
  * @param  pDPC_ADC_Conf: Digital Power Converter - ADC handler
  * 
  * @retval INRUSH_StatusTypeDef 
  *
  * @note Function valid for STM32G4xx microcontroller family  
  */ 
void DPC_LCT_Inrush_Init(DPC_LCT_InrushCtrl_t *INRUSH_CTRL_f,uint16_t Vref_Hist_V,uint16_t DeltaHigh_VrefHist_V,uint16_t DeltaLow_VrefHist_V,float I_DC_NoLoadLimit_A,FlagStatus InrushEnable_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf){
  
  
  uint16_t Vref_Hist;
  uint16_t deltahigh_Vref_hist_loc;
  uint16_t deltalow_Vref_hist_loc;
  uint16_t Vout_load_max;                                                                       /*!< Local hysteresis higher ouput DC voltage threshold expressed in Bits */
  uint16_t Vout_load_min;                                                                       /*!< Local hysteresis lower ouput DC voltage threshold expressed in Bits */  
  uint16_t I_dc_NoLoad_Limit;                                                                   /// Local variable to pass Output current threshold (Expressed in AMPs) to determinate No Load Condition  
  
  
  
  Vref_Hist=(uint16_t)(((float)Vref_Hist_V*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);
  deltahigh_Vref_hist_loc=(uint16_t)(((float)DeltaHigh_VrefHist_V*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);
  deltalow_Vref_hist_loc=(uint16_t)(((float)DeltaLow_VrefHist_V*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);
  
  
  Vout_load_max=Vref_Hist+deltahigh_Vref_hist_loc;                                          /*!< Obtain and set higher output voltage term*/
  Vout_load_min=Vref_Hist-deltalow_Vref_hist_loc;                                           /*!< Obtain and set lower output voltage term*/
  
  I_dc_NoLoad_Limit=(uint16_t)(((float)I_DC_NoLoadLimit_A*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);   /// (IDC_No_LOAD_threshold [Ampere] * DC Current Sensing Gain) + DC Current Sensing Bias
  

  INRUSH_CTRL_f->Vref_hist=Vref_Hist;
  INRUSH_CTRL_f->deltahigh_Vref_hist=deltahigh_Vref_hist_loc;
  INRUSH_CTRL_f->deltalow_Vref_hist=deltalow_Vref_hist_loc;  
  INRUSH_CTRL_f->InrushEnable=InrushEnable_loc;
  if (INRUSH_CTRL_f->InrushEnable!=SET) {
    INRUSH_CTRL_f->INRUSH_Status=INRUSH_Disable;
  }
  INRUSH_CTRL_f->Vout_max=Vout_load_max;
  INRUSH_CTRL_f->Vout_min=Vout_load_min;  
  INRUSH_CTRL_f->Iout_load_threshold=I_dc_NoLoad_Limit;
}






/**
* @brief  DPC_LPCNTRL_DAB_Init: To init Dual Active Bridge converter 
* @param  TBD
* 
* @retval TBD 
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_LPCNTRL_DAB_Init(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,DPC_LCT_DAB_CtrlState_t DAB_CTRL_State,uint16_t DAB_VDC_Ref_loc,uint16_t DAB_IDC_Ref_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf)
{
  uint16_t DAB_VDC_Ref_BITs_loc;                                                                                /// Local variable to pass Output voltage reference  (Expressed in BITs)   
  uint16_t DAB_IDC_Ref_BITs_loc;                                                                                /// Local variable to pass Output current reference  (Expressed in BITs) 
  DAB_VDC_Ref_BITs_loc=(uint16_t)(((float)DAB_VDC_Ref_loc*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);   /// (V_dc_ref [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias
  DAB_IDC_Ref_BITs_loc=(uint16_t)(((float)DAB_IDC_Ref_loc*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);   /// (I_dc_ref [Amp] * DC Current Sensing Gain) + DC Current Sensing Bias
  
  pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.fDAB_VDC_RefNext_V=DAB_VDC_Ref_loc;
  pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.DAB_VDC_Ref_BITs=DAB_VDC_Ref_BITs_loc;
  pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.fDAB_IDC_Ref_A=DAB_IDC_Ref_loc;
  pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.DAB_IDC_Ref_BITs=DAB_IDC_Ref_BITs_loc;    
  
  pDAB_CTRL_loc->VdcCTRL_Reset=RESET;                                                                   /// Reset of the controller for first initialization
  pDAB_CTRL_loc->IdcCTRL_Reset=RESET;                                                                   /// Reset of the controller for first initialization  
  pDAB_CTRL_loc->DAB_CTRL_State=DAB_CTRL_State;                                                         /// Start-up control type selection (OPEN - CURRENT - VOLTAGE |Loop)

  pDAB_CTRL_loc->swFreq_Hz=DPC_DAB_PWM_FREQ;
  pDAB_CTRL_loc->swPeriod_s=1.0/DPC_DAB_PWM_FREQ;
  pDAB_CTRL_loc->trafoTurnRatio=DPC_DAB_TRAFO_TURN_RATIO;  
  pDAB_CTRL_loc->Inductance_H=DPC_DAB_INDUCTANCE;    
}


/**
* @brief  DPC_LPCNTRL_DAB_Vset: Set Voltage refernce  
* @param  TBD
* 
* @retval TBD 
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_LPCNTRL_DAB_Vset(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,float fDAB_VDC_Ref_V_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf)
{
  uint16_t DAB_VDC_Ref_BITs_loc;                                                                                /// Local variable to pass Output voltage reference  (Expressed in BITs)   
  DAB_VDC_Ref_BITs_loc=(uint16_t)(((float)fDAB_VDC_Ref_V_loc*pDPC_ADC_Conf->fGVdc)+pDPC_ADC_Conf->fBVdc);   /// (V_dc_ref [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias
  
  pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.fDAB_VDC_Ref_V=fDAB_VDC_Ref_V_loc;
  pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.DAB_VDC_Ref_BITs=DAB_VDC_Ref_BITs_loc;    
 }


/**
* @brief  DPC_LPCNTRL_DAB_Iset: Set Current refernce  
* @param  TBD
* 
* @retval TBD 
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_LPCNTRL_DAB_Iset(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,float fDAB_IDC_Ref_A_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf)
{
  uint16_t DAB_IDC_Ref_BITs_loc;                                                                                /// Local variable to pass Output voltage reference  (Expressed in BITs)   
  DAB_IDC_Ref_BITs_loc=(uint16_t)(((float)fDAB_IDC_Ref_A_loc*pDPC_ADC_Conf->fGIdc)+pDPC_ADC_Conf->fBIdc);   /// (V_dc_ref [Volt] * DC Voltage Sensing Gain) + DC Voltage Sensing Bias
  
  pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.fDAB_IDC_Ref_A=fDAB_IDC_Ref_A_loc;
  pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.DAB_IDC_Ref_BITs=DAB_IDC_Ref_BITs_loc;    
 }
 
 
/**
* @brief  DPC_LPCNTRL_DAB_Vramp: 
* @param  TBD
* 
* @retval TBD 
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_LPCNTRL_DAB_Vramp(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc)
{  
 
  float fSlewRate_VtoSec = 10;
  uint32_t uhUpdateFreq_Hz = 3000;  
  float fSlewRate = fSlewRate_VtoSec / uhUpdateFreq_Hz;
  
  pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.fDAB_VDC_RefPrev_V=pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.fDAB_VDC_Ref_V;                         // Save previos step voltage
  
  pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.fDAB_VDC_Ref_V=DPC_MATH_SlewRateLim(pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.fDAB_VDC_RefNext_V , pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.fDAB_VDC_RefPrev_V, fSlewRate);
     
}

/**
* @brief  DPC_LPCNTRL_DAB_Iramp: 
* @param  TBD
* 
* @retval TBD 
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_LPCNTRL_DAB_Iramp(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc)
{  
 
  float fSlewRate_AtoSec = 1;
  uint32_t uhUpdateFreq_Hz = 3000;  
  float fSlewRate = fSlewRate_AtoSec / uhUpdateFreq_Hz;
  
  pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.fDAB_IDC_RefPrev_A=pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.fDAB_IDC_Ref_A;                         // Save previos step current
  
  pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.fDAB_IDC_Ref_A=DPC_MATH_SlewRateLim(pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.fDAB_IDC_RefNext_A , pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.fDAB_IDC_RefPrev_A, fSlewRate);
     
}





/**
  * @brief  DPC_LPCNTRL_DAB_Mode: CORE of control loop (VOLTAGE LOOP or CURRENT LOOP or OPEN LOOP)
  * @param  pDAB_CTRL_loc: 
  * @param  : TBD
  * @param  : TBD
  * @param  : TBD
  * 
  * @retval null 
  *
  * @note Function valid for STM32G4xx microconroller family  
  */ 
void DPC_LPCNTRL_DAB_Mode(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc, DAB_ADC_RAW_Struct_t* DAB_ADC_DC_RAW_Sub)
{
  
  float PhaseShift=0;

  
   if(pDAB_CTRL_loc->DAB_CTRL_State==DAB_VOLTAGE_LOOP) /// Voltage and Current control closed
  {   
    

    pDAB_CTRL_loc->VOLTAGECTRL.Vdc_ref=pDAB_CTRL_loc->pDAB_VCTRL_SlewRate.DAB_VDC_Ref_BITs;     /// Reference voltage acquision [Expresd in Bits - See DPC_Application_Conf.h for more details]
    pDAB_CTRL_loc->VOLTAGECTRL.Vdc_feed=DAB_ADC_DC_RAW_Sub->VDAB2;                              /// Feedback voltage acquision [Expresd in Bits - See DPC_Application_Conf.h for more details] 
    DPC_LCT_VoltageDC_Control(&pDAB_CTRL_loc->VOLTAGECTRL,&PhaseShift);                         /// 
    pDAB_CTRL_loc->VOLTAGECTRL.pPI_VDC_CTRL.resetPI=pDAB_CTRL_loc->VdcCTRL_Reset;               /// Release PI accumulator
    pDAB_CTRL_loc->PhSh_PowerCTRL_norm=PhaseShift;
  }
  else if(pDAB_CTRL_loc->DAB_CTRL_State==DAB_CURRENT_LOOP) /// Current control closed
  {
    pDAB_CTRL_loc->CURRENTCTRL.Idc_ref=pDAB_CTRL_loc->pDAB_ICTRL_SlewRate.DAB_IDC_Ref_BITs;     /// Reference current acquision [Expresd in Bits - See DPC_Application_Conf.h for more details]
    pDAB_CTRL_loc->CURRENTCTRL.Idc_feed=DAB_ADC_DC_RAW_Sub->IDAB2;                              /// Feedback current acquision [Expresd in Bits - See DPC_Application_Conf.h for more details] 
    DPC_LCT_CurrentDC_Control(&pDAB_CTRL_loc->CURRENTCTRL,&PhaseShift);                         /// 
    pDAB_CTRL_loc->CURRENTCTRL.pPI_IDC_CTRL.resetPI=pDAB_CTRL_loc->IdcCTRL_Reset;               /// Release PI accumulator
    pDAB_CTRL_loc->PhSh_PowerCTRL_norm=PhaseShift;
  }  
  else if(pDAB_CTRL_loc->DAB_CTRL_State==DAB_OPEN_LOOP) /// Open LOOP
  {
    pDAB_CTRL_loc->VOLTAGECTRL.pPI_VDC_CTRL.resetPI=SET;                                        /// Mantein Not used regulator in reset mode
    pDAB_CTRL_loc->PhSh_PowerCTRL_norm=pDAB_CTRL_loc->PhSh_CTRL_MAN_norm;
  }
  else /// Generate a ERROR to prevent condition unknow!!! 
  {
    pDAB_CTRL_loc->VOLTAGECTRL.pPI_VDC_CTRL.resetPI=SET;                                        /// Mantein Not used regulator in reset mode  
    pDAB_CTRL_loc->PhSh_PowerCTRL_norm=0;
  }
  
  
    pDAB_CTRL_loc->PhSh_PowerCTRL_rad=PI*pDAB_CTRL_loc->PhSh_PowerCTRL_norm;  

}



/**
* @brief  DPC_LPCNTRL_DAB_TpzPWM_DutyTime_Calc: To calculate the PWM timing in trapezoidal modulation tecnique 
*         
* @param  
             *
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_LPCNTRL_DAB_TpzPWM_DutyTime_Calc(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL, float VHV, float VLV, float n, float phi_rad, float fsw)
{
  float Omega_1_rad;                                                                            /*! Primary side phase shift [expressed in radiants]*/
  float Omega_2_rad;                                                                            /*! Secondary side phase shift [expressed in radiants] */
  float D1_time;                                                                                /*! Primary side time phase shift [expressed in seconds]*/
  float D2_time;                                                                                /*! Secondary side time phase shift [expressed in seconds]*/
  float Duty1;                                                                                  /*! Primary side phase shift duty [-]*/
  float Duty2;                                                                                  /*! Secondary side phase shift duty [-]*/
  float Tsw;                                                                                    /*! Local Internal switching period*/
//  float L;                                                                                      /*! Local Internal series inductance value*/
  float phi_rad_max_trian_rad;                                                                  /*! Max Phase shift allowed in Triangular modulation tecnique*/
  float phi_rad_max_trap_rad;                                                                   /*! Max Phase shift allowed in Trapezoidal modulation tecnique*/
  float phi_rad_max_SPS_rad;                                                                    /*! Max Phase shift allowed in Single Pahse Shift modulation tecnique*/
  
  
  Tsw=pDAB_CTRL->swPeriod_s;                                                                    /// Internal switching period assignement
//  L=pDAB_CTRL->Inductance_H;
  phi_rad_max_trian_rad=(PI_2)*(1-((n*VLV)/(VHV)));                                             /// Calculate Max Phase shift allowed in Triangular modulation tecnique
  phi_rad_max_trap_rad=(PI_2)*(1-((n*VHV*VLV)/((VHV*VHV)+n*VHV*VLV+((n*VLV)*(n*VLV)))));        /// Calculate Max Phase shift allowed in Trapezoidal modulation tecnique
  phi_rad_max_SPS_rad=PI/2;                                                                     /// Calculate Max Phase shift allowed in Single Pahse Shift modulation tecnique

  
  
  /*! Phase shift determination according to the converter voltage gain  */
  if (VHV>= n*VLV){
    Omega_2_rad = (PI*((n*VLV)-VHV)+(2*VHV*phi_rad))/(2*(VHV+n*VLV))-pDAB_CTRL->TPZ_comp;
    Omega_1_rad=phi_rad-Omega_2_rad;
  }
  else{
    Omega_1_rad = (PI*(VHV-(n*VLV))+(2*n*VLV*phi_rad))/(2*(VHV+n*VLV))+pDAB_CTRL->TPZ_comp;
    Omega_2_rad=phi_rad-Omega_1_rad;
  }    
  
  /*! Phase shift time determination */  
  D1_time=(1-(2*Omega_1_rad/PI))*(Tsw/2);
  D2_time=(1-(2*Omega_2_rad/PI))*(Tsw/2);
  
  /*! Phase shift duty determination */  
  Duty1=D1_time/(Tsw/2);
  Duty2=-D2_time/(Tsw/2); 
  
  /*! Processed terms */
  pDAB_CTRL->DAB_ACT.TpzOmega_1_rad=Omega_1_rad;
  pDAB_CTRL->DAB_ACT.TpzOmega_2_rad=Omega_2_rad;  
  pDAB_CTRL->DAB_ACT.TpzD1_time=D1_time;
  pDAB_CTRL->DAB_ACT.TpzD2_time=D2_time;    
  pDAB_CTRL->DAB_ACT.TpzDuty1=Duty1;
  pDAB_CTRL->DAB_ACT.TpzDuty2=Duty2; 
  pDAB_CTRL->phi_rad_max_trap_rad=phi_rad_max_trap_rad;
  pDAB_CTRL->phi_rad_max_trian_rad=phi_rad_max_trian_rad;  
  pDAB_CTRL->phi_rad_max_single_rad=phi_rad_max_SPS_rad;  
  
  
}



/**
* @brief  DPC_LPCNTRL_DAB_MaxPowerCalc: To calculate the max power of the converter 
*         
* @param  TBD
*
* @retval TBD 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_LPCNTRL_DAB_MaxPowerCalc(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL, float VHV, float VLV, float n, float fsw, float L)
{  
  
  pDAB_CTRL->Pmax_tri_W=(((n*VLV)*(n*VLV))*(VHV-(n*VLV)))/(4*fsw*L*VHV);  
  pDAB_CTRL->Pmax_tpz_W=((n*VHV*VLV)*(n*VHV*VLV))/(4*fsw*L*((VHV*VHV)+n*VHV*VLV+((n*VLV)*(n*VLV))));  
  pDAB_CTRL->Pmax_sps_W=(n*VHV*VLV)/(8*fsw*L);   
}





/**
* @brief  DPC_LPCNTRL_DAB_ModulatorSelector: allow to manage the moduloation tecnique according to the request
*         
* @param  TBD
*
* @retval TBD 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_LPCNTRL_DAB_ModulatorSelector(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL){
  
  
  if(pDAB_CTRL->ModTecnique==SPWM)
  {
    pDAB_CTRL->Duty1_Act_CTRL_norm=1.0;
    pDAB_CTRL->Duty2_Act_CTRL_norm=1.0;
    pDAB_CTRL->PhSh_Act_CTRL_norm=pDAB_CTRL->PhSh_PowerCTRL_norm;
  }
  else if (pDAB_CTRL->ModTecnique==TPZ)
  {
    pDAB_CTRL->Duty1_Act_CTRL_norm=pDAB_CTRL->DAB_ACT.TpzDuty1;
    pDAB_CTRL->Duty2_Act_CTRL_norm=pDAB_CTRL->DAB_ACT.TpzDuty2;
    pDAB_CTRL->PhSh_Act_CTRL_norm=0;
  }
  else if (pDAB_CTRL->ModTecnique==TRG)
  {
    pDAB_CTRL->Duty1_Act_CTRL_norm=0;
    pDAB_CTRL->Duty2_Act_CTRL_norm=0;
    pDAB_CTRL->PhSh_Act_CTRL_norm=0;  
  }
  else if (pDAB_CTRL->ModTecnique==MAN)
  {
    pDAB_CTRL->Duty1_Act_CTRL_norm=pDAB_CTRL->Duty1_CTRL_MAN_norm;
    pDAB_CTRL->Duty2_Act_CTRL_norm=pDAB_CTRL->Duty2_CTRL_MAN_norm;
    pDAB_CTRL->PhSh_Act_CTRL_norm=pDAB_CTRL->PhSh_CTRL_MAN_norm;  
  }    
  else
  {
    pDAB_CTRL->Duty1_Act_CTRL_norm=0;
    pDAB_CTRL->Duty2_Act_CTRL_norm=0;
    pDAB_CTRL->PhSh_Act_CTRL_norm=0;    
  }
  
  
}



/**
* @brief  DPC_LPCNTRL_DAB_InrushMode: CORE of inrush mode control
* @param  pDAB_CTRL_loc: 
* @param  : DAB_ADC_DC_RAW:
* @param  : TBD
* @param  : TBD
* 
* @retval null 
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_LPCNTRL_DAB_InrushMode(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,DPC_LCT_InrushCtrl_t *INRUSH_CTRL,DPC_ACT_PWM_t *tDPC_PWM_loc, DAB_ADC_RAW_Struct_t* DAB_ADC_DC_RAW)
{
  
  float InrushDuty1_CTRL=0;             //HV side phase shifting
  float InrushDuty2_CTRL=0;             //HV side phase shifting
  float InrushPhSh_CTRL=0;              //DAB phase shifting
  
  if(INRUSH_CTRL->InrushEnable==SET){
    
    float fSlewRate_AngtoSec = 0.03;                                                                            //SlewRate configuration - angle(normalized) step over second
    uint32_t uhUpdateFreq_Hz = 10000;                                                                           //Function time step execution - Must be adaped according to the related ISR period
    float fSlewRate_StepTerm = fSlewRate_AngtoSec / uhUpdateFreq_Hz;                                            //Calculation of step angle term
    
    pDAB_CTRL_loc->fDAB_INRSH_RefAngleNext_norm=0.15;                                                                //Requested stop(final) angle(normalized)
    pDAB_CTRL_loc->fDAB_INRSH_RefAnglePrev_norm=pDAB_CTRL_loc->fDAB_INRSH_RefAngleRef_norm;                               // Save previos step voltage
    
    pDAB_CTRL_loc->fDAB_INRSH_RefAngleRef_norm=DPC_MATH_SlewRateLim(pDAB_CTRL_loc->fDAB_INRSH_RefAngleNext_norm , pDAB_CTRL_loc->fDAB_INRSH_RefAnglePrev_norm, fSlewRate_StepTerm);
    
    InrushDuty1_CTRL=pDAB_CTRL_loc->fDAB_INRSH_RefAngleRef_norm;             //HV side phase shifting
    InrushDuty2_CTRL=0.0f;                                              //HV side phase shifting
    InrushPhSh_CTRL=0.0f;                                               //DAB phase shifting
  }
  
  else{
    //Zero phase shifting if Inrush disabled
    InrushDuty1_CTRL=0;
    InrushDuty2_CTRL=0;
    InrushPhSh_CTRL=0;    
  }

  //Output data  
  pDAB_CTRL_loc->Duty1_Act_CTRL_norm=InrushDuty1_CTRL;
  pDAB_CTRL_loc->Duty2_Act_CTRL_norm=InrushDuty2_CTRL;
  pDAB_CTRL_loc->PhSh_Act_CTRL_norm=InrushPhSh_CTRL;   
  
}
