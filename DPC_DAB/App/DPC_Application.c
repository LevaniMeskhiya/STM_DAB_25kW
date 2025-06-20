/**
******************************************************************************
* @file           : DPC_Application.c
* @brief          : Application program body
******************************************************************************
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
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

#include "stdint.h"
#include "stdbool.h"
#include "DPC_Application.h"

/* PACK CODE BEGIN Includes */
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//Include of Digital Power Conversion drivers
#include "DPC_Pid.h"
#include "DPC_Loopctrl.h"
#include "DPC_Miscellaneous.h"
#include "DPC_Timeout.h"
#include "DPC_FSM.h"
#include "DPC_Faulterror.h"
#include "DPC_ADC.h"
#include "DPC_Math.h"

/* PACK CODE END Includes */

/* PACK CODE BEGIN PV */

DPC_DAB_t DAB;                                                  /*!APPLICATION main struct*/

/*!ADC Sensing variables*/
uint32_t p_ADC1_Data[ADC1_CHs];                                 /*!< */
uint32_t p_ADC2_Data[ADC2_CHs];                                 /*!< */

GPIO_PinState StateUSRBTN;

/* PACK CODE END PV */

/* PACK CODE BEGIN PFP */

void   DPC_TTC_Init(DPC_DAB_t *DPC_DAB, DPC_DAB_InitMode_t DPC_DAB_InitMode){
  DPC_DAB->InitMode=DPC_DAB_InitMode;
}

/**
* @brief  DPC_MISC_DAB_UserMonitorConversion:
*
* @param TBD
* @retval void
*
* @note Function valid for STM32G4xx microcontroller family
*/
void DPC_DAB_UserMonitorConversion(DPC_DAB_t *DAB){

DAB->pDAB_CTRL.PhSh_Act_CTRL_rad=PI*(DAB->pDAB_CTRL.PhSh_Act_CTRL_norm);
DAB->pDAB_CTRL.PhSh_Act_CTRL_deg=180.0f*(DAB->pDAB_CTRL.PhSh_Act_CTRL_norm);

}
/* PACK CODE END PFP */

/**
* @}
*/
void DPC_APPLICATION_Init(void)
{
  /* PACK CODE BEGIN 2 */

  DPC_MISC_OB_Init();
  DPC_DAB_ADC_Init(&DAB.DPC_ADC_Conf,DPC_G_VDC1,DPC_B_VDC1,DPC_G_IDC1,DPC_B_IDC1,DPC_G_VDC2,DPC_B_VDC2,DPC_G_IDC2,DPC_B_IDC2);                                                                          ///
  DAB.StartUpCheck=STARTUPCHECK_INIT;                                                                                                                                                                   ///
  DAB.pDAB_CTRL.DPC_DTG_DAB1_bin=DPC_ACT_Calc_DeadTime(DPC_DAB_DT_DAB1);                                                                                                                                /// Find the optimal Deadtime timer configuration according the deadtime expressed in seconds
  DAB.pDAB_CTRL.DPC_DTG_DAB2_bin=DPC_ACT_Calc_DeadTime(DPC_DAB_DT_DAB2);                                                                                                                                /// Find the optimal Deadtime timer configuration according the deadtime expressed in seconds
  DPC_ACT_DAB_Conf_DeadTime(DAB.pDAB_CTRL.DPC_DTG_DAB1_bin,DAB.pDAB_CTRL.DPC_DTG_DAB2_bin);                                                                                                             /// Set DTC configuration
  DPC_MISC_APPL_Timer_Init(APPL_Tim1, RefreshTime_DESIDERED);                                                                                                                                           /// Function used to Init the timers APP_TIM1 (htim2) used in the power application
  DPC_MISC_APPL_Timer_Init(APPL_Tim2, RefreshTime_TO_DESIDERED);                                                                                                                                        /// Function used to Init the timers APP_TIM1 (htim3) used in the power application
  DPC_MISC_APPL_Timer_Init(APPL_Tim3, RefreshTime2_DESIDERED);                                                                                                                                          /// Function used to Init the timers APP_TIM3 (htim6) used in the power application
  DPC_MISC_APPL_Timer_Init(APPL_Tim5, RefreshTime3_DESIDERED);                                                                                                                                          /// Function used to Init the timers APP_TIM5 (htim7) used in the power application                                                                                                                                                                     /// Set DTC configuration
  DPC_MISC_Analog_Start();
  DPC_ACT_Init(DPC_BURST_PWM_FREQ,DPC_DAB_PWM_FREQ,DPC_DAB_PWM_INIT,&DAB.tDPC_PWM);
  DPC_PI_Init(&DAB.pDAB_CTRL.VOLTAGECTRL.pPI_VDC_CTRL,DPC_DAB_VCTRL_KP,DPC_DAB_VCTRL_KI,DPC_PI_VDC_TS,DPC_DAB_VCTRL_PI_SAT_UP,DPC_DAB_VCTRL_PI_SAT_DOWN,DPC_DAB_VCTRL_PI_SAT_EN,DPC_DAB_VCTRL_PI_AW_EN,DPC_VCTRL_PI_AWTG,DPC_DAB_PI_VDC_RES_VAL);  /// INIT PI VOLTAGE CTRL
  DPC_PI_Init(&DAB.pDAB_CTRL.CURRENTCTRL.pPI_IDC_CTRL,DPC_DAB_ICTRL_KP,DPC_DAB_ICTRL_KI,DPC_PI_IDC_TS,DPC_DAB_ICTRL_PI_SAT_UP,DPC_DAB_ICTRL_PI_SAT_DOWN,DPC_DAB_ICTRL_PI_SAT_EN,DPC_DAB_ICTRL_PI_AW_EN,DPC_ICTRL_PI_AWTG,DPC_DAB_PI_IDC_RES_VAL);                      /// INIT PI CURRENT CTRL
  DPC_MISC_DCSource_Init(&DAB.DC_Source_Limit,DPC_DAB_VDCHV_OVP,DPC_DAB_VDCHV_UV,DPC_DAB_VDCHV_UVLO,DPC_DAB_VDCHV_MIN,DPC_DAB_IDCHV_OCP,&DAB.DPC_ADC_Conf);                                             /// INIT DC_Source
  DPC_MISC_DAB_DCLoad_Init(&DAB.DC_Load_Limit,DPC_DAB_VDCLV_OVP,DPC_NO_LOAD_CURR,DPC_LOW_LOAD_CURR,DPC_DAB_IDCLV_OCP,&DAB.DPC_ADC_Conf);                                                                /// INIT DC_Load
  DPC_LPCNTRL_DAB_Init(&DAB.pDAB_CTRL,DPC_CTRL_INIT,DPC_DAB_VDC,DPC_DAB_IDC,&DAB.DPC_ADC_Conf);                                                                                                         /// INIT Inrush Mode TASK
  DPC_APP_FSM_Init(&DAB.PC_State,DPC_PC_State_Init,&DAB.FSM_Run_State,DPC_FSM_RUN_INIT);                                                                                                                /// Finite state machine Init function
  DPC_LCT_Inrush_Init(&DAB.INRUSH_CTRL,INRUSH_VREF_V,INRUSH_DELTA_MAX,INRUSH_DELTA_MIN,DPC_INRUSH_NO_LOAD_CURR,DPC_INRS_EN,&DAB.DPC_ADC_Conf);                                                          ///
  DPC_LCT_BurstCtrl_Init(&DAB.BURST_CTRL,DPC_BURST_EN,RUN_BURST_VREF_V,RUN_BURST_VHIST,DPC_NO_LOAD_CURR,DPC_LOW_LOAD_CURR,DPC_BURST_DUTY_NL,DPC_BURST_DUTY_LL,&DAB.DPC_ADC_Conf);                       /// INIT BURST CONTROL
  DPC_DAC_Init(&DAB.DAC_CH,DPC_DAC_CH1_INIT,DPC_DAC_CH2_INIT,DPC_DAC_CH3_INIT,DPC_DAC_G_CH1_INIT,DPC_DAC_G_CH2_INIT,DPC_DAC_G_CH3_INIT,DPC_DAC_B_CH1_INIT,DPC_DAC_B_CH2_INIT,DPC_DAC_B_CH3_INIT);       /// DAC debugging init function
  DPC_FSM_State_Init(DPC_FSM_STATE_INIT);                                                                                                                                                               /// Finite state machine init set-up
  DPC_TO_Init();                                                                                                                                                                                        /// Timeout module init function
  DPC_ADC_TrigSet(TRIGGER_TIME_EVENT);                                                                                                                      ///
  DPC_MISC_Appl_Timer_Start();                                        /// ###########  WARNING MUST BE THE LAST operation to prevent jump by interrrupt  ##########

  /* PACK CODE END 2 */

}

/*
* LM background task
*/
void DPC_APPLICATION_Process(void)
{
  /* PACK CODE BEGIN 3 */

    DPC_FSM_Application();                                                              //// DPC Finite State Machine main Application

  /* PACK CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
* @brief  Executes converter's state machine WAIT STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_WAIT_Func(void)
{
  bool RetVal = false;
  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_Wait);
  DAB.Status_DAB_Plug_HVDCSource=DPC_MISC_DAB_HVDC_SOURCE_Plugged(DAB.DC_Source_Limit,&DAB.DAB_ADC_RAW);                        ///Check DC SOURCE state reading AC Voltage and curent

  if (DAB.StartUpCheck==StartUpCheck_Disabled){
    RetVal = true;
    DPC_FSM_State_Set(DPC_FSM_IDLE);
  }
  else

  {
    if(DAB.Status_DAB_Plug_HVDCSource==OK_Plug_DCSource){
      if(DPC_TO_Set(DPC_TO_IDLE,TO_IDLE_Tick)==DPC_TO_OK){                                       ///TimeOut of Idle State of the Finite State Machine
        RetVal = true;
        DPC_FSM_State_Set(DPC_FSM_IDLE);
      }
      else{
        RetVal = false;
        DPC_FSM_State_Set(DPC_FSM_STOP);
      }
    }
    else{
      RetVal = true;
      DPC_FSM_State_Set(DPC_FSM_WAIT);
    }
  }
  return RetVal;
}

/**
* @brief  Executes converter's state machine IDLE STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_IDLE_Func(void)
{
  bool RetVal = true;

  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_Idle);                           ///DPC Bicolor LED SET to FSM_Idle

  if (DAB.StartUpCheck==StartUpCheck_Disabled){
    RetVal = true;
    DPC_FSM_State_Set(DPC_FSM_INIT);
    DPC_TO_Release(DPC_TO_IDLE);
  }
  else
  {
    if(DPC_TO_CheckState(DPC_TO_IDLE)==DPC_TO_TOOK){
      if(DAB.DPC_HvSourceStatus==OK_DC_SOURCE){
        //      if(DAB.DPC_LvLoadStatus==NO_LOAD){
        DPC_ACT_OutDisable();
        DPC_ACT_DAB_TpzPWM_PS_Update(&DAB.tDPC_PWM,0,0,0);                                                                       ///Trapezoidal tecnique modulator update
        if(DPC_TO_Set(DPC_TO_INIT,TO_INIT_Tick)==DPC_TO_OK){                                       ///TimeOut of Idle State of the Finite State Machine
          RetVal = true;
          DPC_FSM_State_Set(DPC_FSM_INIT);
          DPC_TO_Release(DPC_TO_IDLE);
        }
        else{
          RetVal = false;
          DPC_FSM_State_Set(DPC_FSM_STOP);
        }
        //      }

        //      else{
        //        RetVal = false;
        //        DPC_FLT_Faulterror_Set(DPC_FLT_ERROR_IDLE);
        //        DPC_FSM_State_Set(DPC_FSM_STOP);
        //      }
      }
    }
  }
  return RetVal;
}

/**
* @brief  Executes converter's state machine INIT STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_INIT_Func(void)
{
  bool RetVal = true;

  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_StartUp_inrush);                                                             ///DPC Bicolor LED SET to FSM_StartUp_inrush
  if (DAB.INRUSH_CTRL.INRUSH_Status==INRUSH_Disable){
    DPC_FSM_State_Set(DPC_FSM_START);
    DPC_TO_Release(DPC_TO_INIT);
    RetVal = true;
  }
  else {
    if(DPC_TO_CheckState(DPC_TO_INIT)==DPC_TO_TOOK){
      if(DAB.INRUSH_CTRL.INRUSH_Status==INRUSH_Start){
        DAB.INRUSH_CTRL.INRUSH_Status=INRUSH_Progress;
        DPC_ACT_OutEnable(&DAB.tDPC_PWM);
        DAB.PC_State=FSM_StartUp_inrush;
      }
      else if(DAB.INRUSH_CTRL.INRUSH_Status==INRUSH_Progress){
        DAB.INRUSH_CTRL.INRUSH_Status=DPC_LCT_DAB_InrushCheck(&DAB.DAB_ADC_RAW,&DAB.INRUSH_CTRL);            ///Inrush Check for the FSM
      }
      else if(DAB.INRUSH_CTRL.INRUSH_Status==INRUSH_Complete){
        if(DPC_TO_Set(DPC_TO_START,TO_START_Tick)==DPC_TO_OK){                                       ///TimeOut of Idle State of the Finite State Machine
          DPC_FSM_State_Set(DPC_FSM_START);
          DPC_TO_Release(DPC_TO_INIT);
        }
      }
      else if(DAB.INRUSH_CTRL.INRUSH_Status==INRUSH_Error){
        RetVal = false;
        DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_INR);
      }
    }
  }
  return RetVal;
}

/**
* @brief  Executes converter's state machine START STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_START_Func(void)
{
  bool RetVal = true;

  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_StartUp_burst);                                                      ///DPC Bicolor LED SET to FSM_StartUp_burst
    switch(DAB.BURST_CTRL.BURST_Status){
    case BURST_Start:
      DAB.PC_State=FSM_StartUp_burst;
      DAB.BURST_CTRL.BURST_Status=BURST_Progress;
      break;

    case BURST_Progress:
      DAB.BURST_CTRL.BURST_Status=DPC_LCT_DAB_BurstCheck(&DAB.DAB_ADC_RAW,&DAB.BURST_CTRL);        ///Future dev
      break;

    case BURST_Complete:
      if(DPC_TO_CheckState(DPC_TO_START)==DPC_TO_TOOK){
      DAB.PC_State=FSM_Run;
      DPC_FSM_State_Set(DPC_FSM_RUN);
      DPC_ACT_OutDisable();
      }
      break;

    case BURST_Error:
      RetVal = false;
      DPC_FSM_State_Set(DPC_FSM_STOP);
      DPC_FLT_Faulterror_Set(DPC_FLT_ERROR_BRS);
      DPC_FLT_Faulterror_Set(DPC_FLT_FAULT_BRS);
      break;

    case BURST_Disable:
      DAB.PC_State=FSM_Run;
      DAB.FSM_Run_State=Run_HV2LV_Mode;
      DPC_FSM_State_Set(DPC_FSM_RUN);
      DPC_ACT_OutEnable(&DAB.tDPC_PWM);
      break;
    case BURST_TIMEOUT:
	  break;
    case BURST_Run:
	  break;
  }

  return RetVal;
}

/**
* @brief  Executes converter's state machine RUN STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_RUN_Func(void)
{
  bool RetVal = true;

  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_Run);                                  ///DPC Bicolor LED SET to FSM_Run
  if(DPC_FLT_Faulterror_Check()){
    RetVal = false;
    DPC_FSM_State_Set(DPC_FSM_STOP);
  }
  else{
      DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_Run);                                  ///DPC Bicolor LED SET to FSM_Run
  }
  return RetVal;
}

/**
* @brief  Executes converter's state machine STOP STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_STOP_Func(void)
{
  bool RetVal = true;
  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_Stop);                                 ///DPC Bicolor LED SET to FSM_Stop
  DPC_FSM_State_Set(DPC_FSM_ERROR);
  return RetVal;
}

/**
* @brief  Executes converter's state machine ERR/FAUL STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_ERROR_Func(void)
{
  bool RetVal = true;
  DPC_FLT_FaultErrorList_t eError;
  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_Error);                                ///DPC Bicolor LED SET to FSM_Error
  eError = DPC_FLT_Faulterror_Check();
  if(eError & FAULT_MASK){
    //Fault detected
    DPC_ACT_OutDisable();                                                                 ///Safe: Disable PWM outputs if enabled
    RetVal = false;
    DPC_FSM_State_Set(DPC_FSM_FAULT);
  }
  else if(eError & ERROR_MASK){
    //put here the error recovery
    DPC_ACT_OutDisable();                                                                 ///Safe: Disable PWM outputs if enabled
    RetVal = false;
    DPC_FSM_State_Set(DPC_FSM_FAULT);
  }
  return RetVal;
}

/**
* @brief  Executes converter's state machine FAULT STate Function
* @param  None
* @retval true/false
*/
bool DPC_FSM_FAULT_Func(void)
{
  bool RetVal = true;
  DPC_MISC_BLED_Set(&DPC_BLED_TIM,DPC_BLED_CH,BLED_Fault);                                ///DPC Bicolor LED SET to FSM_Fault
  DPC_ACT_OutDisable();                                                                   ///Safe: Disable PWM outputs if enabled
  DPC_ACT_SetFault(&DAB.tDPC_PWM);
  DAB.PC_State=FSM_Fault;
  return RetVal;
}

/**
* @brief  None
* @param  None
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM2)
  {

    DPC_GPIO1_ON;                                                                                         /// DEBUGGER ######################################
    DATA_Acquisition_from_DMA(p_ADC1_Data,p_ADC2_Data);                                                           ///Pass ADC DMA Data in DATA LAYER
    ADC2Phy_DAB_ProcessData(&DAB.DPC_ADC_Conf,(uint32_t*)Read_DAB(),&DAB.DAB_ADC_PHY);                            /// Read Current AC from DATA Layer and pass it at CURRENT_ADC_AC_IN_NORM
    ADC2RAW_DAB_ProcessData((uint32_t*)Read_DAB(),&DAB.DAB_ADC_RAW);                                              /// Read Current AC from DATA Layer and pass it at CURRENT_ADC_AC_IN_NORM
    DAB.DPC_LvLoadStatus=DPC_MISC_DABLoadCheck(&DAB.DPC_Load,DAB.DC_Load_Limit);                                  ///Check Load state reading Load Voltage and curent (Val_Load_limit -> (uint32_t* p_ADC_Data_Sub,uint32_t V_cap_Limit,uint32_t I_No_load_Threshold,uint32_t I_Low_load_Threshold,uint32_t I_Over_load_Threshold))
    DAB.DPC_HvSourceStatus=DPC_MISC_DAB_HVDCSource_Check(&DAB.HVDC_Source,DAB.DC_Source_Limit,&DAB.DAB_ADC_RAW);  ///Check AC SOURCE state reading AC Voltage and curent

    switch(DAB.PC_State){
    case FSM_Run:                             ///__________FSM_Run________
      switch(DAB.FSM_Run_State){
      case Run_HV2LV_Mode:
        DPC_LPCNTRL_DAB_Mode(&DAB.pDAB_CTRL,&DAB.DAB_ADC_RAW);
        DPC_LPCNTRL_DAB_MaxPowerCalc(&DAB.pDAB_CTRL,DAB.DAB_ADC_PHY.VDAB1,DAB.DAB_ADC_PHY.VDAB2,DAB.pDAB_CTRL.trafoTurnRatio,DAB.pDAB_CTRL.swFreq_Hz,DAB.pDAB_CTRL.Inductance_H);
        DPC_LPCNTRL_DAB_TpzPWM_DutyTime_Calc(&DAB.pDAB_CTRL,DAB.DAB_ADC_PHY.VDAB1,DAB.DAB_ADC_PHY.VDAB2,DAB.pDAB_CTRL.trafoTurnRatio,DAB.pDAB_CTRL.PhSh_PowerCTRL_rad,DAB.pDAB_CTRL.swFreq_Hz);                                             /// Duty Time calculation for Trapezoidal PWM technique
        DPC_LPCNTRL_DAB_ModulatorSelector(&DAB.pDAB_CTRL);
        DPC_ACT_DAB_TpzPWM_PS_Update(&DAB.tDPC_PWM,DAB.pDAB_CTRL.Duty1_Act_CTRL_norm,DAB.pDAB_CTRL.Duty2_Act_CTRL_norm,DAB.pDAB_CTRL.PhSh_Act_CTRL_norm);                                                                       ///Trapezoidal tecnique modulator update
        break;
      case Run_LV2HV_Mode:
        DPC_ACT_OutDisable();
        break;
      case Run_Burst_Mode:
        DPC_ACT_OutDisable();
        break;
      case Run_Idle:
        DPC_ACT_OutDisable();
        DPC_LCT_DAB_Ctrl_Reset(&DAB.pDAB_CTRL);
        break;
      }
      break;
    case FSM_StartUp_inrush:               ///__________FSM_StartUp_inrush__________
      DPC_LPCNTRL_DAB_InrushMode(&DAB.pDAB_CTRL,&DAB.INRUSH_CTRL,&DAB.tDPC_PWM,&DAB.DAB_ADC_RAW);
      DPC_ACT_DAB_TpzPWM_PS_Update(&DAB.tDPC_PWM,DAB.pDAB_CTRL.Duty1_Act_CTRL_norm,DAB.pDAB_CTRL.Duty2_Act_CTRL_norm,DAB.pDAB_CTRL.PhSh_Act_CTRL_norm);                                                                       ///Trapezoidal tecnique modulator update
      break;
    case FSM_StartUp_burst:                ///__________FSM_StartUp_burst__________
      DPC_ACT_OutDisable();
      break;
    case FSM_Idle:                       ///FSM_Idle
      DPC_ACT_OutDisable();
      break;
    case FSM_Fault:                       ///FSM_Fault
      DPC_ACT_OutDisable();
      break;
    case FSM_Debug:                        ///__________FSM_Debug__________
      DPC_ACT_OutDisable();
      break;
    case FSM_Error:                        ///
      break;
    case FSM_Stop:                        ///
      break;
    }

    DPC_GPIO1_OFF;                                                                                         /// DEBUGGER ######################################
  }
  else if(htim->Instance == TIM3){
    DPC_TO_TimeoutMng();
//    if(pDAB_CTRL.Duty2_CTRL_MAN_norm<0)
//    {
//    pDAB_CTRL.Duty2_CTRL_MAN_norm=0.4;
//    }
//    else if(pDAB_CTRL.Duty2_CTRL_MAN_norm>=0)
//    {
//      pDAB_CTRL.Duty2_CTRL_MAN_norm=-0.4;
//    }
  }
  else if(htim->Instance == TIM6){
  DPC_LPCNTRL_DAB_Vramp(&DAB.pDAB_CTRL);
  DPC_LPCNTRL_DAB_Iramp(&DAB.pDAB_CTRL);
  DPC_LPCNTRL_DAB_Vset(&DAB.pDAB_CTRL,DAB.pDAB_CTRL.pDAB_VCTRL_SlewRate.fDAB_VDC_Ref_V,&DAB.DPC_ADC_Conf);
  DPC_LPCNTRL_DAB_Iset(&DAB.pDAB_CTRL,DAB.pDAB_CTRL.pDAB_ICTRL_SlewRate.fDAB_IDC_Ref_A,&DAB.DPC_ADC_Conf);

  }
  else if(htim->Instance == TIM7){
  DPC_DAB_UserMonitorConversion(&DAB);
#ifdef DEBUG_DAC
    Debug_DATA_DAC(DAB.DAC_CH);
#endif     ///DEBUG_DAC
  }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1){
//    DPC_LED1_ON;                                                                                       /// DEBUGGER ######################################
//    DPC_LED1_OFF;
  }
  else{
    DPC_LED2_ON;                                                                                       /// DEBUGGER ######################################
    DPC_LED2_OFF;
  }
}

/**
  * @brief  DPC_MISC_Analog_Start:
  * @param  TBD
  *
  * @retval none
  *
  * @note Function valid for STM32G4xx microconroller family
  */
void  DPC_MISC_Analog_Start(void){

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  HAL_ADC_Start_DMA(&hadc1,p_ADC1_Data,ADC1_CHs);                              ///HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* p_ADC1_Data, uint32_t Length)
  HAL_ADC_Start_DMA(&hadc2,p_ADC2_Data,ADC2_CHs);                              ///HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* p_ADC1_Data, uint32_t Length)

  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
  HAL_DAC_Start(&hdac2,DAC_CHANNEL_1);

  //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sine_wave_array, 32, DAC_ALIGN_12B_R);
  //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)sine_wave_array, 32, DAC_ALIGN_12B_R);
  //HAL_DAC_Start_DMA(&hdac2, DAC_CHANNEL_1, (uint32_t*)sine_wave_array, 32, DAC_ALIGN_12B_R);

}

/**
* @brief  Send select data from DATA Layer to DAC for debugging
* @param  DAC_CH_Sub: DAC_Channel_STRUCT that contain DAC setting.
*
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family
*/

void Debug_DATA_DAC(DAC_Channel_STRUCT DAC_CH_Sub){

  float DATA_DAC1,DATA_DAC2,DATA_DAC3;
  float DATA_D0,DATA_D1,DATA_D2,DATA_D3,DATA_D4,DATA_D5,DATA_D6,DATA_D7,DATA_D8,DATA_D9,DATA_D10,DATA_D11,DATA_D12,DATA_D13,DATA_D14,DATA_D15,DATA_D16,DATA_D17,DATA_D18,DATA_D19,DATA_D20,DATA_D21;

    DATA_D0=0;
    DATA_D1=0;
    DATA_D2=0;
    DATA_D3=0;
    DATA_D4=0;
    DATA_D5=0;
    DATA_D6=0;
    DATA_D7=0;
    DATA_D8=0;
    DATA_D9=0;
    DATA_D10=0;
    DATA_D11=0;
    DATA_D12=0;
    DATA_D13=0;
    DATA_D14=0;
    DATA_D15=0;
    DATA_D16=0;
    DATA_D17=0;
    DATA_D18=0;
    DATA_D19=0;
    DATA_D20=0;
    DATA_D21=0;

  switch(DAC_CH_Sub.CH1){
  case 0:DATA_DAC1=DATA_D0;break;
  case 1:DATA_DAC1=DATA_D1;break;
  case 2:DATA_DAC1=DATA_D2;break;
  case 3:DATA_DAC1=DATA_D3;break;
  case 4:DATA_DAC1=DATA_D4;break;
  case 5:DATA_DAC1=DATA_D5;break;
  case 6:DATA_DAC1=DATA_D6;break;
  case 7:DATA_DAC1=DATA_D7;break;
  case 8:DATA_DAC1=DATA_D8;break;
  case 9:DATA_DAC1=DATA_D9;break;
  case 10:DATA_DAC1=DATA_D10;break;
  case 11:DATA_DAC1=DATA_D11;break;
  case 12:DATA_DAC1=DATA_D12;break;
  case 13:DATA_DAC1=DATA_D13;break;
  case 14:DATA_DAC1=DATA_D14;break;
  case 15:DATA_DAC1=DATA_D15;break;
  case 16:DATA_DAC1=DATA_D16;break;
  case 17:DATA_DAC1=DATA_D17;break;
  case 18:DATA_DAC1=DATA_D18;break;
  case 19:DATA_DAC1=DATA_D19;break;
  case 20:DATA_DAC1=DATA_D20;break;
  case 21:DATA_DAC1=DATA_D21;break;
  }

  switch(DAC_CH_Sub.CH2){
  case 0:DATA_DAC2=DATA_D0;break;
  case 1:DATA_DAC2=DATA_D1;break;
  case 2:DATA_DAC2=DATA_D2;break;
  case 3:DATA_DAC2=DATA_D3;break;
  case 4:DATA_DAC2=DATA_D4;break;
  case 5:DATA_DAC2=DATA_D5;break;
  case 6:DATA_DAC2=DATA_D6;break;
  case 7:DATA_DAC2=DATA_D7;break;
  case 8:DATA_DAC2=DATA_D8;break;
  case 9:DATA_DAC2=DATA_D9;break;
  case 10:DATA_DAC2=DATA_D10;break;
  case 11:DATA_DAC2=DATA_D11;break;
  case 12:DATA_DAC2=DATA_D12;break;
  case 13:DATA_DAC2=DATA_D13;break;
  case 14:DATA_DAC2=DATA_D14;break;
  case 15:DATA_DAC2=DATA_D15;break;
  case 16:DATA_DAC2=DATA_D16;break;
  case 17:DATA_DAC2=DATA_D17;break;
  case 18:DATA_DAC2=DATA_D18;break;
  case 19:DATA_DAC2=DATA_D19;break;
  case 20:DATA_DAC2=DATA_D20;break;
  case 21:DATA_DAC2=DATA_D21;break;
  }

  switch(DAC_CH_Sub.CH3){
  case 0:DATA_DAC3=DATA_D0;break;
  case 1:DATA_DAC3=DATA_D1;break;
  case 2:DATA_DAC3=DATA_D2;break;
  case 3:DATA_DAC3=DATA_D3;break;
  case 4:DATA_DAC3=DATA_D4;break;
  case 5:DATA_DAC3=DATA_D5;break;
  case 6:DATA_DAC3=DATA_D6;break;
  case 7:DATA_DAC3=DATA_D7;break;
  case 8:DATA_DAC3=DATA_D8;break;
  case 9:DATA_DAC3=DATA_D9;break;
  case 10:DATA_DAC3=DATA_D10;break;
  case 11:DATA_DAC3=DATA_D11;break;
  case 12:DATA_DAC3=DATA_D12;break;
  case 13:DATA_DAC3=DATA_D13;break;
  case 14:DATA_DAC3=DATA_D14;break;
  case 15:DATA_DAC3=DATA_D15;break;
  case 16:DATA_DAC3=DATA_D16;break;
  case 17:DATA_DAC3=DATA_D17;break;
  case 18:DATA_DAC3=DATA_D18;break;
  case 19:DATA_DAC3=DATA_D19;break;
  case 20:DATA_DAC3=DATA_D20;break;
  case 21:DATA_DAC3=DATA_D21;break;
  }

  /// SetValue DAC________________________________________
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,(uint16_t)((DATA_DAC1*(float)DAC_CH_Sub.Gain_CH1)+DAC_CH_Sub.Bias_CH1)); //Set Value DAC1
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,(uint16_t)((DATA_DAC2*(float)DAC_CH_Sub.Gain_CH2)+DAC_CH_Sub.Bias_CH2)); //Set Value DAC2
  HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R,(uint16_t)((DATA_DAC3*(float)DAC_CH_Sub.Gain_CH3)+DAC_CH_Sub.Bias_CH3)); //Set Value DAC3
}

/**
* @brief  DPC_APP_FSM_Init
* @param  None
* @retval None
*/
void DPC_APP_FSM_Init(DAB_FSM_State_TypeDef* PC_State,DAB_FSM_State_TypeDef PC_State_Set,Run_State_TypeDef* Run_State,Run_State_TypeDef Run_State_Set)
{
    if(*PC_State!=FSM_Error)
    {
      *PC_State=PC_State_Set;
      *Run_State=Run_State_Set;
    }
}

/* USER CODE END 4 */

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
