/**
  ******************************************************************************
  * @file    Loop_Ctrl.h
  * @brief   This file contains the headers of the Loop_Ctrl Module.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __DPC_LOOPCTRL_H
#define __DPC_LOOPCTRL_H

/* Includes ------------------------------------------------------------------*/

#ifdef STM32G474xx
  #include "stm32g4xx_hal.h"
#endif

#include "DPC_CommonData.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */




  






void DPC_LCT_VoltageDC_Control(DPC_LCT_VDC_Ctrl_t *pVOLTAGECTRL_sub, float *pId_ctrl_sub);
void DPC_LCT_CurrentDC_Control(DPC_LCT_IDC_Ctrl_t *CURRENTCTRL_sub, float *Out_ctrl_sub);
void DPC_LCT_DAB_Ctrl_Reset(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc);
void DPC_LCT_BurstCtrl_Init(DPC_ACT_Burst_t *BURST_t_local,FlagStatus Burst_Enable_loc,uint16_t Vref_Hist,uint16_t delta_Vref_hist,float I_DC_NoLoadLimit_A,float I_dc_LOW_LOAD_Limit_AMP_loc,float duty_local,float duty_no_load_local,DPC_ADC_Conf_t *pDPC_ADC_Conf);
DPC_ACT_BurstStatus_t DPC_LCT_DAB_BurstCheck(DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw,DPC_ACT_Burst_t *BURST_CTRL_f);
void DPC_CTRL_RELAY_ChangeState(Relay_Typedef *Relay_local);
DPC_ACT_InrushStatus_t DPC_LCT_DAB_InrushCheck(DAB_ADC_RAW_Struct_t* DAB_ADC_DCraw,DPC_LCT_InrushCtrl_t *INRUSH_CTRL_f);
void DPC_LCT_Inrush_Init(DPC_LCT_InrushCtrl_t *INRUSH_CTRL_f,uint16_t Vref_Hist_V,uint16_t DeltaHigh_VrefHist_V,uint16_t DeltaLow_VrefHist_V,float I_DC_NoLoadLimit_A,FlagStatus InrushEnable_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf);
void DPC_LPCNTRL_DAB_Init(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,DPC_LCT_DAB_CtrlState_t DAB_CTRL_State,uint16_t DAB_VDC_Ref_loc,uint16_t DAB_IDC_Ref_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf);
void DPC_LPCNTRL_DAB_Vset(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,float fDAB_VDC_Ref_V_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf);
void DPC_LPCNTRL_DAB_Iset(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,float fDAB_IDC_Ref_A_loc,DPC_ADC_Conf_t *pDPC_ADC_Conf);
void DPC_LPCNTRL_DAB_Vramp(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc);
void DPC_LPCNTRL_DAB_Iramp(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc);
void DPC_LPCNTRL_DAB_Mode(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc, DAB_ADC_RAW_Struct_t* DAB_ADC_DC_RAW_Sub);
void DPC_LPCNTRL_DAB_TpzPWM_DutyTime_Calc(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL, float VHV, float VLV, float n, float phi_rad, float fsw);
void DPC_LPCNTRL_DAB_MaxPowerCalc(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL, float VHV, float VLV, float n, float fsw, float L);
void DPC_LPCNTRL_DAB_ModulatorSelector(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL);
void DPC_LPCNTRL_DAB_InrushMode(DPC_LCT_DAB_Ctrl_t *pDAB_CTRL_loc,DPC_LCT_InrushCtrl_t *INRUSH_CTRL,DPC_ACT_PWM_t *tDPC_PWM_loc, DAB_ADC_RAW_Struct_t* DAB_ADC_DC_RAW);
#endif /*__DPC_LOOPCTR.H */