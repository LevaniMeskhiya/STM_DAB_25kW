/**
  ******************************************************************************
  * @file    DPC_Actuator.h
  * @brief   This file contains the headers of the DPC_Actuator module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DPC_PWMCONVERTER_H
#define __DPC_PWMCONVERTER_H


/* Includes ------------------------------------------------------------------*/
#include "DPC_CommonData.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DPC_ACT_ADVTIM_PWMStart(void);
void DPC_ACT_DeadTimeSet(uint16_t uwDeadTimeSub);
void DPC_ACT_Send_Duty_SVM(DPC_ACT_PWM_t *tDPC_PWM_loc,float VA,float VB,float VC);
void DPC_ACT_Send_Burst_PWM(DPC_ACT_PWM_t *tDPC_PWM_loc,float BURST_A,float BURST_B,float BURST_C);
void DPC_ACT_3LTTC_SPWM_DutySet_Opt(float VA,float VB,float VC,uint32_t  PWM_PERIOD_COUNTER_INT);
void DPC_ACT_HRTIM_Stop(void);
void DPC_ACT_HRTIM_Start(void);
void DPC_ACT_HRTIM_Init(void);
void DPC_ACT_HRTIM_Set(uint32_t Period,float Duty);
void DPC_ACT_HRTIM_OutEnable(void);
void DPC_ACT_HRTIM_OutDisable(void);
void DPC_ACT_ADVTIM_OutDisable(void);
void DPC_ACT_ADVTIM_OutEnable(void);
void DPC_ACT_Set_HRTIM(float VA,float VB,float VC);
void DPC_ACT_SetFault(DPC_ACT_PWM_t *tDPC_PWM_loc);
void DPC_ACT_OutDisable(void);
void DPC_ACT_OutEnable(DPC_ACT_PWM_t *tDPC_PWM_loc);
void DPC_ACT_Start(void);
void DPC_ACT_Init(uint32_t  Burst_FreqSet,uint32_t  PWM_FreqSet,DPC_ACT_Status_t DPC_PWM_SET, DPC_ACT_PWM_t *tDPC_PWM_loc);  
void DPC_ACT_CPWM_PS_Init(DPC_ACT_PWM_t *tDPC_PWM_loc);
void DPC_ACT_CPWM_PS_Update(DPC_ACT_PWM_t *tDPC_PWM_loc,float PS);
void DPC_ACT_TpzPWM_PS_Init(DPC_ACT_PWM_t *tDPC_PWM_loc);
void DPC_ACT_DAB_TpzPWM_PS_Update(DPC_ACT_PWM_t *tDPC_PWM_loc,float D1, float D2,float PS_Duty);
void DPC_ACT_Conf_DeadTime(uint32_t Deadtime);
void DPC_ACT_DAB_Conf_DeadTime(uint32_t DT_Dab1,uint32_t DT_Dab2);
uint8_t DPC_ACT_Calc_DeadTime(float DT_TimeVal);
void DPC_ACT_Calc_DeadTimRange(float DT_TimeVal, float t_tim_ket_ck);
#endif /* __DPC_PWMCONVERTER_H */
