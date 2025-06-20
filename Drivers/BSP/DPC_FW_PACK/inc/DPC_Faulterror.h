/**
  ******************************************************************************
  * @file    DPC_Faulterror.h
  * @brief   This file contains the headers of the transform module.
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
#ifndef __DPC_FAULTERROR_H
#define __DPC_FAULTERROR_H


/* Includes ------------------------------------------------------------------*/
#include "DPC_Actuator.h"
/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Type of control activated OPEN - INNER - OUTER
  */
typedef enum
{
  DPC_FLT_NO_FAULTERROR =0,
  DPC_FLT_FAULT_TRUE,
  DPC_FLT_ERROR_TRUE,
  DPC_FLT_NOT_ERASABLE,
  DPC_FLT_ERASE_ERROR_LIST_NOT_EMPTY,
  DPC_FLT_ERASE_OK,
 } DPC_FAULTERROR_STATUS_t;

typedef enum
{
  DPC_FLT_NO_FAULT =0,
  DPC_FLT_ERROR_ALL             = 0xFFFF0000,           //ERROR_ALL 
  DPC_FLT_FAULT_ALL             = 0x0000FFFF,           //FAULT ALL     
  DPC_FLT_FAULT_OCL             = 0x00000001,           //OVER CURRENT LOAD
  DPC_FLT_FAULT_OVL             = 0x00000002,           //OVER VOLTAGE LOAD
  DPC_FLT_FAULT_OVC             = 0x00000004,           //OVER VOLTAGE CAPACITOR
  DPC_FLT_FAULT_OCS             = 0x00000008,           //OVER CURRENT SOURCE
  DPC_FLT_FAULT_OVS             = 0x00000010,           //OVER VOLTAGE SOURCE
  DPC_FLT_FAULT_INR             = 0x00000020,           //INRUSH
  DPC_FLT_FAULT_BRS             = 0x00000040,           //BURST
  DPC_FLT_FAULT_PLL_OR          = 0x00000080,           //PLL Out of Range  
  DPC_FLT_FAULT_PFC_UVLO        = 0x00000100,           //FAULT Under-Voltage-AC
  DPC_FLT_FAULT_IDLE            = 0x00000200,           //FAULT_IDLE
  DPC_FLT_FAULT_GEN             = 0x00000400,           //TBD
  DPC_FLT_FAULT_012             = 0x00000800,           //TBD
  DPC_FLT_FAULT_013             = 0x00001000,           //TBD
  DPC_FLT_FAULT_014             = 0x00002000,           //TBD
  DPC_FLT_FAULT_015             = 0x00004000,           //TBD
  DPC_FLT_FAULT_MAN             = 0x00008000,           //FORCE FAULT by manual  
  DPC_FLT_ERROR_PLL             = 0x00010000,           //ERROR PLL
  DPC_FLT_ERROR_IDLE            = 0x00020000,           //ERROR IDLE
  DPC_FLT_ERROR_START_INRS      = 0x00040000,           //ERROR START-UP INRUSH
  DPC_FLT_ERROR_FSM             = 0x00080000,           //ERROR in Finite State Machine
  DPC_FLT_ERROR_PFC_UVLO        = 0x00100000,           //ERROR AC UnderVoltage during PFC mode
  DPC_FLT_ERROR_BRS             = 0x00200000,           //TBD
  DPC_FLT_ERROR_AC_UV           = 0x00400000,           //ERROR Under-Voltage-AC
  DPC_FLT_ERROR_PLL_OR          = 0x00800000,           //ERROR PLL Out of Range
  DPC_FLT_ERROR_PFC_RUN         = 0x01000000,           //DPC_FLT_ERROR_PFC_RUN
  DPC_FLT_ERROR_AC_UVLO         = 0x02000000,           //ERROR Under-Voltage-Lockout AC
  DPC_FLT_ERROR_AC_OFF          = 0x04000000,           //ERROR No AC 
  DPC_FLT_ERROR_PFC             = 0x08000000,           //ERROR during PFC mode
  DPC_FLT_ERROR_PFC_ERRSEQ      = 0x10000000,           //ERROR 3Phase sequence connection
  DPC_FLT_ERROR_DAB             = 0x20000000,           //TBD  
  DPC_FLT_ERROR_015             = 0x40000000,           //TBD
  DPC_FLT_ERROR_016             = 0x80000000,           //TBD  

} DPC_FLT_FaultErrorList_t;


/* Exported constants --------------------------------------------------------*/
#define FAULT_MASK      0x0000FFFF
#define ERROR_MASK      0xFFFF0000
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
DPC_FLT_FaultErrorList_t DPC_FLT_Faulterror_Check(void);
void DPC_FLT_Faulterror_Set(DPC_FLT_FaultErrorList_t eFaulterror);
DPC_FAULTERROR_STATUS_t DPC_FLT_Error_Reset(DPC_FLT_FaultErrorList_t eError);


#endif /* __DPC_FAULTERROR_H */
