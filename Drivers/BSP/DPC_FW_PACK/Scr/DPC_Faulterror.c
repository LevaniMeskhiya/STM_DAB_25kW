/**
  ******************************************************************************
  * @file           : DPC_Faulterror.c
  * @brief          : Fault and Error  modulation management
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
/* Includes ------------------------------------------------------------------*/

#ifdef STM32G474xx
  #include "stm32g4xx_hal.h"
#endif
#include "DPC_Faulterror.h"




/* external variables ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint32_t uwFaultErrorVector;

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  DPC_FLT_Faulterror_Check: Check if an error or fault was occurs 
  *         
  * @param None  
  *
  * @retval DPC_FLT_FaultErrorList_t: error or fault occurs, highest priority
  *
  * @note Function valid for STM32G4xx and STM32F74x microcontroller family   
  */
DPC_FLT_FaultErrorList_t DPC_FLT_Faulterror_Check(void){

DPC_FLT_FaultErrorList_t uwFaErVecLocal=DPC_FLT_NO_FAULT;

if(uwFaultErrorVector != DPC_FLT_NO_FAULT){

  if(uwFaultErrorVector & DPC_FLT_FAULT_OCL){
    uwFaErVecLocal = DPC_FLT_FAULT_OCL;
  }
  else if(uwFaultErrorVector & DPC_FLT_FAULT_OVL){
    uwFaErVecLocal = DPC_FLT_FAULT_OVL;
  }
  else if(uwFaultErrorVector & DPC_FLT_FAULT_OVC){
    uwFaErVecLocal = DPC_FLT_FAULT_OVC;
  }
  else if(uwFaultErrorVector & DPC_FLT_FAULT_OCS){
    uwFaErVecLocal = DPC_FLT_FAULT_OCS;
  }
  else if(uwFaultErrorVector & DPC_FLT_FAULT_OVS){
    uwFaErVecLocal = DPC_FLT_FAULT_OVS;
 }
  else if(uwFaultErrorVector & DPC_FLT_FAULT_INR){
    uwFaErVecLocal = DPC_FLT_FAULT_INR;
  }
  else if(uwFaultErrorVector & DPC_FLT_FAULT_BRS){
    uwFaErVecLocal = DPC_FLT_FAULT_BRS;
  }
  else if(uwFaultErrorVector & DPC_FLT_FAULT_PLL_OR){
    uwFaErVecLocal = DPC_FLT_FAULT_PLL_OR;
  }  
  else if(uwFaultErrorVector & DPC_FLT_FAULT_PFC_UVLO){
    uwFaErVecLocal = DPC_FLT_FAULT_PFC_UVLO;
  }  
  else if(uwFaultErrorVector & DPC_FLT_FAULT_IDLE){
    uwFaErVecLocal = DPC_FLT_FAULT_IDLE;
  }  
  else if(uwFaultErrorVector & DPC_FLT_FAULT_GEN){
    uwFaErVecLocal = DPC_FLT_FAULT_GEN;
  }  
  else if(uwFaultErrorVector & DPC_FLT_FAULT_012){
    uwFaErVecLocal = DPC_FLT_FAULT_012;
  }  
  else if(uwFaultErrorVector & DPC_FLT_FAULT_013){
    uwFaErVecLocal = DPC_FLT_FAULT_013;
  }  
  else if(uwFaultErrorVector & DPC_FLT_FAULT_014){
    uwFaErVecLocal = DPC_FLT_FAULT_014;
  }    
  else if(uwFaultErrorVector & DPC_FLT_FAULT_015){
    uwFaErVecLocal = DPC_FLT_FAULT_015;
  }    
  else if(uwFaultErrorVector & DPC_FLT_FAULT_MAN){
    uwFaErVecLocal = DPC_FLT_FAULT_MAN;
  }      
  // ADD new Fault Index
  else if(uwFaultErrorVector & DPC_FLT_ERROR_PLL){
    uwFaErVecLocal = DPC_FLT_ERROR_PLL;
  }  
  else if(uwFaultErrorVector & DPC_FLT_ERROR_IDLE){
    uwFaErVecLocal = DPC_FLT_ERROR_IDLE;
  }  
  else if(uwFaultErrorVector & DPC_FLT_ERROR_START_INRS){
    uwFaErVecLocal = DPC_FLT_ERROR_START_INRS;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_FSM){
    uwFaErVecLocal = DPC_FLT_ERROR_FSM;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC_UVLO){
    uwFaErVecLocal = DPC_FLT_ERROR_PFC_UVLO;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_BRS){
    uwFaErVecLocal = DPC_FLT_ERROR_BRS;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_AC_UV){
    uwFaErVecLocal = DPC_FLT_ERROR_AC_UV;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_PLL_OR){
    uwFaErVecLocal = DPC_FLT_ERROR_PLL_OR;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC_RUN){
    uwFaErVecLocal = DPC_FLT_ERROR_PFC_RUN;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_AC_UVLO){
    uwFaErVecLocal = DPC_FLT_ERROR_AC_UVLO;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_AC_OFF){
    uwFaErVecLocal = DPC_FLT_ERROR_AC_OFF;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC){
    uwFaErVecLocal = DPC_FLT_ERROR_PFC;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC_ERRSEQ){
    uwFaErVecLocal = DPC_FLT_ERROR_PFC_ERRSEQ;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_DAB){
    uwFaErVecLocal = DPC_FLT_ERROR_DAB;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_015){
    uwFaErVecLocal = DPC_FLT_ERROR_015;
  }
  else if(uwFaultErrorVector & DPC_FLT_ERROR_016){
    uwFaErVecLocal = DPC_FLT_ERROR_016;
  }  
  // ADD new Error Index
  
}

return uwFaErVecLocal;  
}




/**
  * @brief  DPC_FLT_Faulterror_Set: Set the Fault/error occurs, the PWM output is automatically disabled.
  *         
  * @param eFaulterror: Fault or error occurs  
  *
  * @retval None
  *
  * @note Function valid for STM32G4xx and STM32F74x microcontroller family   
  */
void DPC_FLT_Faulterror_Set(DPC_FLT_FaultErrorList_t eFaulterror){
  uwFaultErrorVector |= eFaulterror;                                                    /*!< Set fault/error in the fault/error vector*/
}



/**
  * @brief  DPC_FLT_Error_Reset: Reset the error occurs 
  *         
  * @param eError: Error to reset.  
  *
  * @retval DPC_FLT_FaultErrorList_t: error or fault occurs, highest priority
  *
  * @note Function valid for STM32G4xx and STM32F74x microcontroller family   
  */
DPC_FAULTERROR_STATUS_t DPC_FLT_Error_Reset(DPC_FLT_FaultErrorList_t eError){

DPC_FAULTERROR_STATUS_t uwErVecLocal=DPC_FLT_NO_FAULTERROR;

if(uwFaultErrorVector & 0x0000FFFF){
  uwErVecLocal = DPC_FLT_NOT_ERASABLE;
}
else{
   if(uwFaultErrorVector & DPC_FLT_ERROR_PLL){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_PLL;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_IDLE){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_IDLE;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }            
    else if(uwFaultErrorVector & DPC_FLT_ERROR_START_INRS){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_START_INRS;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }           
    else if(uwFaultErrorVector & DPC_FLT_ERROR_FSM){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_FSM;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC_UVLO){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_PFC_UVLO;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_BRS){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_BRS;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_AC_UV){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_AC_UV;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_PLL_OR){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_PLL_OR;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC_RUN){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_PFC_RUN;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_AC_UVLO){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_AC_UVLO;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_AC_OFF){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_AC_OFF;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_PFC;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_PFC_ERRSEQ){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_PFC_ERRSEQ;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_DAB){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_DAB;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_015){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_015;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
    else if(uwFaultErrorVector & DPC_FLT_ERROR_016){
      uwFaultErrorVector &= !(uint32_t)DPC_FLT_ERROR_016;
      uwErVecLocal = DPC_FLT_ERASE_OK;
    }
   if(uwFaultErrorVector | 0x00000000){
    uwErVecLocal = DPC_FLT_ERASE_ERROR_LIST_NOT_EMPTY;
  }
}
return uwErVecLocal;  
}





















