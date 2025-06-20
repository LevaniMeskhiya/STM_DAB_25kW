/**
  ******************************************************************************
  * @file    DPC_Timeout.h
  * @author  SRA Power Conversion Team
  * @brief   Header file of DPC Timeout module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "DPC_Timeout.h"


/** @addtogroup DPC
  * @{
  */

/** @defgroup TIMEOUT TIMEOUT
  * @brief DPC TIMEOUT module
  * @{
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup TIMEOUT_Private_Variables TIMEOUT Private Variables
  * @{
  */

/** the array of timeouts */
static DPC_TO_TimeoutData_t xaTimeoutList[DPC_TO_MAX_NUMBER];
/**
  * @}
  */

/* Exported function definitions -----------------------------------------------*/
/** @addtogroup TIMEOUT_Exported_Functions
  * @{
  */

/**
  * @brief  Initialization of the vector of timeouts.
  */
void DPC_TO_Init(void)
{
  for (uint8_t ubIndexElementt = 0; ubIndexElementt < DPC_TO_MAX_NUMBER; ubIndexElementt++)
  {
    xaTimeoutList[ubIndexElementt].eTimeoutState = DPC_TO_OFF;
    xaTimeoutList[ubIndexElementt].uwTimeoutCount = 0;
  }
}


/**
  * @brief      Check if the requested timeout is valid.
  * @param[in]  eTimeoutType The type of timeout.
  * @return     Whether the timeout is valid or not.
  */
static DPC_TO_ReturnState_t DPC_TO_CheckTimeoutType(DPC_TO_TimeoutTypes_t eTimeoutType)
{
	DPC_TO_ReturnState_t eReturnState = DPC_TO_ERR;

	if(eTimeoutType < DPC_TO_MAX_NUMBER)
	{
		eReturnState = DPC_TO_OK;
	}
	return eReturnState;
}


/**
  * @brief      Configure a timeout.
  * @param[in]  eTimeoutType    The type of timeout.
  * @param[in]  uwTimeoutValue  The value to count down from.
  * @return     Whether the timeout is set correctly.
  */
DPC_TO_ReturnState_t DPC_TO_Set(DPC_TO_TimeoutTypes_t eTimeoutType, uint32_t uwTimeoutValue)
{
  DPC_TO_ReturnState_t eReturnState = DPC_TO_ERR;
  
  if(DPC_TO_CheckTimeoutType(eTimeoutType) == DPC_TO_OK && xaTimeoutList[eTimeoutType].eTimeoutState == DPC_TO_OFF)
  {
    xaTimeoutList[eTimeoutType].eTimeoutState = DPC_TO_RUN;
    xaTimeoutList[eTimeoutType].uwTimeoutCount = uwTimeoutValue;
    eReturnState = DPC_TO_OK;
  }
return eReturnState;
}


/**
  * @brief      Get the state of a timeout.
  * @param[in]  eTimeoutType  The type of timeout.
  * @return     Timeout status.
  */
DPC_TO_TimeoutState_t DPC_TO_CheckState(DPC_TO_TimeoutTypes_t eTimeoutType)
{
  DPC_TO_TimeoutState_t timeoutState = DPC_TO_OFF;

  if(DPC_TO_CheckTimeoutType(eTimeoutType) == DPC_TO_OK)
  {
    timeoutState = xaTimeoutList[eTimeoutType].eTimeoutState;
  }
  return timeoutState;
}


/**
  * @brief      Release a timeout.
  * @param[in]  eTimeoutType  The type of timeout.
  * @return     Whether the timeout has been released or not.
  */
DPC_TO_ReturnState_t DPC_TO_Release(DPC_TO_TimeoutTypes_t eTimeoutType)
{
  DPC_TO_ReturnState_t eReturnState = DPC_TO_ERR;

  if(DPC_TO_CheckTimeoutType(eTimeoutType) == DPC_TO_OK)
  {
    xaTimeoutList[eTimeoutType].eTimeoutState = DPC_TO_OFF;
    eReturnState = DPC_TO_OK;
  }
  return eReturnState;
}


/**
  * @brief      Reload the counter of a running timeout.
  * @param[in]  eTimeoutType    The type of timeout.
  * @param[in]  uwTimeoutValue  The value to count down from.
  * @return     Whether the timeout was reloaded successfully or not.
  */
DPC_TO_ReturnState_t DPC_TO_Reload(DPC_TO_TimeoutTypes_t eTimeoutType, uint32_t uwTimeoutValue)
{
  DPC_TO_ReturnState_t eReturnState = DPC_TO_ERR;

  if(DPC_TO_CheckTimeoutType(eTimeoutType) == DPC_TO_OK && xaTimeoutList[eTimeoutType].eTimeoutState == DPC_TO_RUN)
  {
    xaTimeoutList[eTimeoutType].uwTimeoutCount = uwTimeoutValue;
    eReturnState = DPC_TO_OK;
  }
  return eReturnState;
}


/**
  * @brief  Timeouts management.
  */
void DPC_TO_TimeoutMng(void)
{
  for (uint8_t ubIndexElement = 0; ubIndexElement < DPC_TO_MAX_NUMBER; ubIndexElement++)
  {
    if(xaTimeoutList[ubIndexElement].eTimeoutState == DPC_TO_RUN)
    {
      if(xaTimeoutList[ubIndexElement].uwTimeoutCount > 0)
      {
        xaTimeoutList[ubIndexElement].uwTimeoutCount--;
      }
      if(xaTimeoutList[ubIndexElement].uwTimeoutCount == 0)
      {
        xaTimeoutList[ubIndexElement].eTimeoutState = DPC_TO_TOOK;
      }
    }
  }
}


/**
  * @}
  */

