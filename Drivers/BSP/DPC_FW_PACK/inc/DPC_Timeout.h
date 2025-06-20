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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DPC_TIMEOUT_H
#define DPC_TIMEOUT_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"                       //uint8_t

/** @addtogroup DPC
  * @{
  */

/** @addtogroup TIMEOUT
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TIMEOUT_Exported_Types TIMEOUT Exported Types
  * @{
  */

/**
  * @brief TIMEOUT boolean function return values
  */
typedef enum
{
  DPC_TO_ERR      = 0x0U,
  DPC_TO_OK
} DPC_TO_ReturnState_t;

/**
  * @brief Timeout states
  */
typedef enum
{
  DPC_TO_OFF      = 0x2U,
  DPC_TO_RUN,
  DPC_TO_TOOK
} DPC_TO_TimeoutState_t;

/**
  * @brief Timeout structure definition.
  */
typedef struct
{
  uint32_t uwTimeoutCount;              /*!< timeout counter */
  DPC_TO_TimeoutState_t eTimeoutState;  /*!< timeout state */
} DPC_TO_TimeoutData_t;

/**
  * @brief Timeout types.
  */
typedef enum
{
  DPC_TO_TELEM_1  = 0x00U,
  DPC_TO_TELEM_2  = 0x01U,
  DPC_TO_INDX1    = 0x02U,
  DPC_TO_INDX2    = 0x03U,
  DPC_TO_INDX3    = 0x04U,
  DPC_TO_INDX4    = 0x05U,
  DPC_TO_INDX5    = 0x06U,
  DPC_TO_ICP_1    = 0x07U,
  DPC_TO_ICP_2    = 0x08U
} DPC_TO_TimeoutTypes_t;

#define DPC_TO_MAX_NUMBER  0x09U  /*!< number of different types of timeout. */

/**
 * @}
 */

/* Exported functions ------------------------------------------------------- */
/** @defgroup TIMEOUT_Exported_Functions TIMEOUT Exported Functions
  * @{
  */

void DPC_TO_Init(void);
DPC_TO_ReturnState_t DPC_TO_Set(DPC_TO_TimeoutTypes_t eTimeoutType, uint32_t uwTimeoutValue);
DPC_TO_TimeoutState_t DPC_TO_CheckState(DPC_TO_TimeoutTypes_t eTimeoutType);
DPC_TO_ReturnState_t DPC_TO_Release(DPC_TO_TimeoutTypes_t eTimeoutType);
DPC_TO_ReturnState_t DPC_TO_Reload(DPC_TO_TimeoutTypes_t eTimeoutType, uint32_t uwTimeoutValue);
void DPC_TO_TimeoutMng(void);

/**
  * @}
  */

#endif /* DPC_TIMEOUT_H */
