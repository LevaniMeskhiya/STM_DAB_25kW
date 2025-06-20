/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : DPC_Application.h
  * @brief          : Header for DPC_Application.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DPC_APPLICATIO_H
#define __DPC_APPLICATIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DPC_CommonData.h"
#include "DPC_Application_Conf.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*!FSM*/
typedef enum
{
  FSM_Idle = 0,
  FSM_StartUp_inrush,
  FSM_StartUp_burst,
  FSM_Run,
  FSM_Fault,
  FSM_Error,
  FSM_Debug,
  FSM_Stop,
} DAB_FSM_State_TypeDef;

typedef enum
{
  Run_Idle = 0,
  Run_Burst_Mode,
  Run_HV2LV_Mode,
  Run_LV2HV_Mode
} Run_State_TypeDef;

  typedef enum
  {
    HV2LV_OPERATION,
    LV2HV_OPERATION,
    BIDI_OPERATION
  } DPC_DAB_InitMode_t;

  typedef enum
  {
    StartUpCheck_Disabled,
    StartUpCheck_Enabled,
  } StartUpCheck_TypeDef;

  typedef struct {
    /* DAB initialization struct types ------------------------------------------------------------*/
    DPC_DAB_InitMode_t InitMode;                                        /*!< Data struct that contain initialization data */
    /* DAB HighVoltage structs types ------------------------------------------------------------*/
    DPC_MISC_DCSourceLimit_t DC_Source_Limit;                           /*!< Data struct that contain HVDC Source limit threshold */
    DPC_MISC_DCSource_t HVDC_Source;                                    /*!< Data struct that contain HVDC Source data */
    DPC_MISC_PlugDCSourceStatus_t Status_DAB_Plug_HVDCSource;           /*!< Data struct that contain HVDC Source main status */
    DPC_MISC_DCSourceStatus_t DPC_HvSourceStatus;                       /*!< Data struct that contain HVDC Source main status  */
    /* DAB LowVoltage structs types ------------------------------------------------------------*/
    DPC_MISC_DCLoadLimit_t DC_Load_Limit;                               /*!< Data struct that contain LVDC Load limit threshold */
    DPC_MISC_DCLoad_t DPC_Load;                                         /*!< Data struct that contain LVDC Load data  */
    DPC_MISC_DCLoadStatus_t DPC_LvLoadStatus;                           /*!< Data struct that contain LVDC Load main status */
    /* DAB Sensing structs types ------------------------------------------------------------*/
    DPC_ADC_Conf_t DPC_ADC_Conf;                                        /*!< Data struct that contain ADC scaling factors */
    DAB_ADC_RAW_Struct_t DAB_ADC_RAW;                                   /*!< Data struct that contain ADC Raw data */
    DAB_ADC_PHY_Struct_t DAB_ADC_PHY;                                   /*!< Data struct that contain ADC cooked data */
    /* DAB State Machine structs types ------------------------------------------------------------*/
    StartUpCheck_TypeDef StartUpCheck;                                  /*!< DAB startup procedure*/
    DAB_FSM_State_TypeDef PC_State;                                     /*!< DAB Finite State Machine */
    Run_State_TypeDef FSM_Run_State;                                    /*!< Control Finite State Machine */
    /* DAB Control structs types ------------------------------------------------------------*/
    DPC_LCT_InrushCtrl_t INRUSH_CTRL;                                   /*!< Data struct that contain Inrush control data  */
    DPC_ACT_Burst_t BURST_CTRL;                                         /*!< Data struct that contain Burst control data  */
    DPC_LCT_DAB_Ctrl_t pDAB_CTRL;                                       /*!< DAB control supervisor data struct */
    /* DAB Actuation structs types ------------------------------------------------------------*/
    DPC_ACT_PWM_t tDPC_PWM;                                             /*!< DAB actuation data struct*/
    /* DAB Debug/Telemetry/MISC structs types ------------------------------------------------------------*/
    DAC_Channel_STRUCT DAC_CH;                                          /*!< DAC Channel Struct Declaration*/
  }DPC_DAB_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

  void DPC_APPLICATION_Init(void);
  void DPC_APPLICATION_Process(void);
  void DPC_TTC_Init(DPC_DAB_t *DPC_DAB, DPC_DAB_InitMode_t DPC_DAB_InitMode);
  void DPC_DAB_UserMonitorConversion(DPC_DAB_t *DAB);

  void Debug_DATA_DAC(DAC_Channel_STRUCT DAC_CH_Sub);
  void DPC_MISC_Analog_Start(void);
  void DPC_APP_FSM_Init(DAB_FSM_State_TypeDef *PC_State,DAB_FSM_State_TypeDef PC_State_Set,Run_State_TypeDef *Run_State,Run_State_TypeDef Run_State_Set);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __DPC_APPICATION_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
