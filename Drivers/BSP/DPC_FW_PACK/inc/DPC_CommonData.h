/**
  ******************************************************************************
  * @file    DPC_Datacollector.h
  * @brief   This file contains the headers of the Database.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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

#ifndef __DPC_DATACOLLECTOR_H
#define __DPC_DATACOLLECTOR_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif
#include "DPC_Lib_Conf.h"


/* Exported types ------------------------------------------------------------*/
/****** DPC_CommonData.h section Start *****/


//DC Side
typedef struct{
  float Vdc_pos;
  float Vdc_neg;
  float Vdc_tot;
}DPC_CDT_VOLT_DC_ADC_NORM_t;

typedef struct{
  uint32_t Vdc_pos;
  uint32_t Vdc_neg;
  uint32_t Vdc_tot;
}VoltageDC_ADC_RAW_Struct;

typedef struct{  
  uint32_t Vdc_pos;
  uint32_t Vdc_neg;
}DPC_CDT_VOLT_DC_ADC_t;

/****** DPC_CommonData.h section End *****/

typedef struct{  
  uint32_t IDC_adc;
}DPC_CDT_CURR_DC_ADC_t;
/****** DPC_Actuator.h section Start *****/
/**
  * @brief  Inrush mode status
  */

typedef enum
{
  INRUSH_Start    = 0x00U,
  INRUSH_Complete = 0x01U,
  INRUSH_Error    = 0x02U,
  INRUSH_Progress = 0x03U,
  INRUSH_TIMEOUT  = 0x04U,
  INRUSH_Disable = 0x05U
} DPC_ACT_InrushStatus_t;


/**
  * @brief  Burst mode status
  */
typedef enum
{
  BURST_Start    = 0x00U,
  BURST_Complete = 0x01U,
  BURST_Error    = 0x02U,
  BURST_Progress = 0x03U,
  BURST_TIMEOUT  = 0x04U,
  BURST_Disable  = 0x05U,
  BURST_Run      = 0x06U  
} DPC_ACT_BurstStatus_t;

typedef struct{
  float IDC_adc;
}DPC_CDT_CURR_DC_ADC_NORM_t;


//Reference Frame 
typedef struct{  
  float axA;
  float axB;
  float axC;
}DPC_RFT_ABC_t;

/**
  * @brief  Burst task status
  */
typedef enum
{
  Task_Idle    = 0x00U,
  Task_Active  = 0x01U,
} DPC_ACT_BurstTask_t;
   
  
/**
  * @brief  Burst mode handler
  */
typedef struct {
uint16_t Vout_load;
uint16_t Vref_hist;
uint16_t delta_Vref_hist;
uint16_t Vout_max;                                              /*!< Histeresis higher ouput DC voltage Thrueshold expressed in Bits */
uint16_t Vout_min;                                              /*!< Histeresis lower ouput DC voltage Thrueshold expressed in Bits*/
FlagStatus BURST_PACKAGE;
FlagStatus BURST_IN_RANGE;
float Duty_noload;
float Duty_lowload;
float Duty_Limit;
FlagStatus Burst_Enable;
uint16_t Iout_no_load_threshold;
uint16_t Iout_low_load_threshold;
uint16_t uI_load_Burst;
float Burst_Duty;
DPC_ACT_BurstTask_t Burst_Task;
DPC_ACT_BurstStatus_t BURST_Status;                              /*!< Data struct that contain Inrush control status  */ 
}DPC_ACT_Burst_t;



/** @defgroup ACT_DPC_STATUS_DEFINE  Actuator - Actuator main status
  * @{
  */
typedef enum
{
  PWM_Safe        = 0x00U,
  PWM_Armed       = 0x01U,
  PWM_Error       = 0x02U,
  PWM_Fault       = 0x03U,
  PWM_Stop        = 0x04U,    
} DPC_ACT_Status_t;



/**
  * @brief  Dynamic Actuator structure definition
  */
typedef struct {
DPC_ACT_Status_t DPC_PWM_Status;           /*!< Represent the main status of the Actuation of the Digital Power Converter
                                               This parameter can be a value of @ref ACT_DPC_STATUS_DEFINE */
uint32_t dutyMaxLim;
uint32_t dutyMinLim;
uint32_t PWM_Period;
uint32_t BURST_PWM_Period;
}DPC_ACT_PWM_t;

/****** DPC_Actuator.h section End *****/

/****** DPC_ADC.h section Start *****/
/**
  * @brief  Digital Power Converter - ADC handler
  */
typedef struct {
float fGVac;
float fInvGVac;
float fBVac;
float fGIac;
float fInvGIac;
float fBIac;
float fGVdc;
float fInvGVdc;
float fBVdc;
float fGIdc;
float fInvGIdc;
float fBIdc;
FlagStatus DPC_ADC_Conf_Complete;
}DPC_ADC_Conf_t;



/**
  * @brief  Digital Power Converter - ADC handler
  */
typedef struct {
float fGVac;
float fInvGVac;
float fBVac;
float fGIac;
float fInvGIac;
float fBIac;
float fGVdc;
float fInvGVdc;
float fBVdc;
float fGIdc;
float fInvGIdc;
float fBIdc;
float fGNTCac;                   /* Gain NTC AC */
float fInvGNTCac;                /* Invers of Gain NTC AC */
float fBNTCac;                   /* Bias NTC AC */
FlagStatus DPC_ADC_Conf_Complete;
}DPC_DAB_ADC_Conf_t;


typedef struct{
  uint32_t phA_adc;
  uint32_t phB_adc;
  uint32_t phC_adc;
}AC_ADC_Struct;


typedef union{
  AC_ADC_Struct VAC;
  AC_ADC_Struct IAC;
  DPC_CDT_VOLT_DC_ADC_t VDC;
} RAW_ADC_UNION;


typedef struct{
  uint32_t VDAB1;
  uint32_t VDAB2;
  uint32_t IDAB1;
  uint32_t IDAB2;
  uint32_t ILK;
} DAB_ADC_Value_Struct;

typedef struct{
  float VDAB1;
  float VDAB2;
  float IDAB1;
  float IDAB2;
  float ILK;
} DAB_ADC_PHY_Struct_t;


typedef struct{
  int32_t VDAB1;
  int32_t VDAB2;
  int32_t IDAB1;
  int32_t IDAB2;
  int32_t ILK;
} DAB_ADC_RAW_Struct_t;

/****** DPC_DPC_ADC.h section End *****/

/****** DPC_Pid.h section Start *****/

/* Exported types ------------------------------------------------------------*/
typedef struct {
  float Ref;
  float Feed;
  float Kp; 
  float Ki;
  float Ts;
  float Integral;
  float PIout;
  float PIout_sat;
  float PIsat_up;
  float PIsat_down;
  float error;
  float Integralout;
  FlagStatus resetPI;  
  float k0;
  float k1;
  float Antiwindup_Term;
  FlagStatus satPI_toggle;
  FlagStatus antiwindPI_toggle;
  float Antiwindup_Gain;
  float resetValue;

  uint8_t PI_Q_FxP;
  int32_t Ref_s32;  
  int32_t Feed_s32;   
  int32_t Integral_s32;
  int32_t PIout_s32;
  int32_t PIout_sat_s32;
  int32_t PIsat_up_s32;
  int32_t PIsat_down_s32;  
  int32_t error_s32;
  int32_t Integralout_s32;  
  uint32_t k0_s32;  
  uint32_t k1_s32;
  int32_t Antiwindup_Term_s32;
  int32_t Antiwindup_Gain_s32;
  int32_t resetValue_s32;  
}DPC_PI_t;

/****** DPC_Pid.h section End *****/

/****** DPC_Loopctrl.h section Start *****/

/**
  * @brief  Relays Structure definition
  */
  typedef struct {
    FlagStatus RELAY_INRSH_State;
    FlagStatus RELAY_GRID_A_State;
    FlagStatus RELAY_GRID_B_State;
    FlagStatus RELAY_GRID_C_State;
    FlagStatus FAN_State;    
  }
  Relay_Typedef;
typedef struct{
  uint32_t Vdc_pos;
  uint32_t Vdc_neg;
  uint32_t Vdc_tot;
}DAB_ADC_NORM_Struct_t;


/**
  * @brief  DC Voltage Control Structure definition
  */
typedef struct{
  float Vdc_ref;
  float Vdc_feed;
  float Id_ctrl;
  DPC_PI_t pPI_VDC_CTRL;
}DPC_LCT_VDC_Ctrl_t;

/**
  * @brief  DC Current Control Structure definition
  */
typedef struct{
  float Idc_ref;
  float Idc_feed;
  float Out_ctrl;
  DPC_PI_t pPI_IDC_CTRL;
}DPC_LCT_IDC_Ctrl_t;  



/**
  * @brief  Inrush Current Control Structure definition
  */
typedef struct {
uint16_t Vout_load;                                             /*!< Output DC voltage expressed in Bits*/     
uint16_t Vref_hist;                                             /*!< Histeresis ouput DC voltage Thrueshold expressed in Bits*/
uint16_t deltahigh_Vref_hist;                                   /*!< Delta high histeresis ouput DC voltage Thrueshold expressed in Bits*/
uint16_t deltalow_Vref_hist;                                    /*!< Delta low histeresis ouput DC voltage Thrueshold expressed in Bits*/
uint16_t Vout_max;                                              /*!< Histeresis higher ouput DC voltage Thrueshold expressed in Bits */
uint16_t Vout_min;                                              /*!< Histeresis lower ouput DC voltage Thrueshold expressed in Bits*/
FlagStatus InrushEnable;                                        /*!< Inrush Enabled flag*/
uint16_t Iout_load_threshold;                                   /*!< Ouput DC current Thrueshold expressed in Bits*/
uint16_t I_load_Inrush;                                         /*!< */
DPC_ACT_InrushStatus_t INRUSH_Status;
}DPC_LCT_InrushCtrl_t;


/**
  * @brief  Type of control activated OPEN - INNER - OUTER
  */
typedef enum
{
  DAB_OPEN_LOOP = 0,
  DAB_CURRENT_LOOP,
  DAB_VOLTAGE_LOOP,
 } DPC_LCT_DAB_CtrlState_t;

    
/**
  * @brief  
  */
typedef enum{
  SPWM = 0,
  TPZ,
  TRG,
  MAN,
}DABModTecnique_TypeDef;

 

/**
  * @brief  
  */
typedef struct{
  float TpzDuty1;
  float TpzDuty2;
  float TpzD1_time;
  float TpzD2_time;
  float TpzOmega_1_rad;
  float TpzOmega_2_rad;
}DAB_ACT_t;

/**
  * @brief  
  */
typedef struct{
  float fDAB_VDC_Ref_V;
  float fDAB_VDC_RefPrev_V;
  float fDAB_VDC_RefNext_V; 
  uint16_t DAB_VDC_Ref_BITs;
  uint16_t DAB_VDC_RefPrev_BITs;
  uint16_t DAB_VDC_RefNext_BITs;    
}DAB_VCTRL_SlewRate_t;

/**
  * @brief  
  */
typedef struct{
  float fDAB_IDC_Ref_A;
  float fDAB_IDC_RefPrev_A;
  float fDAB_IDC_RefNext_A; 
  uint16_t DAB_IDC_Ref_BITs;
  uint16_t DAB_IDC_RefPrev_BITs;
  uint16_t DAB_IDC_RefNext_BITs;    
}DAB_ICTRL_SlewRate_t;
/**
  * @brief  
  */
typedef struct{
/* HW parametrs --------------------------------------------------------*/
  float Inductance_H;                           /*!< Equivalent series Inductance value */                                  
  float trafoTurnRatio;                         /*!< Transformer ratio */   
/* Power specs ---------------------------------------------------------*/ 
  float phi_rad_max_trian_rad;                  /*!< Theoretical Max power transger with triangular modulation*/   
  float phi_rad_max_trap_rad;                   /*!< Theoretical Max power transger with trapezodal  modulation*/
  float phi_rad_max_single_rad;                 /*!< Theoretical Max power transger with Conventional modulation*/   
  float Pmax_sps_W;                             /*!< Max power at SPS modulation*/   
  float Pmax_tpz_W;                             /*!< Max power at TPZ modulation*/
  float Pmax_tri_W;                             /*!< Max power at Tri modulation*/   
/* Control  ------------------------------------------------------------*/
  DPC_LCT_DAB_CtrlState_t DAB_CTRL_State;       /*!< DAB Control mode selection */   
  DPC_LCT_VDC_Ctrl_t VOLTAGECTRL;               /*!< Voltage control handler */   
  DPC_LCT_IDC_Ctrl_t CURRENTCTRL;               /*!< Current control handler */
  FlagStatus VdcCTRL_Reset;                     /*!< Voltage control reset flag */
  FlagStatus IdcCTRL_Reset;                     /*!< Current control reset flag */
  DAB_VCTRL_SlewRate_t pDAB_VCTRL_SlewRate;     /*!< Voltage Control Slew Rate maganer Handler*/
  DAB_ICTRL_SlewRate_t pDAB_ICTRL_SlewRate;     /*!< Current Control Slew Rate maganer Handler*/
  float fDAB_INRSH_RefAngleRef_norm;            /*!< Inrush ctrl actual value */   
  float fDAB_INRSH_RefAnglePrev_norm;           /*!< Inrush ctrl Previous value */    
  float fDAB_INRSH_RefAngleNext_norm;           /*!< Inrush ctrl Next value */    
/* HRTIM param ---------------------------------------------------------*/  
  float swFreq_Hz;                              /*!< Switching frequency */   
  float swPeriod_s;                             /*!< Switching period*/   
  uint32_t DPC_DTG_DAB1_bin;                    /*!< HV side DT register configurator value*/   
  uint32_t DPC_DTG_DAB2_bin;                    /*!< LV side DT register configurator value*/    
/* Phase Shift   -------------------------------------------------------*/  
  float PhSh_PowerCTRL_norm;                    /*!< Control Phase shift [Expressed in normalized form]*/    
  float PhSh_PowerCTRL_rad;                     /*!< Control Phase shift [Expressed in radiants]*/ 
  float PhSh_Act_CTRL_norm;                     /*!< Actual Phase shift [Expressed in normalized form]*/   
  float PhSh_Act_CTRL_rad;                      /*!< Actual Phase shift [Expressed in radiants]*/  
  float PhSh_Act_CTRL_deg;                      /*!< Actual Phase shift [Expressed in angular degree]*/    
  float Duty1_Act_CTRL_norm;                    /*!< Actual Phase shift of HV side bridge [Expressed in normalized form]*/     
  float Duty2_Act_CTRL_norm;                    /*!< Actual Phase shift of LV side bridge [Expressed in normalized form]*/     
 /* Phase Drift   -------------------------------------------------------*/  
  float PhaseDriftComp;                         /*!< Phase drift compenstion term */     
  float PhaseDriftCompOld;                      /*!< Phase drift compenstion prev term*/
/* Actuation   ---------------------------------------------------------*/  
  DABModTecnique_TypeDef ModTecnique;           /*!< Modulation tecnique selector */   
  DAB_ACT_t DAB_ACT;                            /*!< DAB PWM actuation handler*/   
/* Debug mode  ---------------------------------------------------------*/
  float PhSh_CTRL_MAN_norm;                     /*!< Debug - Manual Phase shift [Expressed in normalized form]*/   
  float Duty1_CTRL_MAN_norm;                    /*!< Debug - Phase shift of HV side bridge [Expressed in normalized form]*/   
  float Duty2_CTRL_MAN_norm;                    /*!< Debug - Phase shift of LV side bridge [Expressed in normalized form]*/
  float TPZ_comp;
}DPC_LCT_DAB_Ctrl_t;
/****** DPC_Loopctrl.h section End *****/

/****** DPC_Math.h section Start *****/

typedef struct {
  float Ts;
  float Integral_prev;
  float Integral_in;
  float Integralout;
  FlagStatus resetINTEGRATOR;  

  uint32_t Ts_s32;  
  int32_t Integral_prev_s32;  
  int32_t Integral_in_s32;  
  int32_t Integralout_s32;  
  
}INTEGRATOR_STRUCT;


/****** DPC_Math.h section End *****/

/****** DPC_Miscellaneous.h section Start *****/

/**
 *@brief
 */
typedef enum
{
  NO_LOAD = 0,                          /*!< */
  LOW_LOAD,                             /*!< */
  ON_LOAD,                              /*!< */
  OVERVOLTAGE_LOAD,                     /*!< */
  OVERCURRENT_LOAD,                     /*!< */
  OVERVOLTAGE_CAP,                      /*!< */
} DPC_MISC_DCLoadStatus_t;

/**
 *@brief
 */
typedef enum
{
  WAIT_DC_SOURCE = 0,                      /*!< */
  NO_DC_SOURCE,                            /*!< */
  UV_DC_SOURCE,                            /*!< */
  UVLO_DC_SOURCE,                          /*!< */  
  OK_DC_SOURCE,                            /*!< */
  OVERVOLTAGE_DC_SOURCE,                   /*!< */
  OVERCURRENT_DC_SOURCE,                   /*!< */
  ERR_DC_SOURCE,                           /*!< */
}DPC_MISC_DCSourceStatus_t;

/**
 *@brief
 */
typedef enum
{
  WAIT_Plug_DCSource = 0,               /*!< */  
  NO_Plug_DCSource,                     /*!< */
  OK_Plug_DCSource,                     /*!< */
}DPC_MISC_PlugDCSourceStatus_t;


/**
 *@brief LEDs: Variables
 */
typedef enum
{
  BLED_Idle = 0x00,                    /*!< Bicolor LED state in DPC "Idle" mode*/
  BLED_StartUp_inrush = 0x01,          /*!< Bicolor LED state in DPC "StartUp_Inrush" mode*/
  BLED_Fault = 0x02,                   /*!< Bicolor LED state in DPC "Fault" mode*/
  BLED_Error = 0x3,                    /*!< Bicolor LED state in DPC "Error" mode*/
  BLED_Run = 0x04,                     /*!< Bicolor LED state in DPC "Run" mode*/
  BLED_StartUp_burst = 0x5,            /*!< Bicolor LED state in DPC "StartUp Burst" mode*/
  BLED_Stop = 0x6,                     /*!< Bicolor LED state in DPC "Stop" mode*/
  BLED_Debug = 0x7,                    /*!< Bicolor LED state in DPC "Debug" mode*/  
  BLED_Wait = 0x8,                     /*!< Bicolor LED state in DPC "Wait" mode*/    
} DPC_BLED_TypeDef;


/**
 *@brief Pulse BLEDs (colors) : Variables
 */
typedef enum
{
  BLED_Pulse_Idle = 0xB000,            /*!< Bicolor LED state in DPC "Idle" mode*/
  BLED_Pulse_StartUp_inrush = 0xE000,  /*!< Bicolor LED state in DPC "StartUp_Inrush" mode*/
  BLED_Pulse_Fault = 0x0000,           /*!< Bicolor LED state in DPC "Fault" mode*/
  BLED_Pulse_Error = 0x0000,           /*!< Bicolor LED state in DPC "Error" mode*/
  BLED_Pulse_Run = 0xFFFF,             /*!< Bicolor LED state in DPC "Run" mode*/
  BLED_Pulse_StartUp_burst = 0x0FFF,   /*!< Bicolor LED state in DPC "StartUp Burst" mode*/
  BLED_Pulse_Stop = 0xD000,            /*!< Bicolor LED state in DPC "Stop" mode*/
  BLED_Pulse_Debug = 0xA000,           /*!< Bicolor LED state in DPC "Debug" mode*/  
  BLED_Pulse_Wait = 0xC000,            /*!< Bicolor LED state in DPC "Wait" mode*/     
} DPC_BLED_Pulse_TypeDef;

/**
 *@brief DC Load: Variables & Limit
 */
typedef struct {
uint16_t V_cap_Limit;                   /*!< */
uint16_t I_No_load_Threshold;           /*!< */
uint16_t I_No_load_Max_Threshold;       /*!< */
uint16_t I_No_load_Min_Threshold;       /*!< */
uint16_t I_Low_load_Threshold;          /*!< */
uint16_t I_Low_load_Max_Threshold;      /*!< */
uint16_t I_Low_load_Min_Threshold;      /*!< */
uint16_t I_Over_load_Threshold;         /*!< */
uint16_t V_dc_Limit;                    /*!< */
}DPC_MISC_DCLoadLimit_t;

/** 
 *@brief DC Source: Variables & Limit
 */
typedef struct {
uint16_t V_dc_pos_Limit;                /*!< OverVoltageProtection "OVP" DC voltage limit expressed in volt*/
uint16_t V_dc_neg_Limit;                /*!< OverVoltageProtection "OVP" DC voltage limit expressed in volt*/
uint16_t V_dc_pos_UV_Limit;             /*!< UnderVoltage "UV" DC voltage limit expressed in volt (Recoverable)*/
uint16_t V_dc_neg_UV_Limit;             /*!< UnderVoltage "UV" DC voltage limit expressed in volt (Recoverable)*/
uint16_t V_dc_pos_UVLO_Limit;           /*!< UnderVoltageLockOut "UVLO" DC voltage limit expressed in volt (UnRecoverable)*/
uint16_t V_dc_neg_UVLO_Limit;           /*!< UnderVoltageLockOut "UVLO" DC voltage limit expressed in volt (UnRecoverable)*/
uint16_t V_dc_pos_Low_Limit;            /*!< Min Voltage DC voltage limit expressed in volt (Start-up)*/
uint16_t V_dc_neg_Low_Limit;            /*!< Min Voltage DC voltage limit expressed in volt (Start-up)*/
uint16_t I_dc_pos_Limit;                /*!< Over Current Protection "OCP" DC current limit expressed amp*/
uint16_t I_dc_neg_Limit;                /*!< Over Current Protection "OCP" DC current limit expressed amp*/
}DPC_MISC_DCSourceLimit_t;

/** 
 *@brief DC Source Variables & Limit
 */
typedef struct {
uint16_t V_dc_pk_pos_local;                     /*!< */
uint16_t V_dc_pk_neg_local;                     /*!< */
uint16_t I_dc_pk_pos_local;                     /*!< */
uint16_t I_dc_pk_neg_local;                     /*!< */
DPC_MISC_DCSourceStatus_t Status_DCSource;        /*!< */
}DPC_MISC_DCSource_t;

/** 
 *@brief DC Load 
 */
typedef struct {
DPC_MISC_DCLoadStatus_t DPC_Load_Status;/*!< */
}DPC_MISC_DCLoad_t;


/* Exported constants --------------------------------------------------------*/
#define DOUBLE_PI 6.28318530718f //Represent 2*Pi in floating point
#define DOUBLE_PI_s32 25729 //6.28318530718f*((2^16)-1) Represent 2*Pi in FxP
#define HALF_PI 1.57079632679f //Represent Pi/2 in floating point
/* Exported macro ------------------------------------------------------------*/
/*!DAC Channel Struct Declaration*/
typedef struct{
uint8_t CH1;                            /*!< Data connector for DAC Channel 1*/  
uint8_t CH2;                            /*!< Data connector for DAC Channel 2*/
uint8_t CH3;                            /*!< Data connector for DAC Channel 3*/
uint16_t Gain_CH1;                      /*!< Gain Data for DAC Channel 1*/
uint16_t Gain_CH2;                      /*!< Gain Data for DAC Channel 2*/
uint16_t Gain_CH3;                      /*!< Gain Data for DAC Channel 3*/
uint16_t Bias_CH1;                      /*!< Bias Data for DAC Channel 1*/
uint16_t Bias_CH2;                      /*!< Bias Data for DAC Channel 2*/
uint16_t Bias_CH3;                      /*!< Bias Data for DAC Channel 3*/
}DAC_Channel_STRUCT;    

/* Exported functions ------------------------------------------------------- */

void DATA_Write_Theta_PLL(float Theta);    
float DATA_Read_Theta_PLL(void);
void InitDMA_ADC_CONV(void);
RAW_ADC_UNION* DATA_Read_ADC_Raw(void);


void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data,uint32_t* p_ADC2_Data); 

    


DPC_CDT_CURR_DC_ADC_t* Read_Curr_DC(void);
DPC_CDT_VOLT_DC_ADC_t* Read_Volt_DC(void);
DAB_ADC_Value_Struct* Read_DAB(void);


#endif    /*__DPC_DATACOLLECTOR_H*/


/************************ (C) COPYRIGHT STMicroelectronics 2020 END OF FILE****/
