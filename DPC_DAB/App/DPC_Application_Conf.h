/**
******************************************************************************
* @file    DPC_Application_Conf.h
* @brief   This file contains the DPC Library Configuration.
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
#ifndef __DPC_APPLICATION_CONF_H
#define __DPC_APPLICATION_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "STMicroelectronics.DPC_DAB_conf.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

///__Start_________________________________________________________________STDES_DCDCDAB_______________________________________________________________________

///DPC Finite State Machine DEFINE of STDES_DABBIDIR

//#define DPC_DAB_CPWM
#define DPC_DAB_ADPPWM

#define DPC_PC_State_Init               FSM_Idle                                        /*!< PC_State_TypeDef */
#define DPC_FSM_RUN_INIT                Run_Idle                                        /*!< Run_State_TypeDef */
#define DPC_FSM_STATE_INIT              DPC_FSM_WAIT                                    /*!<  */

#ifndef eDS_DPC_DAB_CTRL_INIT
#define DPC_CTRL_INIT                   DAB_OPEN_LOOP                                     /*!<  DPC Init Operation */ //eDS
//#define DPC_CTRL_INIT                   DAB_CURRENT_LOOP                                  /*!<  DPC Init Operation */
//#define DPC_CTRL_INIT                   DAB_VOLTAGE_LOOP                                  /*!<  DPC Init Operation */
#else
#if eDS_DPC_DAB_CTRL_INIT == 0
  #define DPC_CTRL_INIT                        DAB_OPEN_LOOP                                     /*!<  DPC Init Operation OPEN LOOP*/
#endif
#if eDS_DPC_DAB_CTRL_INIT == 1
  #define DPC_CTRL_INIT                        DAB_CURRENT_LOOP                                  /*!<  DPC Init Operation CURRENT LOOP*/
#endif
#if eDS_DPC_DAB_CTRL_INIT == 2
  #define DPC_CTRL_INIT                        DAB_VOLTAGE_LOOP                                  /*!<  DPC Init Operation VOLTAGE LOOP*/
#endif
#endif

#ifndef eDS_DPC_DAB_PWM_INIT
#define DPC_DAB_PWM_INIT                 PWM_Armed                                       /*!<Use during normal operation*///eDS
//#define DPC_DAB_PWM_INIT                 PWM_Safe                                        /*!<Use during Sensing debugging*/
#else
#define DPC_DAB_PWM_INIT                 eDS_DPC_DAB_PWM_INIT
#endif
///_________________________________________________________________________BASIC APPLICATION CONFIGURATOR______________________________________

///---------------------------------------------------DAB phisics------------------------------------------------

#ifndef eDS_DPC_DAB_TRAFO_TURN_RATIO    /*!< Transformer turn ratio [-]*/
#define DPC_DAB_TRAFO_TURN_RATIO        1.78
#else
#define DPC_DAB_TRAFO_TURN_RATIO        eDS_DPC_DAB_TRAFO_TURN_RATIO
#endif

#ifndef eDS_DPC_DAB_INDUCTANCE          /*!< DAB External inductance [H]*/
#define DPC_DAB_INDUCTANCE              28e-6
#else
#define DPC_DAB_INDUCTANCE              eDS_DPC_DAB_INDUCTANCE
#endif

///---------------------------------------------------DC OUTPUT  DEFINE of STDES-DCDC_DAB------------------------
#ifndef eDS_DPC_DAB_VDC_OUT             /*!< DPC - DC Outout Voltage referance value of the Power converetr [Expresed in Volt]*/
#define DPC_DAB_VDC_OUT                 50
#else
#define DPC_DAB_VDC_OUT                 eDS_DPC_DAB_VDC_OUT
#endif

#ifndef eDS_DPC_DAB_IDC_OUT             /*!< DPC - DC Outout Current referance value of the Power converetr [Expresed in Amps]*/
#define DPC_DAB_IDC_OUT                 1
#else
#define DPC_DAB_IDC_OUT                 eDS_DPC_DAB_IDC_OUT
#endif

///---------------------------------------------------PROTECTION------------------------
#ifndef eDS_DPC_DAB_VDCHV_OVP           /*!< High Voltage Side OverVoltageProtection "OVP" DC voltage limit [Expressed in Volts]*/
#define DPC_DAB_VDCHV_OVP               900
#else
#define DPC_DAB_VDCHV_OVP               eDS_DPC_DAB_VDCHV_OVP
#endif

#ifndef eDS_DPC_DAB_VDCHV_UV            /*!< High Voltage Side UnderVoltage "UV" DC voltage limit (Recoverable) [Expressed in Volts]*/
#define DPC_DAB_VDCHV_UV                40
#else
#define DPC_DAB_VDCHV_UV                eDS_DPC_DAB_VDCHV_UV
#endif

#ifndef eDS_DPC_DAB_VDCHV_UVLO          /*!< High Voltage Side UnderVoltageLockOut "UVLO" DC voltage (UnRecoverable) [Expressed in Volts]*/
#define DPC_DAB_VDCHV_UVLO              30
#else
#define DPC_DAB_VDCHV_UVLO              eDS_DPC_DAB_VDCHV_UVLO
#endif

#ifndef eDS_DPC_DAB_VDCHV_MIN           /*!< High Voltage Side Min Voltage DC voltage limit (Start-up) [Expressed in Volts]*/
#define DPC_DAB_VDCHV_MIN               20
#else
#define DPC_DAB_VDCHV_MIN               eDS_DPC_DAB_VDCHV_MIN
#endif

#ifndef eDS_DPC_DAB_IDCHV_OCP           /*!< High Voltage Side Source Over current threshold [Expressed in AMPs]*/
#define DPC_DAB_IDCHV_OCP               50
#else
#define DPC_DAB_IDCHV_OCP               eDS_DPC_DAB_IDCHV_OCP
#endif

#ifndef eDS_DPC_DAB_IDCLV_OCP           /*!< low voltage side Load Over current threshold of  [Expressed in AMPs]*/
#define DPC_DAB_IDCLV_OCP               100
#else
#define DPC_DAB_IDCLV_OCP               eDS_DPC_DAB_IDCLV_OCP
#endif

#ifndef eDS_DPC_DAB_VDCLV_OVP           /*!< Low Voltage Side OverVoltageProtection "OVP" DC voltage limit [Expressed in Volts]*/
#define DPC_DAB_VDCLV_OVP               550
#else
#define DPC_DAB_VDCLV_OVP               eDS_DPC_DAB_VDCLV_OVP
#endif

///_________________________________________________________________________ADV APPLICATION CONFIGURATOR______________________________________

///StartupCkeck DEFINE of STDES-DCDC_DAB
#define STARTUPCHECK_INIT               StartUpCheck_Disabled
//#define STARTUPCHECK_INIT              StartUpCheck_Enabled
///Inrush DEFINE of STDES-DCDC_DAB
#define DPC_INRS_EN                     RESET                                             /*!<*/
///DPC Run Burst Define of STDES-DCDC_DAB
#define DPC_BURST_EN                    RESET                                             /*!< [Expressed in Boolean]*/

///DPC LOAD Define of STDES_DABBIDIR
#define DPC_NO_LOAD_CURR                0.5                                             /*!< [Expressed in AMPs]*/
#define DPC_LOW_LOAD_CURR               1.2                                             /*!< [Expressed in AMPs]*/
#define DPC_NO_LOAD_DELTA_CURR          20                                              /*!< [Expressed in % Percentage]*/
#define DPC_LOW_LOAD_DELTA_CURR         30                                              /*!< [Expressed in % Percentage]*/

///DPC START-UP Define of STDES_DABBIDIR
#define INRUSH_VREF_V                   200                                             /*!< INRUSH DC Voltage threshold - (Expressed in VOLT)*/
#define INRUSH_DELTA_MAX                10                                              /*!< Hysteresis Delta positive threshold INRUSH DC Voltage[Expressed in volt-rms]*/
#define INRUSH_DELTA_MIN                10                                              /*!< Hysteresis Delta negative threshold INRUSH DC Voltage[Expressed in volt-rms]*/
#define DPC_INRUSH_NO_LOAD_CURR         1                                               /*!< INRUSH mode - DC Current Theshold [Expressed in AMPs]*/

#define RUN_BURST_VREF_V                0                                               /*!< [Expressed in Volts]*/
#define RUN_BURST_VHIST                 0                                               /*!< [Expressed in Volts]*/
#define DPC_BURST_DUTY_NL               0                                               /*!< [Expressed in Unit]*/
#define DPC_BURST_DUTY_LL               0                                               /*!< [Expressed in Unit]*/

///_________________________________________________________________________CONTROL CONFIGURATOR______________________________________

///DPC Task DEFINE of STDES-DCDC_DAB
#define RefreshTime_DESIDERED           10000                                           /*!< Expressed in hz */
#define RefreshTime_TO_DESIDERED        1000                                            /*!< Expressed in hz */
#define RefreshTime2_DESIDERED          3000                                            /*!< Expressed in hz */
#define RefreshTime3_DESIDERED          2000                                            /*!< Expressed in hz */
#define DPC_PI_VDC_TS                   ((float)1/RefreshTime_DESIDERED)                /*!< Discrete time step of the PI regulator related to DC voltage control */
#define DPC_PI_IDC_TS                   ((float)1/RefreshTime_DESIDERED)                /*!< Discrete time step of the PI regulator related to DC current control */

///---------------------------------------------------DPC VCTRL Define of STDES_DABBIDIR------------------------
#ifndef eDS_DPC_DAB_VCTRL_KP            /*!< VCTRL - Proportional gain of the PI regulator related to DC voltage control*/
#define DPC_DAB_VCTRL_KP                0.0005
#else
#define DPC_DAB_VCTRL_KP                eDS_DPC_DAB_VCTRL_KP
#endif

#ifndef eDS_DPC_DAB_VCTRL_KI            /*!< VCTRL - Integral gain of the PI regulator related to DC voltage control*/
#define DPC_DAB_VCTRL_KI                0.1
#else
#define DPC_DAB_VCTRL_KI    eDS_DPC_DAB_VCTRL_KI
#endif

#define DPC_DAB_VDC                     DPC_DAB_VDC_OUT                                 /*!< VCTRL - DC Voltage referance value of the DPC [Expresed in Volt]*/
#define DPC_VCTRL_PI_AWTG               DPC_DAB_VCTRL_KI                                /*!< VCTRL - Anti Wind-up GAIN*/

#ifndef eDS_DPC_DAB_VCTRL_PI_SAT_UP     /*!< VCTRL - Higher Referance Saturation LIMIT*/
#define DPC_DAB_VCTRL_PI_SAT_UP         0.45
#else
#define DPC_DAB_VCTRL_PI_SAT_UP         eDS_DPC_DAB_VCTRL_PI_SAT_UP
#endif

#ifndef eDS_DPC_DAB_VCTRL_PI_SAT_DOWN   /*!< VCTRL - Lower Referance Saturation LIMIT*/
#define DPC_DAB_VCTRL_PI_SAT_DOWN       -0.1
#else
#define DPC_DAB_VCTRL_PI_SAT_DOWN       eDS_DPC_DAB_VCTRL_PI_SAT_DOWN
#endif

#ifndef eDS_DPC_DAB_VCTRL_PI_SAT_EN     /*!< VCTRL - PI Referance Saturation Enable*/
#define DPC_DAB_VCTRL_PI_SAT_EN         SET
#else
#define DPC_DAB_VCTRL_PI_SAT_EN         eDS_DPC_DAB_VCTRL_PI_SAT_EN
#endif

#ifndef eDS_DPC_DAB_VCTRL_PI_AW_EN      /*!< VCTRL - Anti Wind-up Enable*/
#define DPC_DAB_VCTRL_PI_AW_EN          SET
#else
#define DPC_DAB_VCTRL_PI_AW_EN          eDS_DPC_DAB_VCTRL_PI_AW_EN
#endif

#ifndef eDS_DPC_DAB_PI_VDC_RES_VAL      /*!< VCTRL - Reset PI value*/
#define DPC_DAB_PI_VDC_RES_VAL          0
#else
#define DPC_DAB_PI_VDC_RES_VAL          eDS_DPC_DAB_PI_VDC_RES_VAL
#endif

///---------------------------------------------------DPC ICTRL Define of STDES_DABBIDIR------------------------
#ifndef eDS_DPC_DAB_ICTRL_KP            /*!< ICTRL - Proportional gain of the PI regulator related to DC current control*/
#define DPC_DAB_ICTRL_KP                0.0005
#else
#define DPC_DAB_ICTRL_KP                eDS_DPC_DAB_ICTRL_KP
#endif

#ifndef eDS_DPC_DAB_ICTRL_KI            /*!< ICTRL - Integral gain of the PI regulator related to DC current control*/
#define DPC_DAB_ICTRL_KI                0.1
#else
#define DPC_DAB_ICTRL_KI                eDS_DPC_DAB_ICTRL_KI
#endif

#define DPC_DAB_IDC                     DPC_DAB_IDC_OUT                                 /*!< ICTRL - DC Current referance value of the DPC [Expresed in Ampere]*/
#define DPC_ICTRL_PI_AWTG               DPC_DAB_ICTRL_KI                                /*!< ICTRL - Anti Wind-up GAIN*/

#ifndef eDS_DPC_DAB_ICTRL_PI_SAT_UP            /*!< ICTRL - Higher Referance Saturation LIMIT*/
#define DPC_DAB_ICTRL_PI_SAT_UP         0.45
#else
#define DPC_DAB_ICTRL_PI_SAT_UP         eDS_DPC_DAB_ICTRL_PI_SAT_UP
#endif

#ifndef eDS_DPC_DAB_ICTRL_PI_SAT_DOWN   /*!< ICTRL - Lower Referance Saturation LIMIT*/
#define DPC_DAB_ICTRL_PI_SAT_DOWN       -0.1
#else
#define DPC_DAB_ICTRL_PI_SAT_DOWN       eDS_DPC_DAB_ICTRL_PI_SAT_DOWN
#endif

#ifndef eDS_DPC_DAB_ICTRL_PI_SAT_EN     /*!< ICTRL - PI Saturation Enable*/
#define DPC_DAB_ICTRL_PI_SAT_EN         SET
#else
#define DPC_DAB_ICTRL_PI_SAT_EN         eDS_DPC_DAB_ICTRL_PI_SAT_EN
#endif

#ifndef eDS_DPC_DAB_ICTRL_PI_AW_EN      /*!< ICTRL - Anti Wind-up Enable*/
#define DPC_DAB_ICTRL_PI_AW_EN          SET
#else
#define DPC_DAB_ICTRL_PI_AW_EN          eDS_DPC_DAB_ICTRL_PI_AW_EN
#endif

#ifndef eDS_DPC_DAB_PI_IDC_RES_VAL      /*!< ICTRL - Reset PI value*/
#define DPC_DAB_PI_IDC_RES_VAL          0
#else
#define DPC_DAB_PI_IDC_RES_VAL          eDS_DPC_DAB_PI_IDC_RES_VAL
#endif

///_________________________________________________________________________PERIPHERALS CONFIGURATOR______________________________________

///---------------------------------------------------/DPC DAC Define of STDES-DCDC_DAB----------------------
#ifndef eDS_DPC_DAB_DAC_CH1_INIT            /*!PLL theta_out = 12<*/
#define DPC_DAC_CH1_INIT                12
#else
#define DPC_DAC_CH1_INIT                eDS_DPC_DAB_DAC_CH1_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_CH2_INIT            /*!V_ABC_CTRL_A = 0 V_ABC_CTRL_B = 1 V_ABC_CTRL_C = 2 CDC.Id_feed = 3 CDC.Id_ref 4<*/
#define DPC_DAC_CH2_INIT                3
#else
#define DPC_DAC_CH2_INIT                eDS_DPC_DAB_DAC_CH2_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_CH3_INIT            /*!V_ABC_CTRL_A = 0 V_ABC_CTRL_B = 1 V_ABC_CTRL_C = 2 CDC.Id_feed = 3 CDC.Id_ref 4<*/
#define DPC_DAC_CH3_INIT                4
#else
  #define DPC_DAC_CH3_INIT              eDS_DPC_DAB_DAC_CH3_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_G_CH1_INIT      /*!<*/
#define DPC_DAC_G_CH1_INIT              2048
#else
#define DPC_DAC_G_CH1_INIT              eDS_DPC_DAB_DAC_G_CH1_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_G_CH2_INIT      /*!<*/
#define DPC_DAC_G_CH2_INIT              2048
#else
#define DPC_DAC_G_CH2_INIT              eDS_DPC_DAB_DAC_G_CH2_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_G_CH3_INIT      /*!<*/
#define DPC_DAC_G_CH3_INIT              2048
#else
#define DPC_DAC_G_CH3_INIT              eDS_DPC_DAB_DAC_G_CH3_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_B_CH1_INIT      /*!<*/
#define DPC_DAC_B_CH1_INIT              2048
#else
#define DPC_DAC_B_CH1_INIT              eDS_DPC_DAB_DAC_B_CH1_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_B_CH2_INIT      /*!<*/
#define DPC_DAC_B_CH2_INIT              2048
#else
#define DPC_DAC_B_CH2_INIT              eDS_DPC_DAB_DAC_B_CH2_INIT
#endif

#ifndef eDS_DPC_DAB_DAC_B_CH3_INIT      /*!<*/
#define DPC_DAC_B_CH3_INIT              2048
#else
#define DPC_DAC_B_CH3_INIT              eDS_DPC_DAB_DAC_B_CH3_INIT
#endif

///---------------------------------------------------ADC Gain STDES-DCDC_DAB---------------------
#ifndef eDS_DPC_DAB_G_VDC1         	/*!< Gain terms of the DC input voltage sensing */
#define DPC_G_VDC1                      3.901
#else
#define DPC_G_VDC1                      eDS_DPC_DAB_G_VDC1
#endif

#ifndef eDS_DPC_DAB_B_VDC1	        /*!< Bias terms of the DC input voltage sensing */
#define DPC_B_VDC1                      0
#else
#define DPC_B_VDC1                      eDS_DPC_DAB_B_VDC1
#endif

#ifndef eDS_DPC_DAB_G_IDC1	        /*!< Gain terms of the DC input current sensing */
#define DPC_G_IDC1                      42.67
#else
#define DPC_G_IDC1                      eDS_DPC_DAB_G_IDC1
#endif

#ifndef eDS_DPC_DAB_B_IDC1	        /*!< Bias terms of the DC input current sensing */
#define DPC_B_IDC1                      2048
#else
#define DPC_B_IDC1                      eDS_DPC_DAB_B_IDC1
#endif

#ifndef eDS_DPC_DAB_G_VDC2	        /*!< Gain terms of the DC output voltage sensing */
#define DPC_G_VDC2                      6.296
#else
#define DPC_G_VDC2                      eDS_DPC_DAB_G_VDC2
#endif

#ifndef eDS_DPC_DAB_B_VDC2	        /*!< Bias terms of the DC output voltage sensing */
#define DPC_B_VDC2                      0
#else
#define DPC_B_VDC2                      eDS_DPC_DAB_B_VDC2
#endif

#ifndef eDS_DPC_DAB_G_IDC2	        /*!< Gain terms of the DC output current sensing */
#define DPC_G_IDC2                      24.948
#else
#define DPC_G_IDC2                      eDS_DPC_DAB_G_IDC2
#endif

#ifndef eDS_DPC_DAB_B_IDC2	        /*!< Bias terms of the DC output current sensing */
#define DPC_B_IDC2                      2048
#else
#define DPC_B_IDC2                      eDS_DPC_DAB_B_IDC2
#endif

#define TRIGGER_TIME_EVENT              30800                                           /*!<  */
#define ADC_TRG_CU                      HRTIM_COMPAREUNIT_4                             /*!< ADC trigger event*/

///---------------------------------------------------DPC TIMERs Define of STDES-DCDC_DAB---------------------
#ifndef eDS_DPC_DAB_PWM_FREQ            /*!< Switching Frequency of converter expressed in [Hz]*/
#define DPC_DAB_PWM_FREQ                100000
#else
#define DPC_DAB_PWM_FREQ                eDS_DPC_DAB_PWM_FREQ
#endif

#ifndef eDS_DPC_DAB_DT_DAB1             /*!< PWM - Dead Time [Expressed in sec]*/
#define DPC_DAB_DT_DAB1                 0.4e-6
#else
#define DPC_DAB_DT_DAB1                 eDS_DPC_DAB_DT_DAB1
#endif
#ifndef eDS_DPC_DAB_DT_DAB2             /*!< PWM - Dead Time [Expressed in sec]*/
#define DPC_DAB_DT_DAB2                 0.4e-6
#else
#define DPC_DAB_DT_DAB2                 eDS_DPC_DAB_DT_DAB2
#endif

#define DPC_BURST_PWM_FREQ              20000                                           /*!< Switching Frequency of converter expressed in [Hz]*/

///DPC TIMEOUTs Define of STDES-DCDC_DAB
#define TO_IDLE_Tick                    2000                                           /*!< Timeout Index - TimeOut_IDLE [mills]*/
#define TO_INIT_Tick                    1000                                            /*!< Timeout Index - TimeOut_IDLE [mills]*/
#define TO_START_Tick                   1000                                            /*!< Timeout Index - TimeOut_START [mills]*/

/* Exported functions ------------------------------------------------------- */

#endif //__DPC_APPLICATION_CONF_H
