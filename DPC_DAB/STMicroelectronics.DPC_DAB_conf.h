/**
  ******************************************************************************
  * File Name          : STMicroelectronics.DPC_DAB_conf.h
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.DPC_DAB_conf.h instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STMICROELECTRONICS__DPC_DAB_CONF__H__
#define __STMICROELECTRONICS__DPC_DAB_CONF__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/**
	MiddleWare name : STMicroelectronics.eDS-WB_DPC_DAB.1.0.0
	MiddleWare fileName : STMicroelectronics.DPC_DAB_conf.h
	MiddleWare version :
*/
/*---------- DPC_DAB_VDC_OUT  -----------*/
#define eDS_DPC_DAB_VDC_OUT      450

/*---------- DPC_DAB_IDC_OUT  -----------*/
#define eDS_DPC_DAB_IDC_OUT      56

/*---------- DPC_DAB_G_VDC1  -----------*/
#define eDS_DPC_DAB_G_VDC1      3.5986

/*---------- DPC_DAB_B_VDC1  -----------*/
#define eDS_DPC_DAB_B_VDC1      0

/*---------- DPC_DAB_G_IDC1  -----------*/
#define eDS_DPC_DAB_G_IDC1      39.046

/*---------- DPC_DAB_B_IDC1  -----------*/
#define eDS_DPC_DAB_B_IDC1      2045

/*---------- DPC_DAB_G_VDC2  -----------*/
#define eDS_DPC_DAB_G_VDC2      6.847

/*---------- DPC_DAB_B_VDC2  -----------*/
#define eDS_DPC_DAB_B_VDC2      0

/*---------- DPC_DAB_G_IDC2  -----------*/
#define eDS_DPC_DAB_G_IDC2      24.506

/*---------- DPC_DAB_B_IDC2  -----------*/
#define eDS_DPC_DAB_B_IDC2      2047

/*---------- DPC_DAB_INDUCTANCE  -----------*/
#define eDS_DPC_DAB_INDUCTANCE      0.00002861

/*---------- DPC_DAB_TRAFO_TURN_RATIO  -----------*/
#define eDS_DPC_DAB_TRAFO_TURN_RATIO      1.78

/*---------- DPC_DAB_VCTRL_KP  -----------*/
#define eDS_DPC_DAB_VCTRL_KP      0.0005588

/*---------- DPC_DAB_VCTRL_KI  -----------*/
#define eDS_DPC_DAB_VCTRL_KI      6.158

/*---------- DPC_DAB_VCTRL_PI_SAT_EN  -----------*/
#define eDS_DPC_DAB_VCTRL_PI_SAT_EN      true

/*---------- DPC_DAB_VCTRL_PI_AW_EN  -----------*/
#define eDS_DPC_DAB_VCTRL_PI_AW_EN      true

/*---------- DPC_DAB_VCTRL_PI_SAT_UP  -----------*/
#define eDS_DPC_DAB_VCTRL_PI_SAT_UP      0.45

/*---------- DPC_DAB_VCTRL_PI_SAT_DOWN  -----------*/
#define eDS_DPC_DAB_VCTRL_PI_SAT_DOWN      0

/*---------- DPC_DAB_PI_VDC_RES_VAL  -----------*/
#define eDS_DPC_DAB_PI_VDC_RES_VAL      0

/*---------- DPC_DAB_ICTRL_KP  -----------*/
#define eDS_DPC_DAB_ICTRL_KP      0.004526

/*---------- DPC_DAB_ICTRL_KI  -----------*/
#define eDS_DPC_DAB_ICTRL_KI      49.88

/*---------- DPC_DAB_ICTRL_PI_SAT_EN  -----------*/
#define eDS_DPC_DAB_ICTRL_PI_SAT_EN      true

/*---------- DPC_DAB_ICTRL_PI_AW_EN  -----------*/
#define eDS_DPC_DAB_ICTRL_PI_AW_EN      true

/*---------- DPC_DAB_ICTRL_PI_SAT_UP  -----------*/
#define eDS_DPC_DAB_ICTRL_PI_SAT_UP      0.45

/*---------- DPC_DAB_PI_IDC_RES_VAL  -----------*/
#define eDS_DPC_DAB_PI_IDC_RES_VAL      0

/*---------- DPC_DAB_CTRL_INIT  -----------*/
#define eDS_DPC_DAB_CTRL_INIT      2

/*---------- DPC_DAB_VDCHV_OVP  -----------*/
#define eDS_DPC_DAB_VDCHV_OVP      968

/*---------- DPC_DAB_VDCHV_UV  -----------*/
#define eDS_DPC_DAB_VDCHV_UV      30

/*---------- DPC_DAB_VDCHV_UVLO  -----------*/
#define eDS_DPC_DAB_VDCHV_UVLO      640

/*---------- DPC_DAB_IDCHV_OCP  -----------*/
#define eDS_DPC_DAB_IDCHV_OCP      44

/*---------- DPC_DAB_IDCLV_OCP  -----------*/
#define eDS_DPC_DAB_IDCLV_OCP      72

/*---------- DPC_DAB_IDCLV_OVP  -----------*/
#define eDS_DPC_DAB_VDCLV_OVP      518

/*---------- DPC_DAB_IDCHV_MIN  -----------*/
#define eDS_DPC_DAB_VDCHV_MIN      20

/*---------- DPC_DAB_DAC_CH1_INIT  -----------*/
#define eDS_DPC_DAB_DAC_CH1_INIT      12

/*---------- DPC_DAB_DAC_G_CH1_INIT  -----------*/
#define eDS_DPC_DAB_DAC_G_CH1_INIT      2048

/*---------- DPC_DAB_DAC_B_CH1_INIT  -----------*/
#define eDS_DPC_DAB_DAC_B_CH1_INIT      2048

/*---------- DPC_DAB_DAC_CH2_INIT  -----------*/
#define eDS_DPC_DAB_DAC_CH2_INIT      3

/*---------- DPC_DAB_DAC_G_CH2_INIT  -----------*/
#define eDS_DPC_DAB_DAC_G_CH2_INIT      2048

/*---------- DPC_DAB_DAC_B_CH2_INIT  -----------*/
#define eDS_DPC_DAB_DAC_B_CH2_INIT      2048

/*---------- DPC_DAB_DAC_CH3_INIT  -----------*/
#define eDS_DPC_DAB_DAC_CH3_INIT      4

/*---------- DPC_DAB_DAC_G_CH3_INIT  -----------*/
#define eDS_DPC_DAB_DAC_G_CH3_INIT      2048

/*---------- DPC_DAB_DAC_B_CH3_INIT  -----------*/
#define eDS_DPC_DAB_DAC_B_CH3_INIT      2048

/*---------- DPC_DAB_PWM_INIT  -----------*/
#define eDS_DPC_DAB_PWM_INIT      1

/*---------- DPC_DAB_PWM_FREQ  -----------*/
#define eDS_DPC_DAB_PWM_FREQ      100000

/*---------- DPC_DAB_DT_DAB1  -----------*/
#define eDS_DPC_DAB_DT_DAB1      4e-7

/*---------- DPC_DAB_DT_DAB2  -----------*/
#define eDS_DPC_DAB_DT_DAB2      4e-7

#ifdef __cplusplus
}
#endif
#endif /*__ STMICROELECTRONICS__DPC_DAB_CONF__H_H */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
