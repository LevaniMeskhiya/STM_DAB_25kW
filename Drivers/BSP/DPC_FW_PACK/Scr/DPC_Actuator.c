/**
  ******************************************************************************
  * @file           : DPC_PWMConverter.c
  * @brief          : PWM modulation management
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

#include "math.h"
#include "DPC_Actuator.h"
#include "DPC_Math.h"

#ifdef USE_ADVTIM
#include "tim.h"
#endif   

#ifdef USE_HRTIM
#include "hrtim.h"
#include "stm32g4xx_ll_hrtim.h"
#endif  

/* external variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

////
float DT_dist_rangeA;              /*!< Distance to */
float DT_dist_rangeB;
float DT_dist_rangeC;
float DT_dist_rangeD;

float t_dtg_rangeA;
float t_dtg_rangeB;
float t_dtg_rangeC;
float t_dtg_rangeD;
float t_DTS;


/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/





/**
  * @brief  DPC_ACT_ADVTIM_SET_DeadTime: Set the dead time of ADVTIM  
  *         T PWM_Timx and PWM_CHANNEL_x, must be selected in the DPC_Lib_Conf.f
  *
  * @param  uint16_t uwDeadTimeSub> value od dead time 
  *
  * @retval Null 
  *
  * @note This function can be use only if the Dead time upgrade is UNLOCKED, 
  *        see TIM Lock level in stm32g4xx_hal_tim.  
  * @note Function valid for STM32G4xx family   
  */
void DPC_ACT_ADVTIM_SET_DeadTime(uint16_t uwDeadTimeSub)
{
#ifdef DPC_PWM_2ADVTIM_3CH_3CHX  
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

sBreakDeadTimeConfig.DeadTime = uwDeadTimeSub;

  if (HAL_TIMEx_ConfigBreakDeadTime(&PWM_Tim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&PWM_Tim1);

  if (HAL_TIMEx_ConfigBreakDeadTime(&PWM_Tim2, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&PWM_Tim2);

//DPC_PWM_2ADVTIM_3CH_3CHX
  
#else
 // Put here feature conf. of ADVTIM

#endif     
}




/**
* @brief  DPC_ACT_Calc_DeadTime: Accordingly with DT input variable expressed in seconds provide a better DTG[7:0] configuration of TIMx_BDTR
* 
* @param  DT_TimeVal: Dead time value expressed in seconds
* @retval DTG_bin: DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR)   
* @note  Function linked to  DPC_ACT_Calc_DeadTimRange function  
* @note  Function valid for STM32G474 microcontroller   
* @note  See 27.6.20 of RM0440 Rev1 (pag1162/2083)   
* @note  RangeX
RangeA -> DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.
RangeB -> DTG[7:5]=10x => DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
RangeC -> DTG[7:5]=110 => DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS.
RangeD -> DTG[7:5]=111 => DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS.
* @warning LOCK config must be disable to obtain effect of DTG_bin Setting
*/  
uint8_t DPC_ACT_Calc_DeadTime(float DT_TimeVal)
{
  
#if defined(USE_ADVTIM)  
  uint32_t DTG_dec;             /*!< DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR) expressed in decimal integer*/  
  uint32_t DTG_bin;             /*!< DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR) expressed in HEX*/ 
  float DT_dist_range_min;         /*!< Local variable to save the dist_rangeX if it's lower*/
  
  
  /*! Find lower dist_rangeX*/
  
  if(DT_dist_rangeA<DT_dist_rangeB){
    DT_dist_range_min=DT_dist_rangeA;
  }
  else
  {
    DT_dist_range_min=DT_dist_rangeB;
  }
  
  if(DT_dist_rangeB<DT_dist_rangeC)
  {
    DT_dist_range_min=DT_dist_range_min;
  }
  else{
    DT_dist_range_min=DT_dist_rangeC;
  }
  if(DT_dist_rangeC<DT_dist_rangeD)
  {
    DT_dist_range_min=DT_dist_range_min;
  }
  else{
    DT_dist_range_min=DT_dist_rangeD;
  }
  
  
  /*! Generate DTG[7-5] according with the better range*/ 
  
  if (DT_dist_range_min==DT_dist_rangeA){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeA));
    DTG_bin = (DTG_dec & 0x7F) | 0x00;                  /// DTG[7-5]=0xx <=> *0x7F=0111 1111 e 0x00=0000 0000
  }
  else if (DT_dist_range_min==DT_dist_rangeB){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeB))-64;
    DTG_bin = (DTG_dec & 0x3F) | 0x80;                  /// DTG[7-5]=10x <=> *0x3F=0011 1111 e 0x80=1000 0000
  } 
  else if (DT_dist_range_min==DT_dist_rangeC){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeC))-32;
    DTG_bin = (DTG_dec & 0x1F) | 0xC0;                  /// DTG[7-5]=110 <=> *0x1F=0001 1111 e  0xC0=1100 0000 
  }
  else if (DT_dist_range_min==DT_dist_rangeD){
    DTG_dec= (uint32_t)((DT_TimeVal/t_dtg_rangeD))-32;
    DTG_bin = (DTG_dec & 0x1F) | 0xE0;                  /// DTG[7-5]=111 <=> *0x1F=0001 1111 e  0xE0=1110 0000   
  }
  
#elif defined(USE_HRTIM)
  uint32_t DTG_bin;             /*!< DTG[7:0] Dead-time generator setup (bits of TIMx_BDTR) expressed in HEX*/ 
#endif      
  
  
  float F_HRTIM=170e6;
  float T_HRTIM = 1.0f/F_HRTIM;
  float T_DTG;
  uint32_t DTPRSC;  
  
  //  DTPRSC=LL_HRTIM_DT_GetPrescaler(hhrtim1.Instance, LL_HRTIM_TIMER_A); 
  DTPRSC=LL_HRTIM_DT_PRESCALER_MUL2;
  
  
  switch(DTPRSC){
  case LL_HRTIM_DT_PRESCALER_MUL8:  //
    T_DTG=1*T_HRTIM/8;
    break;
  case LL_HRTIM_DT_PRESCALER_MUL4:  //
    T_DTG=2*T_HRTIM/8;
    break;
  case LL_HRTIM_DT_PRESCALER_MUL2:  //
    T_DTG=4*T_HRTIM/8;
    break;
  case LL_HRTIM_DT_PRESCALER_DIV1:  //
    T_DTG=8*T_HRTIM/8;
    break;
  case LL_HRTIM_DT_PRESCALER_DIV2:  //
    T_DTG=16*T_HRTIM/8;
    break;
  case LL_HRTIM_DT_PRESCALER_DIV4:  //
    T_DTG=32*T_HRTIM/8;
    break;
  case LL_HRTIM_DT_PRESCALER_DIV8:  //
    T_DTG=64*T_HRTIM/8;
    break;
  case LL_HRTIM_DT_PRESCALER_DIV16:  //
    T_DTG=120*T_HRTIM/8;
    break;
  default:
    Error_Handler();
    break;
  }
  
  
  DTG_bin=(uint32_t)(DT_TimeVal/T_DTG);
  
  return DTG_bin;
}



/**
  * @brief  DPC_ACT_Calc_DeadTimRange: Accordingly to TIMx clk , prescaler and DeadTime request the possible DeadTime ranges will be determinate.  
  * @param  t_tim_ket_ck: Internal clock (tim_ker_ck)
  * @param  DT_TimeVal: Dead time value expressed in seconds
  * @retval none
  * @note  Function linked to  DPC_ACT_Calc_DeadTime function  
  * @note  Function valid for STM32G474 microcontroller   
  * @note  See 27.6.20 of RM0440 Rev1 (pag 1162/2083) & 27.6.1 TIMx control register 1 (TIMx_CR1)(x = 1, 8, 20) of RM0440 Rev1 (pag 1131/2083)
  * @note __HAL_TIM_GET_CLOCKDIVISION -> CKD[1:0]: Clock division This bit-field indicates the division ratio between the timer clock (tim_ker_ck) frequency and the dead-time and sampling clock (tDTS)used by the dead-time generators and the digital filters (tim_etr_in, tim_tix),
  *         
  */  
void DPC_ACT_Calc_DeadTimRange(float DT_TimeVal, float t_tim_ket_ck){
  
  
#if defined(USE_ADVTIM)
/*!Get clock division of the PWM advanced timer to determinate the dead-time and sampling clock (tDTS)*/   
 uint32_t Tim_Clock_Division_DivX=__HAL_TIM_GET_CLOCKDIVISION(&PWM_Tim1);
  
  switch(Tim_Clock_Division_DivX){
  case TIM_CLOCKDIVISION_DIV1:  //
    t_DTS=t_tim_ket_ck;
    break;
  case TIM_CLOCKDIVISION_DIV2:  //
    t_DTS=2*t_tim_ket_ck;
    break;
  case TIM_CLOCKDIVISION_DIV4:  //
    t_DTS=4*t_tim_ket_ck;
    break;
  default:
  Error_Handler();
    break;
  }

/*! 
Dead Time Range - DT=DTG[7:0]x tdtg with tdtg=tDTS
*/
  t_dtg_rangeA=t_DTS;
  float DTG_rangeA_max=127.0*t_dtg_rangeA;
  float DTG_rangeA_min=0.0*t_dtg_rangeA;
//  float step_rangeA=t_dtg_rangeA;
  float center_rangeA=(DTG_rangeA_max+DTG_rangeA_min)/2;
  DT_dist_rangeA=center_rangeA-DT_TimeVal;
  if(DT_dist_rangeA<0){DT_dist_rangeA=-1.0f*DT_dist_rangeA;}else{DT_dist_rangeA=DT_dist_rangeA;}
  
/*! 
Dead Time Range - DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
*/  
  t_dtg_rangeB=2.0*t_DTS;
  float DTG_rangeB_max=(64+63.0)*t_dtg_rangeB;
  float DTG_rangeB_min=(64+0.0)*t_dtg_rangeB;
//  float step_rangeB=t_dtg_rangeB;
  float center_rangeB=(DTG_rangeB_max+DTG_rangeB_min)/2;
  DT_dist_rangeB=center_rangeB-DT_TimeVal;
  if(DT_dist_rangeB<0){DT_dist_rangeB=-1.0f*DT_dist_rangeB;}else{DT_dist_rangeB=DT_dist_rangeB;}

/*! 
Dead Time Range - DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS
*/    
  t_dtg_rangeC=8.0*t_DTS;
  float DTG_rangeC_max=(32+31.0)*t_dtg_rangeC;
  float DTG_rangeC_min=(32+0.0)*t_dtg_rangeC;
//  float step_rangeC=t_dtg_rangeC;
  float center_rangeC=(DTG_rangeC_max+DTG_rangeC_min)/2;
  DT_dist_rangeC=center_rangeC-DT_TimeVal;
  if(DT_dist_rangeC<0){DT_dist_rangeC=-1.0f*DT_dist_rangeC;}else{DT_dist_rangeC=DT_dist_rangeC;}

/*! 
Dead Time Range - DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS
*/    
  t_dtg_rangeD=16.0*t_DTS;
  float DTG_rangeD_max=(32+31.0)*t_dtg_rangeD;
  float DTG_rangeD_min=(32+0.0)*t_dtg_rangeD;
//  float step_rangeD=t_dtg_rangeD;
  float center_rangeD=(DTG_rangeD_max+DTG_rangeD_min)/2;
  DT_dist_rangeD=center_rangeD-DT_TimeVal;
  if(DT_dist_rangeD<0){DT_dist_rangeD=-1.0f*DT_dist_rangeD;}else{DT_dist_rangeD=DT_dist_rangeD;}
  
#elif defined(USE_HRTIM)
    //Put here the HRTIM Function
#endif    


 
}


/**
  * @brief  DPC_ACT_Conf_DeadTime: Set DeadTime value configuration
  * @param  
  * @param  
  * @retval none
  * @note  Function linked to  DPC_ACT_Calc_DeadTime and DPC_ACT_Calc_DeadTimRange functions   
  * @note  Function valid for STM32G474 microcontroller   
  *         
  */ 
void DPC_ACT_Conf_DeadTime(uint32_t Deadtime){
  
#if defined(USE_ADVTIM)
  HAL_TIMEx_ConfigDeadTime(&PWM_Tim1, Deadtime);                                /*!<*/
  HAL_TIMEx_ConfigDeadTime(&PWM_Tim2, Deadtime);                                /*!<*/
#elif defined(USE_HRTIM)  
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_1, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_1, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_2, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_2, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_3, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_3, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_4, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_4, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_5, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_5, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_6, Deadtime);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_6, Deadtime);    /*!<*/
#else
  #error PWM TIMER  not specified
#endif
}


/**
  * @brief  DPC_ACT_DAB_Conf_DeadTime: Set DeadTime value configuration
  * @param  
  * @param  
  * @retval none
  * @note  Function linked to  DPC_ACT_Calc_DeadTime and DPC_ACT_Calc_DeadTimRange functions   
  * @note  Function valid for STM32G474 microcontroller   
  *         
  */ 
void DPC_ACT_DAB_Conf_DeadTime(uint32_t DT_Dab1,uint32_t DT_Dab2){
  
#if defined(USE_ADVTIM)
  HAL_TIMEx_ConfigDeadTime(&PWM_Tim1, DT_Dab1);                                 /*!<*/
  HAL_TIMEx_ConfigDeadTime(&PWM_Tim2, DT_Dab1);                                 /*!<*/
#elif defined(USE_HRTIM)  
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_1, DT_Dab1);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_1, DT_Dab1);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_2, DT_Dab1);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_2, DT_Dab1);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_3, DT_Dab2);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_3, DT_Dab2);    /*!<*/
  LL_HRTIM_DT_SetRisingValue (PWM_Tim1.Instance, PWM_LL_IDX_CH_4, DT_Dab2);    /*!<*/
  LL_HRTIM_DT_SetFallingValue(PWM_Tim1.Instance, PWM_LL_IDX_CH_4, DT_Dab2);    /*!<*/
#else
  #error PWM TIMER  not specified
#endif
}




/**
* @brief  Start PWM generated by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
#ifdef USE_HRTIM
void DPC_ACT_HRTIM_Start(void)
{     
  HAL_HRTIM_WaveformCounterStart(&PWM_Tim1, HRTIM_TIMERID_MASTER);  
  HAL_HRTIM_WaveformCounterStart(&PWM_Tim1, PWM_ID_CHANNEL_1 + PWM_ID_CHANNEL_2 + PWM_ID_CHANNEL_3 + PWM_ID_CHANNEL_4);     
  HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,  PWM_CHANNEL_1 + PWM_CHANNEL_1N + PWM_CHANNEL_2 + PWM_CHANNEL_2N + PWM_CHANNEL_3 + PWM_CHANNEL_3N + PWM_CHANNEL_4 + PWM_CHANNEL_4N);    
}
#endif  



/**
* @brief  DPC_ACT_HRTIM_OutDisable Output DISABLE PWM generator by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
#ifdef USE_HRTIM
void DPC_ACT_HRTIM_OutDisable(void)
{     
    /* Disable the generation of the waveform signal on the designated output(s)*/     
HAL_HRTIM_WaveformOutputStop(&PWM_Tim1,PWM_CHANNEL_1 | PWM_CHANNEL_2 | PWM_CHANNEL_3 | PWM_CHANNEL_4 | PWM_CHANNEL_1N | PWM_CHANNEL_2N | PWM_CHANNEL_3N | PWM_CHANNEL_4N );    
}
#endif





/**
* @brief  DPC_ACT_HRTIM_OutEnable Output ENABLE PWM generator by HRTIM 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
#ifdef USE_HRTIM
void DPC_ACT_HRTIM_OutEnable(void)
{     
    /* Enable the generation of the signal on the designated output(s)*/       
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_1);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_2);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_3);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_4);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_1N);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_2N);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_3N);
HAL_HRTIM_WaveformOutputStart(&PWM_Tim1,PWM_CHANNEL_4N); 
}
#endif  



/**
* @brief  DPC_ACT_ADVTIM_OutDisable Output DISABLE PWM generator by Advanced Time 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_ACT_ADVTIM_OutDisable(void)
{     
#ifdef DPC_PWM_2ADVTIM_3CH_3CHX
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim1);
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim2);    
#endif     
  
#ifdef DPC_PWM_1ADVTIM_3CH  
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim1);  
#endif   
  
#ifdef DPC_PWM_1ADVTIM_2CH_1CHX  
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_Tim1);
#endif     
  
  //#ifdef DPC_PWM_xADVTIM_x CH Future dev.  
  //      .... Put here the code 
  //#endif      
      
}





/**
* @brief  DPC_ACT_ADVTIM_OutEnable Output ENABLE PWM generator by Advanced Time 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_ACT_ADVTIM_OutEnable(void)
{
#ifdef DPC_PWM_2ADVTIM_3CH_3CHX
  __HAL_TIM_MOE_ENABLE(&PWM_Tim1);
  __HAL_TIM_MOE_ENABLE(&PWM_Tim2);
#endif 
  
#ifdef DPC_PWM_1ADVTIM_3CH
  __HAL_TIM_MOE_ENABLE(&PWM_Tim1);
#endif  
  
#ifdef DPC_PWM_1ADVTIM_2CH_1CHX  
  __HAL_TIM_MOE_ENABLE(&PWM_Tim1);
#endif     
  
  //#ifdef DPC_PWM_xADVTIM_x CH Future dev.  
  //      .... Put here the code 
  //#endif             

}





/**
* @brief  DPC_ACT_OutDisable Output DISBLE PWM generator 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_ACT_OutDisable(void)
{
#if defined(USE_ADVTIM)
    DPC_ACT_ADVTIM_OutDisable();                                                ///Safe: Disable ADVTIM outputs if enabled
#elif defined(USE_HRTIM)  
    DPC_ACT_HRTIM_OutDisable();                                                 ///Safe: Disable HRTIM outputs if enabled
#else
  #error PWM TIMER  not specified
#endif 
}


/**
* @brief  DPC_ACT_OutEnable Output DISABLE PWM generator 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_ACT_OutEnable(DPC_ACT_PWM_t *tDPC_PWM_loc)
{
  if(tDPC_PWM_loc->DPC_PWM_Status==PWM_Armed){    
#if defined(USE_ADVTIM)
    DPC_ACT_ADVTIM_OutEnable();
#elif defined(USE_HRTIM)
    DPC_ACT_HRTIM_OutEnable();
#else
  #error PWM TIMER  not specified
#endif   
  }
}



/**
* @brief  DPC_ACT_3LTT_BURST_SetOutput Output Enable PWM generator in BURST MODE 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STDES-PFCBIDIR with STM32G4xx microconroller family   
*/
void DPC_ACT_3LTT_BURST_SetOutput(DPC_ACT_PWM_t *tDPC_PWM_loc)
{
  if(tDPC_PWM_loc->DPC_PWM_Status==PWM_Armed){    
#if defined(USE_ADVTIM)
//    DPC_ACT_ADVTIM_OutEnable();
    NOT_SUPPORTED_YET
#elif defined(USE_HRTIM)
  LL_HRTIM_DisableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_1);                   /*! Vertcal leg device output disable*/
  LL_HRTIM_DisableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_2);                   /*! Vertcal leg device output disable*/
  LL_HRTIM_DisableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_3);                   /*! Vertcal leg device output disable*/
  LL_HRTIM_DisableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_4);                   /*! Vertcal leg device output disable*/
  LL_HRTIM_DisableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_5);                   /*! Vertcal leg device output disable*/
  LL_HRTIM_DisableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_6);                   /*! Vertcal leg device output disable*/
  
  LL_HRTIM_EnableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_1N);                   /*! Horizontal leg device output disable*/
  LL_HRTIM_EnableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_2N);                   /*! Horizontal leg device output disable*/
  LL_HRTIM_EnableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_3N);                   /*! Horizontal leg device output disable*/
  LL_HRTIM_EnableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_4N);                   /*! Horizontal leg device output disable*/
  LL_HRTIM_EnableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_5N);                   /*! Horizontal leg device output disable*/
  LL_HRTIM_EnableOutput(PWM_Tim1.Instance,PWM_LL_CHANNEL_6N);                   /*! Horizontal leg device output disable*/
#else
  #error PWM TIMER  not specified
#endif   
  }
}


/**
* @brief  DPC_ACT_SetFault SET Fault state in Actuator module 
*         
* @param  tDPC_PWM_loc: Actuator structure
*
* @retval Null 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_ACT_SetFault(DPC_ACT_PWM_t *tDPC_PWM_loc)
{
  tDPC_PWM_loc->DPC_PWM_Status=PWM_Fault;                                       /*! Set internal actuator paramer in fault mode to prevent future unexpected reactivation of the PWM ouput*/
}





/**
* @brief  DPC_ACT_Start: Start all PWM of the timer but the Output of PWM is Forced in Disable 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_ACT_Start(void)
{
  DPC_ACT_HRTIM_Start();                                                        /*! Start actuator timer*/
  DPC_ACT_HRTIM_OutDisable();                                                   /*! Safe: Disable HRTIM outputs if enabled  */
}




/**
* @brief  DPC_ACT_Init 
*         
* @param  Burst_FreqSet: Burst mode frequency
* @param  PWM_FreqSet: PWM mode frequency
* @param  DPC_PWM_SET: PWM status
* @param  tDPC_PWM_loc: Dynamic Actuator structure
*
* @retval Null 
*
* @note Function valid for STM32G4xx microcontroller family   
*/
void DPC_ACT_Init(uint32_t  Burst_FreqSet,uint32_t  PWM_FreqSet,DPC_ACT_Status_t DPC_PWM_SET, DPC_ACT_PWM_t *tDPC_PWM_loc) 
{

  uint32_t Timers_Clock;                                                        /*! Timer clock variable */

#if defined(USE_ADVTIM) 
  uint32_t PWM_Period;                                                          ///
  uint32_t BURST_PWM_Period;                                                    ///
  uint32_t f_tim_ket_ck;                                                        ///  
  Timers_Clock=HAL_RCC_GetPCLK2Freq();                                          ///
  f_tim_ket_ck=Timers_Clock;                                                    /// Represent frequency Internal clock source (tim_ker_ck) expressed in Hz - see: pag-1063 RM0440 Rev1
  PWM_Period=((f_tim_ket_ck/PWM_FreqSet) - 1);                           ///  uint32_t PWM_Period;          
  BURST_PWM_Period=((f_tim_ket_ck/Burst_FreqSet) - 1);               ///  uint32_t PWM_Period;           
  //  tDPC_PWM_loc->dutyMaxLim=0;                                               /// Adapt to PRESCALER
  //  tDPC_PWM_loc->dutyMinLim=0;                                               /// Adapt to PRESCALER  
#elif defined(USE_HRTIM)   //Null
  float HRTIM_PWM_Period;                                                       /*! PWM mode period variable */  
  float HRTIM_BURST_Period;                                                     /*! Burst mode period variable  */ 
  uint32_t Timers_Prescaler;                                                    /*! Prescaler variable */  
  uint32_t f_HRCK_kHz;                                                          /*! Timer clock frequency variable */ 
  uint32_t HRTIM_Period;                                                        ///
  uint32_t BURST_HRTIM_Period;                                                  ///

    /* Prescaler adaptation*/    
  Timers_Clock=HAL_RCC_GetPCLK2Freq();                                          
  switch(LL_HRTIM_TIM_GetPrescaler(PWM_Tim1.Instance, PWM_LL_IDX_CH_1)){
    case(LL_HRTIM_PRESCALERRATIO_MUL32):
      Timers_Prescaler=32;
      break;
      case(LL_HRTIM_PRESCALERRATIO_MUL16):
        Timers_Prescaler=16;
        break;
        case(LL_HRTIM_PRESCALERRATIO_MUL8):
          Timers_Prescaler=8;
          break;
          case(LL_HRTIM_PRESCALERRATIO_MUL4):
            Timers_Prescaler=4;
            break;
            case(LL_HRTIM_PRESCALERRATIO_MUL2):
              Timers_Prescaler=2;
              break;
              case(LL_HRTIM_PRESCALERRATIO_DIV1):
                Timers_Prescaler=1;
                break;
                case(LL_HRTIM_PRESCALERRATIO_DIV2):
                  Timers_Prescaler=1;
                  break;
                  case(LL_HRTIM_PRESCALERRATIO_DIV4):
                    Timers_Prescaler=1;
                    break;               
  }

  f_HRCK_kHz=(Timers_Clock/1000)*(uint32_t)Timers_Prescaler;                            /// Represent frequency Internal clock source (fHRTIM) expressed in Hz - see: pag-808 RM0440 Rev1
  HRTIM_PWM_Period=((float)f_HRCK_kHz/(float)PWM_FreqSet)*1000.0f;                ///  uint32_t PWM_Period;             
  HRTIM_BURST_Period=((float)f_HRCK_kHz/(float)Burst_FreqSet)*1000.0f;        ///  uint32_t BURST_Period;             
  
  switch(LL_HRTIM_TIM_GetCountingMode(PWM_Tim1.Instance, PWM_LL_IDX_CH_1)){
    case(LL_HRTIM_COUNTING_MODE_UP):
      HRTIM_Period=(uint32_t)(HRTIM_PWM_Period);
      BURST_HRTIM_Period=(uint32_t)(HRTIM_BURST_Period);
      break;
      case(LL_HRTIM_COUNTING_MODE_UP_DOWN):
        HRTIM_Period=(uint32_t)(HRTIM_PWM_Period/2.0f);
        BURST_HRTIM_Period=(uint32_t)(HRTIM_BURST_Period/2.0f);
        break;  
  }
  
    /* Operative range extimation*/    
  switch(LL_HRTIM_TIM_GetPrescaler(PWM_Tim1.Instance, PWM_LL_IDX_CH_1)){
    case(LL_HRTIM_PRESCALERRATIO_MUL32):
      tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x0060);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
      tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x0060);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]         
      break;
      case(LL_HRTIM_PRESCALERRATIO_MUL16):
        tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x0030);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
        tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x0030);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]                 
        break;
        case(LL_HRTIM_PRESCALERRATIO_MUL8):
          tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x0018);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
          tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x0018);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]                 
          break;
          case(LL_HRTIM_PRESCALERRATIO_MUL4):
            tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x000C);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
            tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x000C);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]                 
            break;
            case(LL_HRTIM_PRESCALERRATIO_MUL2):
              tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x0006);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
              tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x0006);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]                 
              break;
              case(LL_HRTIM_PRESCALERRATIO_DIV1):
                tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x0003);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
                tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x0003);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]                 
                break;
                case(LL_HRTIM_PRESCALERRATIO_DIV2):
                  tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x0003);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
                  tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x0003);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]                
                  break;
                  case(LL_HRTIM_PRESCALERRATIO_DIV4):
                    tDPC_PWM_loc->dutyMaxLim=(uint32_t)(HRTIM_Period-0x0003);          /// Period and comppare registers Max values [Ref. Table 207 RM.0440 Rev1]  
                    tDPC_PWM_loc->dutyMinLim=(uint32_t)(0x0003);                       /// Period and comppare registers min values [Ref. Table 207 RM.0440 Rev1]                
                    break;               
  }  
  

    
#endif //USE_XXXTIM   
  
  

#ifdef USE_ADVTIM  
  PWM_Tim2.Init.Prescaler = uwPrescalerValueTIM8;                                  ///
  PWM_Tim2.Init.Period = PWM_Period;                                               ///
  PWM_Tim1.Init.Prescaler = uwPrescalerValueTIM1;                                  ///
  PWM_Tim1.Init.Period = PWM_Period;                                               ///
  if (HAL_TIM_Base_Init(&PWM_Tim1) != HAL_OK){Error_Handler();}                 ///PWM Timer8
  if (HAL_TIM_Base_Init(&PWM_Tim2) != HAL_OK){Error_Handler();}                 ///PWM Timer1  
  tDPC_PWM_loc->PWM_Period=PWM_Period;                                          ///
  tDPC_PWM_loc->BURST_PWM_Period=BURST_PWM_Period;                              /// 
#elif defined(USE_HRTIM)  
  __HAL_HRTIM_SETPERIOD(&PWM_Tim1, 0x06, HRTIM_Period);                 ///  
  __HAL_HRTIM_SETPERIOD(&PWM_Tim1, PWM_IDX_CH_1, HRTIM_Period);                 ///
  __HAL_HRTIM_SETPERIOD(&PWM_Tim1, PWM_IDX_CH_2, HRTIM_Period);                 ///
  __HAL_HRTIM_SETPERIOD(&PWM_Tim1, PWM_IDX_CH_3, HRTIM_Period);                 ///
  __HAL_HRTIM_SETPERIOD(&PWM_Tim1, PWM_IDX_CH_4, HRTIM_Period);                 ///

  tDPC_PWM_loc->PWM_Period=HRTIM_Period;                                        ///
  tDPC_PWM_loc->BURST_PWM_Period=BURST_HRTIM_Period;                            ///     
  
#endif //USE_XXXTIM 
  
#if defined(DPC_DAB_CPWM)
  DPC_ACT_CPWM_PS_Init(tDPC_PWM_loc);
#elif defined(DPC_DAB_ADPPWM)
  DPC_ACT_TpzPWM_PS_Init(tDPC_PWM_loc);
#else 
#error DPC  configuration not specified  // ERROR TO PREVENT NO APPLICATION DEFINITION 
#endif
  

  
  
  tDPC_PWM_loc->DPC_PWM_Status=DPC_PWM_SET;                                     ///
  DPC_ACT_Start();                                                              ///
  DPC_ACT_OutDisable();                                                         ///Safe: Disable PWM outputs if enabled 
}

/**
* @brief  DPC_ACT_CPWM_PS_Init: Conventional PWM Phase shift modulation tecnique 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_ACT_CPWM_PS_Init(DPC_ACT_PWM_t *tDPC_PWM_loc) 
{

  
#if defined(STDES_DABBIDIR)
#ifdef USE_ADVTIM  
//
#elif defined(USE_HRTIM)  
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.0f));                 ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_1,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_2,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_3,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_4,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 /// 
#endif //USE_XXXTIM   
#else
   #error DPC not specified  // ERROR TO PREVENT NO APPLICATION DEFINITION 
#endif  //APPLICATION 

}

/**
* @brief  DPC_ACT_TrzPWM_PS_Init: Trapezoidal PWM Phase shift modulation tecnique 
*         
* @param  Null
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_ACT_TpzPWM_PS_Init(DPC_ACT_PWM_t *tDPC_PWM_loc) 
{

  
#if defined(STDES_DABBIDIR)
#ifdef USE_ADVTIM  
//
#elif defined(USE_HRTIM)  
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5,HRTIM_COMPAREUNIT_2, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.25f));                ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5,HRTIM_COMPAREUNIT_3, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.75f));                ///

  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_1,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 /// Configure DAB leg 50% Duty 
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_2,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 /// Configure DAB leg 50% Duty 
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_3,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 /// Configure DAB leg 50% Duty 
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_4,HRTIM_COMPAREUNIT_1, (uint32_t)((float)tDPC_PWM_loc->PWM_Period*0.5f));                 /// Configure DAB leg 50% Duty 
#endif //USE_XXXTIM   
#else
   #error DPC not specified  // ERROR TO PREVENT NO APPLICATION DEFINITION 
#endif  //APPLICATION 

}

/**
* @brief  DPC_ACT_CPWM_PS_Update:
*         
* @param  
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_ACT_CPWM_PS_Update(DPC_ACT_PWM_t *tDPC_PWM_loc,float PS_Duty)
{
  /*! Local variable*/
  float PS_Timer=0;                     /*! Auxiliary Phase shift duty */
  uint32_t PS_Timer_Tick=0;             /*! Phase shift expressed in Timer tick*/
  uint16_t PWM_PERIOD_COUNTER_INT;      /*!actual Timer period for phase shift scaling*/
  uint16_t HALF_PWM_PERIOD_COUNTER_INT;      /*!actual Timer half period for phase shift scaling*/  
  
  /*! Phase shift saturation PS_Duty=0.5(Positive MAX power) PS_Duty=-0.5(Negative MAX power) */  
  if(PS_Duty>0.5f){PS_Duty=0.5f;}
  else if(PS_Duty<-0.5f){PS_Duty=-0.5f;}
  
  
  /*! Phase shift Duty adaptation if needed*/   
  //  PS_Timer=((-1.0*PS_Duty)+1)*0.5;
  PS_Timer=PS_Duty;  
  
  
  /*! Get actual Timer period for phase shift scaling*/
  PWM_PERIOD_COUNTER_INT=(uint32_t)LL_HRTIM_TIM_GetPeriod(PWM_Tim1.Instance, LL_HRTIM_TIMER_MASTER);
  HALF_PWM_PERIOD_COUNTER_INT=(uint32_t)(0.5f*PWM_PERIOD_COUNTER_INT);
  
  /*! Phase shift mod WRAP funtion*/  
  if(PS_Timer>0){PS_Timer=PS_Timer;}
  else if(PS_Timer<0){PS_Timer=0.5f-PS_Timer;}
  
  
//  PS_Timer_Tick=(uint32_t)(PS_Timer*PWM_PERIOD_COUNTER_INT);
  PS_Timer_Tick=(uint32_t)(PS_Timer*HALF_PWM_PERIOD_COUNTER_INT);  
  
  /*! High Resoltution Timer Saturation fucntion (Refer to RM for more details)*/ 
  if(PS_Timer_Tick>PWM_PERIOD_COUNTER_INT){PS_Timer_Tick=PWM_PERIOD_COUNTER_INT;}
  else if(PS_Timer_Tick<40){PS_Timer_Tick=40;}     
  
  /*! Timer phase shift apply*/  
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_1, (uint32_t)(PS_Timer_Tick));                 ///
  
  
  return;   
}






/**
* @brief  DPC_ACT_DAB_TpzPWM_PS_Update:
*         
* @param  
*
* @retval Null 
*
* @note Function valid for STM32G4xx microconroller family   
*/
void DPC_ACT_DAB_TpzPWM_PS_Update(DPC_ACT_PWM_t *tDPC_PWM_loc,float D1_Duty, float D2_Duty,float PS_Duty)
{
  
  float PS_Timer=0;                     /*! Auxiliary Phase shift duty */
  float D1_Timer=0;                     /*! Auxiliary primary Phase shift duty */
  float D2_Timer=0;                     /*! Auxiliary secondary Phase shift duty */  
  int32_t PS_Timer_Tick=0;              /*! Phase shift expressed in Timer tick*/
  int32_t D1_Timer_Tick=0;              /*! Primary Phase shift expressed in Timer tick*/
  int32_t D2_Timer_Tick=0;              /*! Primary Phase shift expressed in Timer tick*/  
  float PS_Duty_max=1.0;                /*! Primary to secondary Phase shift duty max */  
  float PS_Duty_min=-PS_Duty_max;       /*! Primary to secondary Phase shift duty min  */ 
  float D1_Duty_max=1.0;                /*! Primary Phase shift duty max  */ 
  float D1_Duty_min=-D1_Duty_max;       /*! Primary Phase shift duty min  */ 
  float D2_Duty_max=1.0;                /*! secondary Phase shift duty max  */            
  float D2_Duty_min=-D2_Duty_max;       /*! secondary Phase shift duty min  */ 
  int32_t HB1_Timer_Tick=0;            /*! Primary Phase shift expressed in Timer tick*/
  int32_t HB2_Timer_Tick=0;            /*! Primary Phase shift expressed in Timer tick*/
  int32_t HB3_Timer_Tick=0;            /*! Primary Phase shift expressed in Timer tick*/
  int32_t HB4_Timer_Tick=0;            /*! Primary Phase shift expressed in Timer tick*/  
  uint32_t HRTIM_CU_min=40;             /*! Minimum Compare Unit HRTIM value */ 
  uint16_t PWM_PERIOD_COUNTER_INT;      /*!actual Timer period for phase shift scaling*/
  uint16_t HALF_PWM_PERIOD_COUNTER_INT; /*!actual Timer half period for phase shift scaling*/      
  

  /*! Phase shift saturation PS_Duty=0.5(Positive MAX power) PS_Duty=-0.5(Negative MAX power) */  
  if(PS_Duty>PS_Duty_max){PS_Timer=PS_Duty_max;}
  else if(PS_Duty<PS_Duty_min){PS_Timer=PS_Duty_min;}
  else{PS_Timer=PS_Duty;}  

  /*! Primary side Phase shift saturation D1_Duty=1(180°) D1_Duty=-1(-180°) */   
  if(D1_Duty>D1_Duty_max){D1_Timer=D1_Duty_max;}
  else if(D1_Duty<D1_Duty_min){D1_Timer=D1_Duty_min;}
  else{D1_Timer=D1_Duty;}    

  /*! Secondary side Phase shift saturation D2_Duty=1(180°) D2_Duty=-1(-180°) */   
  if(D2_Duty>D2_Duty_max){D2_Timer=D2_Duty_max;}
  else if(D2_Duty<D2_Duty_min){D2_Timer=D2_Duty_min;}
  else{D2_Timer=D2_Duty;}    
  
  
  /*! Get actual Timer period for phase shift scaling*/
  PWM_PERIOD_COUNTER_INT=(uint32_t)LL_HRTIM_TIM_GetPeriod(PWM_Tim1.Instance, LL_HRTIM_TIMER_MASTER);
  HALF_PWM_PERIOD_COUNTER_INT=(uint32_t)(0.5f*PWM_PERIOD_COUNTER_INT); 
  

  /*! Phase shift duty to Phase shift tick scaling*/  
//  PS_Timer_Tick=(uint32_t)(PS_Timer*PWM_PERIOD_COUNTER_INT);
  PS_Timer_Tick=(int32_t)(PS_Timer*HALF_PWM_PERIOD_COUNTER_INT);  
  D1_Timer_Tick=(int32_t)(D1_Timer*HALF_PWM_PERIOD_COUNTER_INT);  
  D2_Timer_Tick=(int32_t)(D2_Timer*HALF_PWM_PERIOD_COUNTER_INT);    
  
  
  /*! Timer Phase shift mod WRAP funtion*/  
  if(PS_Timer_Tick>0){PS_Timer_Tick=PS_Timer_Tick;}
  else if(PS_Timer_Tick<0){PS_Timer_Tick=PWM_PERIOD_COUNTER_INT+PS_Timer_Tick;}  
  if(D1_Timer_Tick>0){D1_Timer_Tick=D1_Timer_Tick;}
  else if(D1_Timer_Tick<0){D1_Timer_Tick=PWM_PERIOD_COUNTER_INT+D1_Timer_Tick;}  
  if(D2_Timer_Tick>0){D2_Timer_Tick=D2_Timer_Tick;}
  else if(D2_Timer_Tick<0){D2_Timer_Tick=PWM_PERIOD_COUNTER_INT+D2_Timer_Tick;}  
  

  /*! Leg phase shift assignement*/    
  HB1_Timer_Tick=0;
  HB2_Timer_Tick=D1_Timer_Tick;
  HB3_Timer_Tick=PS_Timer_Tick;
  HB4_Timer_Tick=PS_Timer_Tick+D2_Timer_Tick;
  
  
  
  /*! Timer Phase shift mod WRAP funtion*/  
  if(HB1_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB1_Timer_Tick=HB1_Timer_Tick-PWM_PERIOD_COUNTER_INT;}
  else if(HB1_Timer_Tick<0){HB1_Timer_Tick=PWM_PERIOD_COUNTER_INT+HB1_Timer_Tick;} 
  else {HB1_Timer_Tick=HB1_Timer_Tick;}
  if(HB2_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB2_Timer_Tick=HB2_Timer_Tick-PWM_PERIOD_COUNTER_INT;}
  else if(HB2_Timer_Tick<0){HB2_Timer_Tick=PWM_PERIOD_COUNTER_INT+HB2_Timer_Tick;}  
  else {HB2_Timer_Tick=HB2_Timer_Tick;}  
  if(HB3_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB3_Timer_Tick=HB3_Timer_Tick-PWM_PERIOD_COUNTER_INT;}
  else if(HB3_Timer_Tick<0){HB3_Timer_Tick=PWM_PERIOD_COUNTER_INT+HB3_Timer_Tick;} 
  else {HB3_Timer_Tick=HB3_Timer_Tick;}  
  if(HB4_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB4_Timer_Tick=HB4_Timer_Tick-PWM_PERIOD_COUNTER_INT;}
  else if(HB4_Timer_Tick<0){HB4_Timer_Tick=PWM_PERIOD_COUNTER_INT+HB4_Timer_Tick;}  
  else {HB4_Timer_Tick=HB4_Timer_Tick;}    
  

  
  
  /*! High Resolution Timer Saturation fucntion (Refer to RM for more details)*/ 
  if(HB1_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB1_Timer_Tick=PWM_PERIOD_COUNTER_INT;}
  else if(HB1_Timer_Tick<HRTIM_CU_min){HB1_Timer_Tick=HRTIM_CU_min;}
  if(HB2_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB2_Timer_Tick=PWM_PERIOD_COUNTER_INT;}
  else if(HB2_Timer_Tick<HRTIM_CU_min){HB2_Timer_Tick=HRTIM_CU_min;}  
  if(HB3_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB3_Timer_Tick=PWM_PERIOD_COUNTER_INT;}
  else if(HB3_Timer_Tick<HRTIM_CU_min){HB3_Timer_Tick=HRTIM_CU_min;} 
  if(HB4_Timer_Tick>PWM_PERIOD_COUNTER_INT){HB4_Timer_Tick=PWM_PERIOD_COUNTER_INT;}
  else if(HB4_Timer_Tick<HRTIM_CU_min){HB4_Timer_Tick=HRTIM_CU_min;} 
 
  
  /*! Timer phase shift apply*/  
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_1, (uint32_t)(HB2_Timer_Tick));                 ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_2, (uint32_t)(HB3_Timer_Tick));                 ///
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, PWM_IDX_CH_5 ,HRTIM_COMPAREUNIT_3, (uint32_t)(HB4_Timer_Tick));                 ///

  /*! Timer ADC apply*/ 
#if defined(DPC_DAB_CPWM)
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, HRTIM_TIMERINDEX_MASTER,ADC_TRG_CU, (uint32_t)(TRIGGER_TIME_EVENT));       
#elif defined(DPC_DAB_ADPPWM)  
  __HAL_HRTIM_SETCOMPARE(&PWM_Tim1, HRTIM_TIMERINDEX_MASTER,ADC_TRG_CU, (uint32_t)(HB4_Timer_Tick)); 
#endif  
       
  return;   
}

