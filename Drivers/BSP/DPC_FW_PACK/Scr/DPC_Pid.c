/**
******************************************************************************
* @file           : PID.c
* @brief          : PI Module
******************************************************************************
 *
 * COPYRIGHT(c) 2021 STMicroelectronics
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
#include "DPC_Pid.h"

/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @brief  DPC_PI_Init: Init PI Data Struct.
* @param  pPI: pointer to a DPC_PI_t  that contains
*         the configuration information and data for the specified PI. 
*
* @param  Init_Val_Kp: Init Kp data
* @param  Init_Val_Ki: Init Ki data
* @param  Init_Val_Ts: Init Ts data
* @param  Init_PIsat_up: Init PI saturation Up data
* @param  Init_PIsat_down: Init PI saturation Down data
* @param  resetValue: FeedForward term of PLL (expressed in Hz i.e. 50Hz or 60Hz)
* 
* @retval None
*
* @note Function valid for STM32G4xx microcontroller family  
*/  
void DPC_PI_Init(DPC_PI_t *pPI,float Init_Val_Kp,float Init_Val_Ki,float Init_Val_Ts,float Init_PIsat_up, float Init_PIsat_down,FlagStatus satPI_toggle_local,FlagStatus antiwindPI_toggle_local,float Antiwindup_Gain_local,float resetValue)
{
  pPI->Kp=Init_Val_Kp;
  pPI->Ki=Init_Val_Ki;
  pPI->Ts=Init_Val_Ts;
  pPI->Integral=0;
  pPI->PIout=0;
  pPI->PIsat_up=Init_PIsat_up;
  pPI->PIsat_down=Init_PIsat_down;
  pPI->error=0;
  pPI->Integralout=0;
  pPI->resetPI=RESET;
  pPI->k0=Init_Val_Kp; //K0=Kp
  pPI->k1=Init_Val_Ki*Init_Val_Ts; //K1=Ki*Ts
  pPI->satPI_toggle=satPI_toggle_local;
  pPI->antiwindPI_toggle=antiwindPI_toggle_local;
  pPI->Antiwindup_Gain=Antiwindup_Gain_local;
  pPI->resetValue=resetValue;
  
  pPI->PI_Q_FxP=16;
  pPI->PIsat_up_s32=(int32_t)(Init_PIsat_up*(1<<pPI->PI_Q_FxP));
  pPI->PIsat_down_s32=(int32_t)(Init_PIsat_down*(1<<pPI->PI_Q_FxP));
  
}

/**
* @brief  DPC_PI: PI function
* @param  Ref: 
* @param  Feed: 
* @param  pPI: pointer to a DPC_PI_t  that contains
*         the configuration information and data for the specified PI. 
*
* @retval float Return output data of PI regulator
*
* @note Function valid for STM32G4xx microcontroller family  
*/
float DPC_PI(float Ref, float Feed , DPC_PI_t *pPI)
{
  pPI->Ref=Ref;
  pPI->Feed=Feed;
  
  pPI->error=(float)Ref-(float)Feed;
  
  if(pPI->resetPI==SET)
  {
    pPI->Integral=pPI->resetValue;
  }
  else
  {
    pPI->Integral=pPI->Integral+(pPI->k1*pPI->error)+pPI->Antiwindup_Term;
  }
  pPI->Integralout=pPI->Integral;
  pPI->PIout=(pPI->k0*pPI->error)+pPI->Integralout;  
  
  //Start Check Saturation
  if (pPI->satPI_toggle==SET){
    //Saturation
    if(    pPI->PIout>pPI->PIsat_up)
    {
      pPI->PIout_sat=pPI->PIsat_up;
    }
    else if(    pPI->PIout<pPI->PIsat_down)
    {
      pPI->PIout_sat=pPI->PIsat_down;
    }
    else {
      pPI->PIout_sat=pPI->PIout;
    }
    
    //Start Check Antiwindup
    if (pPI->antiwindPI_toggle==SET){
      //Saturation
      pPI->Antiwindup_Term=(pPI->PIout_sat-pPI->PIout)*pPI->Antiwindup_Gain;
    }
    else {
      pPI->Antiwindup_Term=0;
    }
    //End Check Antiwindup    
  }
  else {
    pPI->PIout_sat=pPI->PIout;  
    pPI->Antiwindup_Term=0;
  }
  //End Check Saturation
  
  return pPI->PIout_sat;  
}



/**
* @brief  DPC_PI_s32: PI function
* @param  Ref: 
* @param  Feed: 
* @param  pPI: pointer to a DPC_PI_t  that contains
*         the configuration information and data for the specified PI. 
*
* @retval float Return output data of PI regulator
*
* @note Function valid for STM32G4xx microcontroller family  
*/
int32_t DPC_PI_s32(int32_t Ref_s32, int32_t Feed_s32 , DPC_PI_t *pPI)
{
  pPI->Ref_s32=Ref_s32;
  pPI->Feed_s32=Feed_s32;
  
  pPI->error_s32=Ref_s32-Feed_s32;
  
  if(pPI->resetPI==SET)
  {
    pPI->Integral_s32=pPI->resetValue_s32;
  }
  else
  {
    pPI->Integral_s32=pPI->Integral_s32+(pPI->k1_s32*pPI->error_s32)+pPI->Antiwindup_Term_s32;
  }
  pPI->Integralout_s32=pPI->Integral_s32;
  pPI->PIout_s32=(pPI->k0_s32*pPI->error_s32)+pPI->Integralout_s32;  
  
  //Start Check Saturation
  if (pPI->satPI_toggle==SET){
    //Saturation
    if(    pPI->PIout_s32>pPI->PIsat_up_s32)
    {
      pPI->PIout_sat_s32=pPI->PIsat_up_s32;
    }
    else if(    pPI->PIout_s32<pPI->PIsat_down_s32)
    {
      pPI->PIout_sat_s32=pPI->PIsat_down_s32;
    }
    else {
      pPI->PIout_sat_s32=pPI->PIout_s32;
    }
    
    //Start Check Antiwindup
    if (pPI->antiwindPI_toggle==SET){
      //Saturation
      pPI->Antiwindup_Term_s32=(pPI->PIout_sat_s32-pPI->PIout_s32)*pPI->Antiwindup_Gain_s32;
    }
    else {
      pPI->Antiwindup_Term_s32=0;
    }
    //End Check Antiwindup    
  }
  else {
    pPI->PIout_sat_s32=pPI->PIout_s32;  
    pPI->Antiwindup_Term_s32=0;
  }
  //End Check Saturation
  pPI->PIout_sat_s32=pPI->PIout_sat_s32>>pPI->PI_Q_FxP; 
  
  return pPI->PIout_sat_s32;  
}


/**
* @brief  Reset PI value
* @param  pPI: pointer to a DPC_PI_t  that contains
*         the configuration information and data for the specified PI. 
*
* @retval None
*
* @note Function valid for STM32G4xx microcontroller family  
*/ 
void PI_RESET(DPC_PI_t *pPI)
{
  pPI->Integral = pPI->resetValue;
}

