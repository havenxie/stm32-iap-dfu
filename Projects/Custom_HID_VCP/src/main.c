/**
  ******************************************************************************
  * @file    main.c
  * @author  Custom HID + VCP Application Team
  * @version V4.0.0
  * @date    21-January-2019
  * @brief   HID + VCP Demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
__IO uint8_t PrevXferComplete = 1;
__IO uint32_t TimingDelay = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void SysTick_Init(void);
void Delay_ms(uint16_t nms);
/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
********************************************************************************/
int main(void)
{
  uint8_t thisCnt = 5; 

#if (SUPPORT_DFU == 1)
  NVIC_SetVectorTable(FLASH_BASE, 0x4000);
#if (USE_BKP_SAVE_FLAG == 1)
  RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN , ENABLE);
#endif
#endif 
    
  SysTick_Init();
  Set_System();
	
  while(thisCnt--)
  {
      STM_EVAL_LEDOn(LED1);
      Delay_ms(50);
      STM_EVAL_LEDOff(LED1);
      Delay_ms(50);
  }
  
  USB_Interrupts_Config();
  Set_USBClock();
  USB_Init();

  while (1)
  {
  }
}

#ifndef USE_SYSTICK_NVIC_DELAY
uint8_t  fac_us=0;
uint16_t fac_ms=0;
#endif

/*******************************************************************************
* Function Name  : SysTick_Init
* Description    : .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Init(void)
{
#ifdef USE_SYSTICK_NVIC_DELAY
    SysTick_Config(9000); //cale = 9000 * (1/9000000) = 1ms
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //Freq = 72/8 = 9MHz
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //turn-off SysTick
#else
	fac_us=9;//Freq = 72/8 = 9MHz
	fac_ms=(u16)fac_us*1000;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //Freq = 72/8 = 9MHz
#endif
}			    

/*******************************************************************************
* Function Name  : Delay_ms
* Description    : Inserts a delay time.
* Input          : nms: specifies the delay time length. nms<=1864 
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_ms(uint16_t nms)
{
#ifdef USE_SYSTICK_NVIC_DELAY 
    TimingDelay = nms;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //turn-on SysTick
    while(TimingDelay);
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //turn-off SysTick
#else    
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//load timer value 	
	SysTick->VAL =0x00;           //clear counter
	SysTick->CTRL=0x01 ;          //start counter 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//wait timer arrive   
	SysTick->CTRL=0x00;       //close counter
	SysTick->VAL =0X00;       //clear counter	  	    
#endif    
}   

/*******************************************************************************
* Function Name  : Delay_us
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length. uint is us.
* Output         : None
* Return         : None
*******************************************************************************/	

#ifndef USE_SYSTICK_NVIC_DELAY
void Delay_us(__IO uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //load timer value 		 
	SysTick->VAL=0x00;        //clear counter
	SysTick->CTRL=0x01 ;      //start counter
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//wait timer arrive 
	SysTick->CTRL=0x00;       //close counter
	SysTick->VAL =0X00;       //clear counter
}
#endif

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
