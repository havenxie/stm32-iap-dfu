/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
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
#include "usb_istr.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Receive_Buffer Report_Buf //Open this define to resolve the impact of opening the serial port on the LEDstate of the BUG.
/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
extern  uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_out;
extern uint32_t USART_Rx_length;
extern uint8_t  USB_Tx_State;
extern uint8_t Report_Buf[2];

#ifndef Receive_Buffer
uint8_t Receive_Buffer[2];
#endif
extern __IO uint8_t PrevXferComplete;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{
  BitAction Led_State;

  /* Read received data (2 bytes) */  
  USB_SIL_Read(EP1_OUT, Receive_Buffer);
      
  Led_State = (Receive_Buffer[1] ==  Bit_SET) ? Bit_SET : Bit_RESET;
  switch (Receive_Buffer[0])  
  {
    /*Change LED's status according to the host report*/
    case 1: /* Led 1 */ 
      Led_State ? STM_EVAL_LEDOn(LED1) : STM_EVAL_LEDOff(LED1);
      break;
    case 2: /* Led 2 */    
      Led_State ? STM_EVAL_LEDOn(LED2) : STM_EVAL_LEDOff(LED2);
      break;
    case 3: /* Led 3 */    
      Led_State ? STM_EVAL_LEDOn(LED3) : STM_EVAL_LEDOff(LED3);
      break;
    case 4: /* Led 4 */    
      Led_State ? STM_EVAL_LEDOn(LED4) : STM_EVAL_LEDOff(LED4);
      break;
    default:
      STM_EVAL_LEDOff(LED1);
      STM_EVAL_LEDOff(LED2);
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED4); 
      break;
  }
 
  SetEPRxStatus(ENDP1, EP_RX_VALID);

}

/*******************************************************************************
* Function Name  : EP1_IN_Callback.
* Description    : EP1 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  PrevXferComplete = 1;
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  uint16_t USB_Rx_Cnt;
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the USART Xfer */
  
  USB_To_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);
 
  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
}

/*******************************************************************************
* Function Name  : EP4_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP4_IN_Callback (void)
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
  if (USB_Tx_State == 1)
  {
    if (USART_Rx_length == 0) 
    {
      USB_Tx_State = 0;
    }
    else 
    {
      if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE){
        USB_Tx_ptr = USART_Rx_ptr_out;
        USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
        
        USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
        USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;    
      }
      else 
      {
        USB_Tx_ptr = USART_Rx_ptr_out;
        USB_Tx_length = USART_Rx_length;
        
        USART_Rx_ptr_out += USART_Rx_length;
        USART_Rx_length = 0;
      }
      UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP4_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP4, USB_Tx_length);
      SetEPTxValid(ENDP4); 
    }
  }
}

/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
  static uint32_t FrameCount = 0;
  
  if(bDeviceState == CONFIGURED)
  {
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
    {
      /* Reset the frame counter */
      FrameCount = 0;
      
      /* Check the data to be sent through IN pipe */
      Handle_USBAsynchXfer();
    }
  }  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

