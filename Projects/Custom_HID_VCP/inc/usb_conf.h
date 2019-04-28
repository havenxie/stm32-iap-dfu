/**
  ******************************************************************************
  * @file    usb_conf.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Virtual COM Port Demo configuration  header
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF_H
#define __USB_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* External variables --------------------------------------------------------*/

/*-------------------------------------------------------------*/
/* EP_NUM */
/* defines how many endpoints are used by the device */
/*-------------------------------------------------------------*/

#define EP_NUM                          (5)

/*-------------------------------------------------------------*/
/* --------------   Buffer Description Table  -----------------*/
/*-------------------------------------------------------------*/
/* buffer table base address */
/* buffer table base address */
#define BTABLE_ADDRESS      (0x00)

///* EP0  */
///* rx/tx buffer base address */
//#define ENDP0_RXADDR        (0x40)
//#define ENDP0_TXADDR        (0x80)

///* EP1  */
///* tx/tx buffer base address */
//#define ENDP1_RXADDR        (0xC0)
//#define ENDP1_TXADDR        (0xC4)

///* EP2  */
///* tx buffer base address */
//#define ENDP2_TXADDR        (0xC8)

///* EP3  */
///* rx buffer base address */
//#define ENDP3_RXADDR        (0xD8)

///* EP4  */
///* tx buffer base address */
//#define ENDP4_TXADDR        (0x118)
#define BASEADDR_DATA (BTABLE_ADDRESS + 0x00000040)
// ENP0
#define ENDP0_PACKETSIZE 0x40
#define ENDP0_RXADDR  BASEADDR_DATA
#define ENDP0_TXADDR (ENDP0_RXADDR + ENDP0_PACKETSIZE)
// ENP1
#define ENDP1_PACKETSIZE 0x02
#define ENDP1_RXADDR (ENDP0_TXADDR + ENDP0_PACKETSIZE)
#define ENDP1_TXADDR (ENDP1_RXADDR + ENDP1_PACKETSIZE)
// ENP2
#define ENDP2_PACKETSIZE 0x08
#define ENDP2_RXADDR (ENDP1_TXADDR + ENDP1_PACKETSIZE)
#define ENDP2_TXADDR (ENDP2_RXADDR + ENDP2_PACKETSIZE)
// ENP3
#define ENDP3_PACKETSIZE 0x40
#define ENDP3_RXADDR (ENDP2_TXADDR + ENDP2_PACKETSIZE)
#define ENDP3_TXADDR (ENDP3_RXADDR + ENDP3_PACKETSIZE)
// ENP4
#define ENDP4_PACKETSIZE 0x40
#define ENDP4_RXADDR (ENDP3_TXADDR + ENDP3_PACKETSIZE)
#define ENDP4_TXADDR (ENDP4_RXADDR + ENDP4_PACKETSIZE)
// ENP5
#define ENDP5_PACKETSIZE 0x40
#define ENDP5_RXADDR (ENDP4_TXADDR + ENDP4_PACKETSIZE)
#define ENDP5_TXADDR (ENDP5_RXADDR + ENDP5_PACKETSIZE)
// ENP6
#define ENDP6_PACKETSIZE 0x40
#define ENDP6_RXADDR (ENDP5_TXADDR + ENDP5_PACKETSIZE)
#define ENDP6_TXADDR (ENDP6_RXADDR + ENDP6_PACKETSIZE)
// ENP7
#define ENDP7_PACKETSIZE 0x40
#define ENDP7_RXADDR (ENDP6_TXADDR + ENDP6_PACKETSIZE)
#define ENDP7_TXADDR (ENDP7_RXADDR + ENDP7_PACKETSIZE)
/*-------------------------------------------------------------*/
/* -------------------   ISTR events  -------------------------*/
/*-------------------------------------------------------------*/
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CNTR_CTRM  | CNTR_WKUPM | CNTR_SUSPM | CNTR_ERRM  | CNTR_SOFM \
                 | CNTR_ESOFM | CNTR_RESETM )

/*#define CTR_CALLBACK*/
/*#define DOVR_CALLBACK*/
/*#define ERR_CALLBACK*/
/*#define WKUP_CALLBACK*/
/*#define SUSP_CALLBACK*/
/*#define RESET_CALLBACK*/
#define SOF_CALLBACK
/*#define ESOF_CALLBACK*/
/* CTR service routines */
/* associated to defined endpoints */
//#define  EP1_IN_Callback   NOP_Process
#define  EP2_IN_Callback   NOP_Process
#define  EP3_IN_Callback   NOP_Process
//#define  EP4_IN_Callback   NOP_Process
#define  EP5_IN_Callback   NOP_Process
#define  EP6_IN_Callback   NOP_Process
#define  EP7_IN_Callback   NOP_Process

//#define  EP1_OUT_Callback   NOP_Process
#define  EP2_OUT_Callback   NOP_Process
/*#define  EP3_OUT_Callback   NOP_Process*/
#define  EP4_OUT_Callback   NOP_Process
#define  EP5_OUT_Callback   NOP_Process
#define  EP6_OUT_Callback   NOP_Process
#define  EP7_OUT_Callback   NOP_Process

#endif /* __USB_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
