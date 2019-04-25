/**
  ******************************************************************************
  * @file    usb_desc.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptor Header for Virtual COM Port Device
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
#ifndef __USB_DESC_H
#define __USB_DESC_H
#include "stdint.h"

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define HID_DESCRIPTOR_TYPE                     0x21

#define CUSTOMHID_VCP_SIZ_DEVICE_DESC            18
#define CUSTOMHID_VCP_SIZ_CONFIG_DESC           115
#define CUSTOMHID_VCP_SIZ_HID_DESC                9
#define CUSTOMHID_VCP_OFF_HID_DESC               92
//#define CUSTOMHID_VCP_SIZ_REPORT_DESC          23 //for usb dfu
#define CUSTOMHID_VCP_SIZ_REPORT_DESC           158  //for usb hid demo and usb dfu(my self version)
#define CUSTOMHID_VCP_SIZ_STRING_LANGID           4
#define CUSTOMHID_VCP_SIZ_STRING_VENDOR          38
#define CUSTOMHID_VCP_SIZ_STRING_PRODUCT         50
#define CUSTOMHID_VCP_SIZ_STRING_SERIAL          26

#define STANDARD_ENDPOINT_DESC_SIZE               9

#define VIRTUAL_COM_PORT_DATA_SIZE               64
#define VIRTUAL_COM_PORT_INT_SIZE                 8

/* Exported functions ------------------------------------------------------- */
extern const uint8_t CustomHID_VCP_DeviceDescriptor[CUSTOMHID_VCP_SIZ_DEVICE_DESC];
extern const uint8_t CustomHID_VCP_ConfigDescriptor[CUSTOMHID_VCP_SIZ_CONFIG_DESC];

extern const uint8_t CustomHID_VCP_StringLangID[CUSTOMHID_VCP_SIZ_STRING_LANGID];
extern const uint8_t CustomHID_VCP_StringVendor[CUSTOMHID_VCP_SIZ_STRING_VENDOR];
extern const uint8_t CustomHID_VCP_StringProduct[CUSTOMHID_VCP_SIZ_STRING_PRODUCT];
extern uint8_t CustomHID_VCP_StringSerial[CUSTOMHID_VCP_SIZ_STRING_SERIAL];

extern const uint8_t CustomHID_VCP_ReportDescriptor[CUSTOMHID_VCP_SIZ_REPORT_DESC];
#endif /* __USB_DESC_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
