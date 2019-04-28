/**
  ******************************************************************************
  * @file    usb_prop.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   All processing related to Virtual COM Port Demo (Endpoint 0)
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
#ifndef __usb_prop_H
#define __usb_prop_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint32_t bitrate;
  uint8_t format;
  uint8_t paritytype;
  uint8_t datatype;
}LINE_CODING;

typedef enum _HID_REQUESTS
{
  GET_REPORT = 1,
  GET_IDLE,
  GET_PROTOCOL,

  SET_REPORT = 9,
  SET_IDLE,
  SET_PROTOCOL
} HID_REQUESTS;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

#define CustomHID_VCP_GetConfiguration          NOP_Process
//#define CustomHID_VCP_SetConfiguration          NOP_Process
#define CustomHID_VCP_GetInterface              NOP_Process
#define CustomHID_VCP_SetInterface              NOP_Process
#define CustomHID_VCP_GetStatus                 NOP_Process
#define CustomHID_VCP_ClearFeature              NOP_Process
#define CustomHID_VCP_SetEndPointFeature        NOP_Process
#define CustomHID_VCP_SetDeviceFeature          NOP_Process
//#define CustomHID_VCP_SetDeviceAddress          NOP_Process

#define SEND_ENCAPSULATED_COMMAND   0x00
#define GET_ENCAPSULATED_RESPONSE   0x01
#define SET_COMM_FEATURE            0x02
#define GET_COMM_FEATURE            0x03
#define CLEAR_COMM_FEATURE          0x04
#define SET_LINE_CODING             0x20
#define GET_LINE_CODING             0x21
#define SET_CONTROL_LINE_STATE      0x22
#define SEND_BREAK                  0x23

#define REPORT_DESCRIPTOR                  0x22

/* Exported functions ------------------------------------------------------- */
void CustomHID_VCP_init(void);
void CustomHID_VCP_Reset(void);
void CustomHID_VCP_SetConfiguration(void);
void CustomHID_VCP_SetDeviceAddress (void);
void CustomHID_VCP_Status_In (void);
void CustomHID_VCP_Status_Out (void);
RESULT CustomHID_VCP_Data_Setup(uint8_t);
RESULT CustomHID_VCP_NoData_Setup(uint8_t);
RESULT CustomHID_VCP_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting);
uint8_t *CustomHID_VCP_GetDeviceDescriptor(uint16_t );
uint8_t *CustomHID_VCP_GetConfigDescriptor(uint16_t);
uint8_t *CustomHID_VCP_GetStringDescriptor(uint16_t);
uint8_t *CustomHID_VCP_GetLineCoding(uint16_t Length);
uint8_t *CustomHID_VCP_SetLineCoding(uint16_t Length);
uint8_t *CustomHID_VCP_GetReportDescriptor(uint16_t Length);
uint8_t *CustomHID_VCP_GetHIDDescriptor(uint16_t Length);
uint8_t *CustomHID_VCP_GetProtocolValue(uint16_t Length);
uint8_t *CustomHID_VCP_SetReport_Feature(uint16_t Length);
RESULT CustomHID_VCP_SetProtocol(void);

#endif /* __usb_prop_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

