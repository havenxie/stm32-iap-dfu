/**
  ******************************************************************************
  * @file    usb_prop.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   All processing related to Virtual Com Port Demo
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
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"


/* Private typedef -----------------------------------------------------------*/ 
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t ProtocolValue;
__IO uint8_t EXTI_Enable;
__IO uint8_t Request = 0;
uint8_t Report_Buf[2];   
/* -------------------------------------------------------------------------- */
/*  Structures initializations */
/* -------------------------------------------------------------------------- */

DEVICE Device_Table =
  {
    EP_NUM,
    1
  };
  
LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
  };
  
DEVICE_PROP Device_Property =
  {
    CustomHID_VCP_init,
    CustomHID_VCP_Reset,
    CustomHID_VCP_Status_In,
    CustomHID_VCP_Status_Out,
    CustomHID_VCP_Data_Setup,
    CustomHID_VCP_NoData_Setup,
    CustomHID_VCP_Get_Interface_Setting,
    CustomHID_VCP_GetDeviceDescriptor,
    CustomHID_VCP_GetConfigDescriptor,
    CustomHID_VCP_GetStringDescriptor,
    0,
    0x40 /*MAX PACKET SIZE*/
  };

USER_STANDARD_REQUESTS User_Standard_Requests =
  {
    CustomHID_VCP_GetConfiguration,
    CustomHID_VCP_SetConfiguration,
    CustomHID_VCP_GetInterface,
    CustomHID_VCP_SetInterface,
    CustomHID_VCP_GetStatus,
    CustomHID_VCP_ClearFeature,
    CustomHID_VCP_SetEndPointFeature,
    CustomHID_VCP_SetDeviceFeature,
    CustomHID_VCP_SetDeviceAddress
  };

ONE_DESCRIPTOR Device_Descriptor =
  {
    (uint8_t*)CustomHID_VCP_DeviceDescriptor,
    CUSTOMHID_VCP_SIZ_DEVICE_DESC
  };

ONE_DESCRIPTOR Config_Descriptor =
  {
    (uint8_t*)CustomHID_VCP_ConfigDescriptor,
    CUSTOMHID_VCP_SIZ_CONFIG_DESC
  };
  
ONE_DESCRIPTOR CustomHID_VCP_Report_Descriptor =
  {
    (uint8_t *)CustomHID_VCP_ReportDescriptor,
    CUSTOMHID_VCP_SIZ_REPORT_DESC
  };

ONE_DESCRIPTOR CustomHID_VCP_Hid_Descriptor =
  {
    (uint8_t*)CustomHID_VCP_ConfigDescriptor + CUSTOMHID_VCP_OFF_HID_DESC,
    CUSTOMHID_VCP_SIZ_HID_DESC
  };
  
ONE_DESCRIPTOR String_Descriptor[4] =
  {
    {(uint8_t*)CustomHID_VCP_StringLangID, CUSTOMHID_VCP_SIZ_STRING_LANGID},
    {(uint8_t*)CustomHID_VCP_StringVendor, CUSTOMHID_VCP_SIZ_STRING_VENDOR},
    {(uint8_t*)CustomHID_VCP_StringProduct, CUSTOMHID_VCP_SIZ_STRING_PRODUCT},
    {(uint8_t*)CustomHID_VCP_StringSerial, CUSTOMHID_VCP_SIZ_STRING_SERIAL}
  };

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*CustomHID_VCP_SetReport_Feature function prototypes*/
uint8_t *CustomHID_VCP_SetReport_Feature(uint16_t Length);

/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : CustomHID_VCP_init.
* Description    : Virtual COM Port Mouse init routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CustomHID_VCP_init(void)
{

  /* Update the serial number string descriptor with the data from the unique
  ID*/
  Get_SerialNum();

  pInformation->Current_Configuration = 0;

  /* Connect the device */
  PowerOn();

  /* Perform basic device initialization operations */
  USB_SIL_Init();

  /* configure the USART to the default settings */
  USART_Config_Default();

  /* Enable USB interrupts */
  USB_Interrupts_Config();
    
  bDeviceState = UNCONNECTED;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_Reset.
* Description    : Custom HID reset routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CustomHID_VCP_Reset(void)
{
  /* Set CustomHID_VCP_DEVICE as not configured */
  pInformation->Current_Configuration = 0;
  pInformation->Current_Interface = 1;/*the default Interface*/
  
  /* Current Feature initialization */
  pInformation->Current_Feature = CustomHID_VCP_ConfigDescriptor[7];
 
  SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
  SetEPType(ENDP0, EP_CONTROL);
  SetEPTxStatus(ENDP0, EP_TX_STALL);
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);
  Clear_Status_Out(ENDP0);
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
  SetEPRxValid(ENDP0);

  /* Initialize Endpoint 1 */
  SetEPType(ENDP1, EP_INTERRUPT);
  SetEPTxAddr(ENDP1, ENDP1_TXADDR);
  SetEPRxAddr(ENDP1, ENDP1_RXADDR);
  SetEPTxCount(ENDP1, 2);
  SetEPRxCount(ENDP1, 2);
  SetEPRxStatus(ENDP1, EP_RX_VALID);
  SetEPTxStatus(ENDP1, EP_TX_NAK);

  /* Initialize Endpoint 2 */
  SetEPType(ENDP2, EP_INTERRUPT);
  SetEPTxAddr(ENDP2, ENDP2_TXADDR);
  SetEPRxStatus(ENDP2, EP_RX_DIS);
  SetEPTxStatus(ENDP2, EP_TX_NAK);

  /* Initialize Endpoint 3 */
  SetEPType(ENDP3, EP_BULK);
  SetEPRxAddr(ENDP3, ENDP3_RXADDR);
  SetEPRxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);
  SetEPRxStatus(ENDP3, EP_RX_VALID);
  SetEPTxStatus(ENDP3, EP_TX_DIS);

  /* Initialize Endpoint 4 */
  SetEPType(ENDP4, EP_BULK);
  SetEPTxAddr(ENDP4, ENDP4_TXADDR);
  SetEPTxStatus(ENDP4, EP_TX_NAK);
  SetEPRxStatus(ENDP4, EP_RX_DIS);
  
  /* Set this device to response on default address */
  SetDeviceAddress(0);
  
  bDeviceState = ATTACHED;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_SetConfiguration.
* Description    : Update the device state to configured and command the ADC 
*                  conversion.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CustomHID_VCP_SetConfiguration(void)
{
  if (pInformation->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
    
    /* Start ADC Software Conversion */ 
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)|| defined(STM32F37X)
    ADC_SoftwareStartConv(ADC1);
#elif defined (STM32F30X)
    ADC_StartConversion(ADC1);
#else
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
#endif /* STM32L1XX_XD */
  }
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_SetConfiguration.
* Description    : Update the device state to addressed.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CustomHID_VCP_SetDeviceAddress (void)
{
  bDeviceState = ADDRESSED;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_Status_In.
* Description    : Virtual COM Port Status In Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CustomHID_VCP_Status_In(void)
{
  BitAction Led_State;
  if (Request == SET_LINE_CODING)
  {
    USART_Config();
    Request = 0;
  }
    
  if(Report_Buf[0] == 0x80 && Report_Buf[1] == 0x55)
  {
#if (USE_BKP_SAVE_FLAG == 1)
    PWR->CR |= PWR_CR_DBP;
    BKP_WriteBackupRegister(IAP_FLAG_ADDR, 0x5A5A);
    PWR->CR &= ~PWR_CR_DBP;
#endif
     __set_FAULTMASK(1); 
     USB_Cable_Config(DISABLE);
     NVIC_SystemReset();
  }  
  
  Led_State = (Report_Buf[1] ==  Bit_SET) ? Bit_SET : Bit_RESET;
  switch (Report_Buf[0])  
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
  
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_Status_Out
* Description    : Virtual COM Port Status OUT Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CustomHID_VCP_Status_Out (void)
{
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_Data_Setup
* Description    : Handle the data class specific requests.
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT CustomHID_VCP_Data_Setup(uint8_t RequestNo)
{
    uint8_t *(*CopyRoutine)(uint16_t);

    CopyRoutine = NULL;

    if (RequestNo == GET_LINE_CODING)
    {
      if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
      {
        CopyRoutine = CustomHID_VCP_GetLineCoding;
      }
    }
    else if (RequestNo == SET_LINE_CODING)
    {
      if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
      {
        CopyRoutine = CustomHID_VCP_SetLineCoding;
      }
      Request = SET_LINE_CODING;
    }
    else if ((RequestNo == GET_DESCRIPTOR) 
		&& (Type_Recipient == (STANDARD_REQUEST | INTERFACE_RECIPIENT))
	      )
    {
      //if (Type_Recipient == (STANDARD_REQUEST | INTERFACE_RECIPIENT))
      //{
        if (pInformation->USBwValue1 == REPORT_DESCRIPTOR)
        {
          CopyRoutine = CustomHID_VCP_GetReportDescriptor;
        }
        else if (pInformation->USBwValue1 == HID_DESCRIPTOR_TYPE)
        {
          CopyRoutine = CustomHID_VCP_GetHIDDescriptor;
        }
      //}
    }
    else if ( (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) )
    {         
      switch( RequestNo )
      {
        case GET_PROTOCOL:
            CopyRoutine = CustomHID_VCP_GetProtocolValue;
            break;
        case SET_REPORT:
            CopyRoutine = CustomHID_VCP_SetReport_Feature;
            Request = SET_REPORT;
            break;
        default:
            break;
      }
    }
    
  if (CopyRoutine == NULL)
  {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*CopyRoutine)(0);
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_NoData_Setup.
* Description    : handle the no data class specific requests.
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT CustomHID_VCP_NoData_Setup(uint8_t RequestNo)
{  
  if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
  {
    if (RequestNo == SET_COMM_FEATURE)
    {
      return USB_SUCCESS;
    }
    else if (RequestNo == SET_CONTROL_LINE_STATE)
    {
      return USB_SUCCESS;
    }
	else if(RequestNo == SET_PROTOCOL)
	{
		return CustomHID_VCP_SetProtocol();
	}
  }

  return USB_UNSUPPORT;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_SetReport_Feature
* Description    : Set Feature request handling
* Input          : Length.
* Output         : None.
* Return         : Buffer
*******************************************************************************/
uint8_t *CustomHID_VCP_SetReport_Feature(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = 2;
    return NULL;
  }
  else
  {
    return Report_Buf;
  }
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_GetDeviceDescriptor.
* Description    : Gets the device descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the device descriptor.
*******************************************************************************/
uint8_t *CustomHID_VCP_GetDeviceDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_GetConfigDescriptor.
* Description    : get the configuration descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *CustomHID_VCP_GetConfigDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_GetStringDescriptor
* Description    : Gets the string descriptors according to the needed index
* Input          : Length.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
uint8_t *CustomHID_VCP_GetStringDescriptor(uint16_t Length)
{
  uint8_t wValue0 = pInformation->USBwValue0;
  if (wValue0 > 4)
  {
    return NULL;
  }
  else
  {
    return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
  }
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_GetReportDescriptor.
* Description    : Gets the HID report descriptor.
* Input          : Length
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *CustomHID_VCP_GetReportDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &CustomHID_VCP_Report_Descriptor);
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_GetHIDDescriptor.
* Description    : Gets the HID descriptor.
* Input          : Length
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *CustomHID_VCP_GetHIDDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &CustomHID_VCP_Hid_Descriptor);
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_Get_Interface_Setting.
* Description    : test the interface and the alternate setting according to the
*                  supported one.
* Input1         : uint8_t: Interface : interface number.
* Input2         : uint8_t: AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
RESULT CustomHID_VCP_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting)
{
  if (AlternateSetting > 0)
  {
    return USB_UNSUPPORT;
  }
  else if (Interface > 1)//Why?
  {
    return USB_UNSUPPORT;
  }
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_SetProtocol
* Description    : Joystick Set Protocol request routine.
* Input          : None.
* Output         : None.
* Return         : USB SUCCESS.
*******************************************************************************/
RESULT CustomHID_VCP_SetProtocol(void)
{
  uint8_t wValue0 = pInformation->USBwValue0;
  ProtocolValue = wValue0;
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_GetProtocolValue
* Description    : get the protocol value
* Input          : Length.
* Output         : None.
* Return         : address of the protocol value.
*******************************************************************************/
uint8_t *CustomHID_VCP_GetProtocolValue(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = 1;
    return NULL;
  }
  else
  {
    return (uint8_t *)(&ProtocolValue);
  }
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_GetLineCoding.
* Description    : send the linecoding structure to the PC host.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *CustomHID_VCP_GetLineCoding(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(uint8_t *)&linecoding;
}

/*******************************************************************************
* Function Name  : CustomHID_VCP_SetLineCoding.
* Description    : Set the linecoding structure fields.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *CustomHID_VCP_SetLineCoding(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(uint8_t *)&linecoding;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

