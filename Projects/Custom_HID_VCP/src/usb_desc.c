/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptors for Virtual Com Port Demo
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
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t CustomHID_VCP_DeviceDescriptor[CUSTOMHID_VCP_SIZ_DEVICE_DESC] =
  {
    0x12,   /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,     /* bDescriptorType */
    0x00,
    0x02,   /* bcdUSB = 2.00 */
    0xEF,
    0x02,
    0x01,
    0x40,   /* bMaxPacketSize0 */
    0x83,
    0x04,   /* idVendor = 0x0483 */
    0x51,
    0x57,   /* idProduct = 0x5751 */
    0x00,
    0x02,   /* bcdDevice = 2.00 */
    1,              /* Index of string descriptor describing manufacturer */
    2,              /* Index of string descriptor describing product */
    3,              /* Index of string descriptor describing the device's serial number */
    0x01    /* bNumConfigurations */
  };

  /*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t CustomHID_VCP_ConfigDescriptor[CUSTOMHID_VCP_SIZ_CONFIG_DESC] =
  {
    /*Configuration Descriptor*/
    0x09,   /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */
    CUSTOMHID_VCP_SIZ_CONFIG_DESC,       /* wTotalLength:no of returned bytes */
    0x00,
    0x03,   /* bNumInterfaces: 3 interface */
    0x01,   /* bConfigurationValue: Configuration value */
    0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
    0xC0,   /* bmAttributes: self powered */
    0x32,   /* MaxPower 100 mA */
	  
    /* Interface Association Descriptor(IAD Descriptor)  */ 
    /* 9 */
    0x08,   /*   bLength  */
    0x0B,   /*   bDescriptorType*/
    0x00,   /*    bFirstInterface*/
    0x02,   /*     bInterfaceCount*/
    0x02,   /*     bFunctionClass --CDC*/
    0x02,   /*     bFunctionSubClass*/
    0x01,   /*    bFunctionProtocoll*/
    0x00,   /*   iFunction */

    /*Interface Descriptor*/
  	/* 17 */
    0x09,   /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
    /* Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x00,   /* iInterface: */
    /*Header Functional Descriptor*/
	/* 26 */
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,
    /*Call Management Functional Descriptor*/
	/* 31 */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */
    /*ACM Functional Descriptor*/
	/* 36 */
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */
    /*Union Functional Descriptor*/
	/* 40 */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */
    /*Endpoint 2 Descriptor*/
	/* 45 */
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x82,   /* bEndpointAddress: (IN2) */
    0x03,   /* bmAttributes: Interrupt */
    VIRTUAL_COM_PORT_INT_SIZE,      /* wMaxPacketSize: */
    0x00,
    0xFF,   /* bInterval: */
    
    /*Data class interface descriptor*/
	/* 52 */
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
    0x01,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */
    /*Endpoint 3 Descriptor*/
	/* 61 */
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x03,   /* bEndpointAddress: (OUT3) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00,   /* bInterval: ignore for Bulk transfer */
    /*Endpoint 1 Descriptor*/
	/* 68 */
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x84,   /* bEndpointAddress: (IN4) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00,    /* bInterval */
	
    /*********************************IAD Descriptor*********************************/
	/* 75 */
    0x08,    //?????
    0x0B,    //IAD?????
    0x02,    //bFirstInterface
    0x01,    //bInferfaceCount
    0x03,    //bFunctionClass:HID
    0x00,    //bFunctionSubClass
    0x00,    //bFunctionProtocol
    0x02,    //iFunction
	
    /************** Descriptor of Custom HID interface ****************/
    /* 83 */
    0x09,         /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type */
    0x02,         /* bInterfaceNumber: Number of Interface */
    0x00,         /* bAlternateSetting: Alternate setting */
    0x02,         /* bNumEndpoints */
    0x03,         /* bInterfaceClass: HID */
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0x00,            /* iInterface: Index of string descriptor */
    /******************** Descriptor of Custom HID HID ********************/
    /* 92 */
    0x09,         /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
    0x10,         /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,         /* bCountryCode: Hardware target country */
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         /* bDescriptorType */
    CUSTOMHID_VCP_SIZ_REPORT_DESC,/* wItemLength: Total length of Report descriptor */
    0x00,
    /******************** Descriptor of Custom HID endpoints ******************/
    /* 101 */
    0x07,          /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: */

    0x81,          /* bEndpointAddress: Endpoint Address (IN) */
    0x03,          /* bmAttributes: Interrupt endpoint */
    0x02,          /* wMaxPacketSize: 2 Bytes max */
    0x00,
    0x20,          /* bInterval: Polling Interval (32 ms) */
    /* 108 */
    0x07,	/* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: */
			/*	Endpoint descriptor type */
    0x01,	/* bEndpointAddress: */
			/*	Endpoint Address (OUT) */
    0x03,	/* bmAttributes: Interrupt endpoint */
    0x02,	/* wMaxPacketSize: 2 Bytes max  */
    0x00,
    0x20	/* bInterval: Polling Interval (32 ms) */
    /* 115 */
  };
  
   /* CustomHID_VCP_ConfigDescriptor */
const uint8_t CustomHID_VCP_ReportDescriptor[CUSTOMHID_VCP_SIZ_REPORT_DESC] =
  { 
	  //for usb dfu
      /* USER CODE BEGIN 0 */      
//      0x06, 0x00, 0xFF, /* USAGE_PAGE (Vendor Page: 0xFF00) */
//      0x09, 0x01, /* USAGE (Demo Kit) */
//      0xa1, 0x00, /* COLLECTION (Physical) */
//	  
//      0x85, 0x80, /* REPORT_ID (128) */ 
//      0x09, 0x55, /*USAGE (LED 1)  */
//      0x15, 0x00, /* LOGICAL_MINIMUM (0) */
//      0x26, 0xFF, 0x00, /* LOGICAL_MAXIMUM (255) */
//      0x75, 0x08, /* REPORT_SIZE (8 bits) */
//      0x95, 0x01, /* REPORT_COUNT (1) */
//      0xB1, 0x82, /* FEATURE (Data,Var,Abs,Vol */
//      /* USER CODE END 0 */
//      0xC0 /* END_COLLECTION */ 

//    0xc0 	          /*     END_COLLECTION	             */
		//for usb hid demo and usb dfu(my self version)
    0x06, 0x00, 0xFF,      /* USAGE_PAGE (Vendor Page: 0xFF00) *///2 global                    
    0x09, 0x01,            /* USAGE (Demo Kit)               */  //3 local   
    0xa1, 0x01,            /* COLLECTION (Application)       */  //1 main
       
    0x85, 0x80, /* REPORT_ID (128) */ 
    0x09, 0x55, /*USAGE (LED 1)  */
    0x15, 0x00, /* LOGICAL_MINIMUM (0) */
    0x26, 0xFF, 0x00, /* LOGICAL_MAXIMUM (255) */
    0x75, 0x08, /* REPORT_SIZE (8 bits) */
    0x95, 0x01, /* REPORT_COUNT (1) */
    0xB1, 0x82, /* FEATURE (Data,Var,Abs,Vol */    
    /* 6 */
    
    /* Led 1 */        
    0x85, 0x01,            /*     REPORT_ID (1)		         */  //2 global
    0x09, 0x01,            /*     USAGE (LED 1)	             */  //3 local
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */  //2 global        
    0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */  //2 global         
    0x75, 0x08,            /*     REPORT_SIZE (8)            */  //2 global      
    0x95, 0x01,            /*     REPORT_COUNT (1)           */  //2 global    
    0xB1, 0x82,             /*    FEATURE (Data,Var,Abs,Vol) */  //1 main

    0x85, 0x01,            /*     REPORT_ID (1)              */  //2 global
    0x09, 0x01,            /*     USAGE (LED 1)              */  //3 local
    0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */  //1 main
    /* 26 */
    
    /* Led 2 */
    0x85, 0x02,            /*     REPORT_ID 2		     */
    0x09, 0x02,            /*     USAGE (LED 2)	             */
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
    0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */           
    0x75, 0x08,            /*     REPORT_SIZE (8)            */        
    0x95, 0x01,            /*     REPORT_COUNT (1)           */       
    0xB1, 0x82,             /*    FEATURE (Data,Var,Abs,Vol) */     

    0x85, 0x02,            /*     REPORT_ID (2)              */
    0x09, 0x02,            /*     USAGE (LED 2)              */
    0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
    /* 46 */
    
    /* Led 3 */        
    0x85, 0x03,            /*     REPORT_ID (3)		     */
    0x09, 0x03,            /*     USAGE (LED 3)	             */
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
    0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */           
    0x75, 0x08,            /*     REPORT_SIZE (8)            */        
    0x95, 0x01,            /*     REPORT_COUNT (1)           */       
    0xB1, 0x82,             /*    FEATURE (Data,Var,Abs,Vol) */     

    0x85, 0x03,            /*     REPORT_ID (3)              */
    0x09, 0x03,            /*     USAGE (LED 3)              */
    0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
    /* 66 */
    
    /* Led 4 */
    0x85, 0x04,            /*     REPORT_ID 4)		     */
    0x09, 0x04,            /*     USAGE (LED 4)	             */
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
    0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */           
    0x75, 0x08,            /*     REPORT_SIZE (8)            */        
    0x95, 0x01,            /*     REPORT_COUNT (1)           */       
    0xB1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */     

    0x85, 0x04,            /*     REPORT_ID (4)              */
    0x09, 0x04,            /*     USAGE (LED 4)              */
    0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
    /* 86 */
    
    /* key Push Button */  
    0x85, 0x05,            /*     REPORT_ID (5)              */  //2 global
    0x09, 0x05,            /*     USAGE (Push Button)        */  //3 local    
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */  //2 global    
    0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */  //2 global    
    0x75, 0x08,            /*     REPORT_SIZE (8)            */  //2 global
    0x81, 0x82,            /*     INPUT (Data,Var,Abs,Vol)   */  //1 main
    
  	0x85, 0x05,            /*     REPORT_ID (5)              */  //2 global   
    0x09, 0x05,            /*     USAGE (Push Button)        */  //3 local                   
    0xb1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */  //1 main
                          
    /* 114 */

    /* Tamper Push Button */   
    /* 142 */
	    
	  0x85, 0x06,            /*     REPORT_ID (6)              */
    0x09, 0x06,            /*     USAGE (Tamper Push Button) */      
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */      
    0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */      
    0x75, 0x08,            /*     REPORT_SIZE (8)            */  //
    0x81, 0x82,            /*     INPUT (Data,Var,Abs,Vol)   */   
    
  	0x85, 0x06,            /*     REPORT_ID (6) */
    0x09, 0x06,            /*     USAGE (Tamper Push Button) */                                 
    0xb1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */  
    
    /* ADC IN */
    0x85, 0x07,            /*     REPORT_ID (7)              */  //2 gloabl      
    0x09, 0x07,            /*     USAGE (ADC IN)             */  //3 local        
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */  //2 global             
    0x26, 0xff, 0x00,      /*     LOGICAL_MAXIMUM (255)      */  //2 global               
    0x75, 0x08,            /*     REPORT_SIZE (8)            */  //2 global         
    0x81, 0x82,            /*     INPUT (Data,Var,Abs,Vol)   */  //1 main
	
    0x85, 0x07,            /*     REPORT_ID (7)              */  //2 global
    0x09, 0x07,            /*     USAGE (ADC in)             */  //3 local                   
    0xb1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */  //1 mian                               
    /* 161 */   

    0xc0 	          /*     END_COLLECTION	             */
  };

/* USB String Descriptors */
const uint8_t CustomHID_VCP_StringLangID[CUSTOMHID_VCP_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_VCP_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04 /* LangID = 0x0409: U.S. English */
  };

const uint8_t CustomHID_VCP_StringVendor[CUSTOMHID_VCP_SIZ_STRING_VENDOR] =
  {
    CUSTOMHID_VCP_SIZ_STRING_VENDOR,     /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,             /* bDescriptorType*/
    /* Manufacturer: "STMicroelectronics" */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
  };

const uint8_t CustomHID_VCP_StringProduct[CUSTOMHID_VCP_SIZ_STRING_PRODUCT] =
  {
    CUSTOMHID_VCP_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    /* Product name: "STM32 Virtual COM Port" */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'V', 0, 'i', 0,
    'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0,
    'M', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0, 't', 0, ' ', 0, ' ', 0
  };

uint8_t CustomHID_VCP_StringSerial[CUSTOMHID_VCP_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_VCP_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
