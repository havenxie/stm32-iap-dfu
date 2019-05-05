/**
  ******************************************************************************
  * @file    mass_mal.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Medium Access Layer interface
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
#include "platform_config.h"
#include "mass_mal.h"

/* Private typedef -----------------------------------------------------------*/
#define     FLASH_START_ADDR        0x8004000     /* Flash start address */
#define     FLASH_SIZE              0x0043000       /* 0x1C000 + 0x6000 */  
#define     FLASH_PAGE_SIZE         0x800         /* 2K per page */
#define     FLASH_SIMU_SIZE         0x6000        /* 虚拟24K */
#define     FLASH_WAIT_TIMEOUT      100000 
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Mass_Memory_Size[2];
uint32_t Mass_Block_Size[2];
uint32_t Mass_Block_Count[2];
u8 dis_mem=0;

u8 Fat12_fat1[64]={0};
u8 setup_count = 0;
u8 setup_ifread = 0;

u8 FAT12_FAT[3] = { 0xF8, 0xFF, 0xFF };

const u8 FAT12_DBR_TABLE[512]= {
	0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30, 0x00, 0x08, 0x01, 0x02, 0x00, 
	0x02, 0x00, 0x02, 0x86, 0x00, 0xF8, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x29, 0xE3, 0x97, 0x08, 0x86, 0x4E, 0x4F, 0x20, 0x4E, 0x41, 
	0x4D, 0x45, 0x20, 0x20, 0x20, 0x20, 0x46, 0x41, 0x54, 0x31, 0x32, 0x20, 0x20, 0x20, 0x33, 0xC9, 
	0x8E, 0xD1, 0xBC, 0xF0, 0x7B, 0x8E, 0xD9, 0xB8, 0x00, 0x20, 0x8E, 0xC0, 0xFC, 0xBD, 0x00, 0x7C, 
	0x38, 0x4E, 0x24, 0x7D, 0x24, 0x8B, 0xC1, 0x99, 0xE8, 0x3C, 0x01, 0x72, 0x1C, 0x83, 0xEB, 0x3A, 
	0x66, 0xA1, 0x1C, 0x7C, 0x26, 0x66, 0x3B, 0x07, 0x26, 0x8A, 0x57, 0xFC, 0x75, 0x06, 0x80, 0xCA, 
	0x02, 0x88, 0x56, 0x02, 0x80, 0xC3, 0x10, 0x73, 0xEB, 0x33, 0xC9, 0x8A, 0x46, 0x10, 0x98, 0xF7, 
	0x66, 0x16, 0x03, 0x46, 0x1C, 0x13, 0x56, 0x1E, 0x03, 0x46, 0x0E, 0x13, 0xD1, 0x8B, 0x76, 0x11, 
	0x60, 0x89, 0x46, 0xFC, 0x89, 0x56, 0xFE, 0xB8, 0x20, 0x00, 0xF7, 0xE6, 0x8B, 0x5E, 0x0B, 0x03, 
	0xC3, 0x48, 0xF7, 0xF3, 0x01, 0x46, 0xFC, 0x11, 0x4E, 0xFE, 0x61, 0xBF, 0x00, 0x00, 0xE8, 0xE6, 
	0x00, 0x72, 0x39, 0x26, 0x38, 0x2D, 0x74, 0x17, 0x60, 0xB1, 0x0B, 0xBE, 0xA1, 0x7D, 0xF3, 0xA6, 
	0x61, 0x74, 0x32, 0x4E, 0x74, 0x09, 0x83, 0xC7, 0x20, 0x3B, 0xFB, 0x72, 0xE6, 0xEB, 0xDC, 0xA0, 
	0xFB, 0x7D, 0xB4, 0x7D, 0x8B, 0xF0, 0xAC, 0x98, 0x40, 0x74, 0x0C, 0x48, 0x74, 0x13, 0xB4, 0x0E, 
	0xBB, 0x07, 0x00, 0xCD, 0x10, 0xEB, 0xEF, 0xA0, 0xFD, 0x7D, 0xEB, 0xE6, 0xA0, 0xFC, 0x7D, 0xEB, 
	0xE1, 0xCD, 0x16, 0xCD, 0x19, 0x26, 0x8B, 0x55, 0x1A, 0x52, 0xB0, 0x01, 0xBB, 0x00, 0x00, 0xE8, 
	0x3B, 0x00, 0x72, 0xE8, 0x5B, 0x8A, 0x56, 0x24, 0xBE, 0x0B, 0x7C, 0x8B, 0xFC, 0xC7, 0x46, 0xF0, 
	0x3D, 0x7D, 0xC7, 0x46, 0xF4, 0x29, 0x7D, 0x8C, 0xD9, 0x89, 0x4E, 0xF2, 0x89, 0x4E, 0xF6, 0xC6, 
	0x06, 0x96, 0x7D, 0xCB, 0xEA, 0x03, 0x00, 0x00, 0x20, 0x0F, 0xB6, 0xC8, 0x66, 0x8B, 0x46, 0xF8, 
	0x66, 0x03, 0x46, 0x1C, 0x66, 0x8B, 0xD0, 0x66, 0xC1, 0xEA, 0x10, 0xEB, 0x5E, 0x0F, 0xB6, 0xC8, 
	0x4A, 0x4A, 0x8A, 0x46, 0x0D, 0x32, 0xE4, 0xF7, 0xE2, 0x03, 0x46, 0xFC, 0x13, 0x56, 0xFE, 0xEB, 
	0x4A, 0x52, 0x50, 0x06, 0x53, 0x6A, 0x01, 0x6A, 0x10, 0x91, 0x8B, 0x46, 0x18, 0x96, 0x92, 0x33, 
	0xD2, 0xF7, 0xF6, 0x91, 0xF7, 0xF6, 0x42, 0x87, 0xCA, 0xF7, 0x76, 0x1A, 0x8A, 0xF2, 0x8A, 0xE8, 
	0xC0, 0xCC, 0x02, 0x0A, 0xCC, 0xB8, 0x01, 0x02, 0x80, 0x7E, 0x02, 0x0E, 0x75, 0x04, 0xB4, 0x42, 
	0x8B, 0xF4, 0x8A, 0x56, 0x24, 0xCD, 0x13, 0x61, 0x61, 0x72, 0x0B, 0x40, 0x75, 0x01, 0x42, 0x03, 
	0x5E, 0x0B, 0x49, 0x75, 0x06, 0xF8, 0xC3, 0x41, 0xBB, 0x00, 0x00, 0x60, 0x66, 0x6A, 0x00, 0xEB, 
	0xB0, 0x42, 0x4F, 0x4F, 0x54, 0x4D, 0x47, 0x52, 0x20, 0x20, 0x20, 0x20, 0x0D, 0x0A, 0x52, 0x65, 
	0x6D, 0x6F, 0x76, 0x65, 0x20, 0x64, 0x69, 0x73, 0x6B, 0x73, 0x20, 0x6F, 0x72, 0x20, 0x6F, 0x74, 
	0x68, 0x65, 0x72, 0x20, 0x6D, 0x65, 0x64, 0x69, 0x61, 0x2E, 0xFF, 0x0D, 0x0A, 0x44, 0x69, 0x73, 
	0x6B, 0x20, 0x65, 0x72, 0x72, 0x6F, 0x72, 0xFF, 0x0D, 0x0A, 0x50, 0x72, 0x65, 0x73, 0x73, 0x20, 
	0x61, 0x6E, 0x79, 0x20, 0x6B, 0x65, 0x79, 0x20, 0x74, 0x6F, 0x20, 0x72, 0x65, 0x73, 0x74, 0x61, 
	0x72, 0x74, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAC, 0xCB, 0xD8, 0x55, 0xAA
};

__IO uint32_t Status = 0;

#if defined(USE_STM3210E_EVAL) || defined(USE_STM32L152D_EVAL)
SD_CardInfo mSDCardInfo;
#endif

/* Private function prototypes -----------------------------------------------*/
extern void SPI_Flash_Init(void);
u16 SPI_Flash_ReadID(void);
extern void SPI_Flash_Write(u32 WriteAddr,u8* pBuffer,u16 NumByteToWrite);
extern void SPI_Flash_Read(u32 ReadAddr,u8* pBuffer,u16 NumByteToRead);

void McuFlash_Write(u32 offset,u32* buff,u16 length);
void McuFlash_Read(u32 offset,u32* buff,u16 length);
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MAL_Init
* Description    : Initializes the Media on the STM32
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Init(uint8_t lun)
{
  uint16_t status = MAL_OK;

  switch (lun)
  {
    case 0:
      Status = SD_Init();
      break;
#ifdef USE_STM3210E_EVAL
    case 1:
      NAND_Init();
      break;
#endif
    case 2:
        status = MAL_OK;
      break;
    default:
      return MAL_FAIL;
  }
  return status;
}
/*******************************************************************************
* Function Name  : MAL_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, uint32_t *Writebuff, uint16_t Transfer_Length)
{
  switch (lun)
  {
    case 0:
      FLASH_Unlock();
      McuFlash_Write(Memory_Offset, Writebuff, Transfer_Length);
      FLASH_Lock();
      break;
//    Status = SD_WriteMultiBlocks((uint8_t*)Writebuff, Memory_Offset, Transfer_Length,1);
//#if defined(USE_STM3210E_EVAL) || defined(USE_STM32L152D_EVAL)
//    Status = SD_WaitWriteOperation();  
//    while(SD_GetStatus() != SD_TRANSFER_OK);
//      if ( Status != SD_OK )
//      {
//        return MAL_FAIL;
//      }      
//#endif /* USE_STM3210E_EVAL ||USE_STM32L152D_EVAL*/      
//      break;
#ifdef USE_STM3210E_EVAL
    case 1:
      NAND_Write(Memory_Offset, Writebuff, Transfer_Length);
      break;
#endif /* USE_STM3210E_EVAL */  
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, uint32_t *Readbuff, uint16_t Transfer_Length)
{
  switch (lun)
  {
    case 0:
      McuFlash_Read(Memory_Offset, Readbuff, Transfer_Length);
      break;
//      SD_ReadMultiBlocks((uint8_t*)Readbuff, Memory_Offset, Transfer_Length, 1);
//#if defined(USE_STM3210E_EVAL) || defined(USE_STM32L152D_EVAL)
//      Status = SD_WaitReadOperation();
//      while(SD_GetStatus() != SD_TRANSFER_OK)
//      {
//      }
//      
//      if ( Status != SD_OK )
//      {
//        return MAL_FAIL;
//      }
//#endif /* USE_STM3210E_EVAL */      
//      break;
#ifdef USE_STM3210E_EVAL
    case 1:
      NAND_Read(Memory_Offset, Readbuff, Transfer_Length);
      ;
      break;
#endif
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_GetStatus
* Description    : Get status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_GetStatus (uint8_t lun)
{
#ifdef USE_STM3210E_EVAL
  NAND_IDTypeDef NAND_ID;
  uint32_t DeviceSizeMul = 0, NumberOfBlocks = 0;
#else
  SD_CSD SD_csd;
  uint32_t DeviceSizeMul = 0;
#endif /* USE_STM3210E_EVAL */

#ifdef USE_STM32L152D_EVAL

  uint32_t NumberOfBlocks = 0;
#endif

  if (lun == 0)
  {    
//#if defined (USE_STM3210E_EVAL)  || defined(USE_STM32L152D_EVAL)
//    if (SD_Init() == SD_OK)
//    {
//      SD_GetCardInfo(&mSDCardInfo);
//      SD_SelectDeselect((uint32_t) (mSDCardInfo.RCA << 16));
//      DeviceSizeMul = (mSDCardInfo.SD_csd.DeviceSizeMul + 2);

//      if(mSDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
//      {
//        Mass_Block_Count[0] = (mSDCardInfo.SD_csd.DeviceSize + 1) * 1024;
//      }
//      else
//      {
//        NumberOfBlocks  = ((1 << (mSDCardInfo.SD_csd.RdBlockLen)) / 512);
//        Mass_Block_Count[0] = ((mSDCardInfo.SD_csd.DeviceSize + 1) * (1 << DeviceSizeMul) << (NumberOfBlocks/2));
//      }
//      Mass_Block_Size[0]  = 512;

//      Status = SD_SelectDeselect((uint32_t) (mSDCardInfo.RCA << 16)); 
//      Status = SD_EnableWideBusOperation(SDIO_BusWide_4b); 
//      if ( Status != SD_OK )
//      {
//        return MAL_FAIL;
//      }
//     
//#else

//    uint32_t temp_block_mul = 0;
//    SD_GetCSDRegister(&SD_csd);
//    DeviceSizeMul = SD_csd.DeviceSizeMul + 2;
//    temp_block_mul = (1 << SD_csd.RdBlockLen)/ 512;
//    Mass_Block_Count[0] = ((SD_csd.DeviceSize + 1) * (1 << (DeviceSizeMul))) * temp_block_mul;
//    Mass_Block_Size[0] = 512;
//    Mass_Memory_Size[0] = (Mass_Block_Count[0] * Mass_Block_Size[0]);
//#endif /* USE_STM3210E_EVAL */
//      Mass_Memory_Size[0] = Mass_Block_Count[0] * Mass_Block_Size[0];
//      STM_EVAL_LEDOn(LED2);
//      return MAL_OK;
        Mass_Block_Count[0] = FLASH_SIZE/FLASH_PAGE_SIZE; 
        Mass_Block_Size[0] =  FLASH_PAGE_SIZE; 
        Mass_Memory_Size[0] = FLASH_SIZE; 

        return MAL_OK;

#if defined (USE_STM3210E_EVAL) || defined(USE_STM32L152D_EVAL)
    }
#endif /* USE_STM3210E_EVAL */
  }
#ifdef USE_STM3210E_EVAL
  else if(lun == 1)
  {
    FSMC_NAND_ReadID(&NAND_ID);
    if (NAND_ID.Device_ID != 0 )
    {
      /* only one zone is used */
      Mass_Block_Count[1] = NAND_ZONE_SIZE * NAND_BLOCK_SIZE * NAND_MAX_ZONE ;
      Mass_Block_Size[1]  = NAND_PAGE_SIZE;
      Mass_Memory_Size[1] = (Mass_Block_Count[1] * Mass_Block_Size[1]);
      return MAL_OK;
    }
  }
#endif /* USE_STM3210E_EVAL */
  STM_EVAL_LEDOn(LED2);
  return MAL_FAIL;
}



void McuFlash_Write(u32 offset,u32* buff,u16 length)
{
    u16 i;  
    u8 *pbuff= (u8 *) buff;
    
   if (offset<0x1000 ) 
    {
        return;   //const u8 FAT12_DBR_TABLE[512]
    }
    else  if (offset<0x2000)
    {
        if ( (offset==0x1000) || (offset==0x1800) )
            for ( i = 0; i<64; i++)    
            {
                Fat12_fat1[i] = pbuff[i];
            }
        return;
    }
    else if ( (offset>=0x2000)&&(offset<0x6000) )
    {
        if (offset==0x2000)    //根目录读+写,认为安装1次
        {
             if (setup_ifread)
             {           
                 if (setup_count<9) setup_count +=1;
                 else if (setup_count==9) setup_count=0;
                 else setup_count=0;
                 
                 setup_ifread=0;
             }
        }  
        return;
    }
    else   /*if (offset >= 0x6000) */
    {   
        for( i = 0; i < length; i += FLASH_PAGE_SIZE )
        { 
            if( FLASH_WaitForLastOperation(FLASH_WAIT_TIMEOUT) != FLASH_TIMEOUT )
            {
                FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
            } 	
            FLASH_ErasePage(FLASH_START_ADDR + offset + i - FLASH_SIMU_SIZE); 
        }			
         
        for( i = 0; i < length; i+=4 )
        { 
            if( FLASH_WaitForLastOperation(FLASH_WAIT_TIMEOUT) != FLASH_TIMEOUT )
            {
                FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR); 
            } 
            FLASH_ProgramWord(FLASH_START_ADDR + offset + i - FLASH_SIMU_SIZE, buff[i>>2]); 
        } 
        return;
    }
}

const u8 empty[]="程序空  TXT";
const u8 notempty[]="已安装00TXT";
void McuFlash_Read(u32 offset,u32* buff,u16 length)
{
    u16 i; 
    u8 *pbuff= (u8 *) buff;

   if (offset < 0x1000) {  
        if (offset==0)
            for (i=0;i<512;i++) pbuff[i] = FAT12_DBR_TABLE[i];
    }    
     else  if (offset < 0x2000)  //FAT1 2K, FAT2 2K
    {
        //for( i=0; i < 64; i++ ) pbuff[i]=Fat12_fat1[i];
        for( i=0; i < length; i++ ) pbuff[i]=0;
        if ( (offset==0x1000) || (offset==0x1800) ) {
            pbuff[0] =  FAT12_FAT[0];
            pbuff[1] =  FAT12_FAT[1];
            pbuff[2] =  FAT12_FAT[2];
        }
    }
    else  if (offset<0x6000)    //根目录 16K
    {        
        if (offset==0x2000) {    
            u8 appbuf[8];
           
            for( i=0; i < 8; i++ ) appbuf[i]=((vu8*)(FLASH_START_ADDR))[i];
            if ( (appbuf[0]==0xff) && (appbuf[1]==0xff) )
                for (i=0;i<11;i++) pbuff[i] = empty[i];
            else {
                for (i=0;i<11;i++) pbuff[i] = notempty[i];
                pbuff[7] = setup_count + 48;
                setup_ifread = 1;
            }
            
            for( i=11; i < 64; i++ ) pbuff[i]=0;
        }
        else  for( i=0; i < length; i++ ) pbuff[i]=0;
    }  
    else  
    {
        for( i=0; i < length; i+=4 )
        {
            buff[i>>2] = ((vu32*)(FLASH_START_ADDR + offset - FLASH_SIMU_SIZE ))[i>>2];        
        }
    }
    
    
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

