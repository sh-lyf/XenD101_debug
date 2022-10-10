/**
  ******************************************************************************
  * @file    gd32_flash.c
  * @author  ting.gao@iclegend.com
  * @brief   flash services
  ******************************************************************************
  */
#include<string.h>
#include "py32_flash.h"
#include "utilities.h"

int8_t Flash_ErasePage(uint32_t pageAddr)
{
#if 0
    if (pageAddr % FMC_PAGE_SIZE)
    {
        return FAIL;
    }
    
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(pageAddr);
    FLASH_ClearFlag(FLASH_FLAG_EOP );
    FLASH_Lock();
#endif
	if (pageAddr % FMC_PAGE_SIZE)
	{
		return FAIL;
	}

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError = 0;
	HAL_FLASH_Unlock(); 													//解锁FLASH
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGEERASE;				//擦写类型FLASH_TYPEERASE_PAGEERASE=Page擦, FLASH_TYPEERASE_SECTORERASE=Sector擦
	EraseInitStruct.PageAddress = pageAddr;								  	//擦写起始地址
	EraseInitStruct.NbPages  = 1; 											//需要擦写的页数量
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)			//执行page擦除,PAGEError返回擦写错误的page,返回0xFFFFFFFF,表示擦写成功
		printf("Flash_ErasePage err code:%d",PAGEError);
	HAL_FLASH_Lock(); 
    return OK;
}

int8_t Flash_Program(uint32_t flashStartAddr, uint32_t *data, uint16_t len)
{
#if 0
    uint16_t loop = 0;
    
    if ((flashStartAddr + (len * sizeof(uint32_t))) > FMC_END_ADDR)
    {
        return FAIL;
    }

    if (flashStartAddr % sizeof(uint32_t))
    {
        return FAIL;
    }
    
    FLASH_Unlock();

    for (loop = 0; loop < len; loop++)
    {
        FLASH_ProgramWord(flashStartAddr, data[loop]);
        flashStartAddr += sizeof(uint32_t);
        FLASH_ClearFlag(FLASH_FLAG_EOP );
    }
    
    FLASH_Lock();
#endif
	uint32_t *src = data; 
	uint32_t flash_program_start = flashStartAddr ;

	uint16_t data_len_cnt = len * sizeof(uint32_t);
	uint8_t count = len / 32;
	uint8_t index = len % 32;
	static uint32_t temBuff[32] = {0};
	memset(temBuff,0x00,32 * sizeof(uint32_t));
	if ((flashStartAddr + data_len_cnt) > FMC_END_ADDR)
    {
        return FAIL;
    }

    if (flashStartAddr % sizeof(uint32_t))
    {
        return FAIL;
    }
	HAL_FLASH_Unlock();
	
	if(count == 0)
	{
		memcpy(temBuff,src,data_len_cnt);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, flash_program_start, temBuff);
	}
	else
	{
		do{
			memcpy(temBuff,src,32 * sizeof(uint32_t));
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, flash_program_start, temBuff);
			memset(temBuff,0x00,32 * sizeof(uint32_t));
			src = src + 32;
			flash_program_start = flash_program_start + 128;
			count--;
		}while(count > 0);
		if(index > 0)
		{
			memcpy(temBuff,src,index * sizeof(uint32_t));
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, flash_program_start, temBuff);
		}
	}
	HAL_FLASH_Lock();
    return OK;
}


