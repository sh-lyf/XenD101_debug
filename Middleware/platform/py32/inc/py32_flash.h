/**
  ******************************************************************************
  * @file    py32_flash.h
  * @brief   flash header file
  ******************************************************************************
  */
#ifndef __FLASH_H__
#define __FLASH_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#include "py32f003_hal_conf.h"
#include "global_conf.h"

#define FMC_START_ADDR                (0x08000000U)
#define FMC_END_ADDR                  (0x08003FFFU)

#define FMC_PAGE_SIZE                 (0x80U)
#define FMC_SW_SETTING_ADDR           (0x08003DFFU)
#define FMC_HW_SETTING_ADDR           (0x08003EFFU)

int8_t Flash_ErasePage(uint32_t pageAddr);
int8_t Flash_Program(uint32_t flashStartAddr, uint32_t *data, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif

