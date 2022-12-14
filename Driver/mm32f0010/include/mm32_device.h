///-----------------------------------------------------------------------------
/// @file mm32_device.h
/// @brief CMSIS Cortex-M Peripheral Access Layer for MindMotion
///        microcontroller devices
///
/// This is a convenience header file for defining the part number on the
/// build command line, instead of specifying the part specific header file.
///
/// Example: Add MM32 series to your build options, to define part
///          Add "#include "mm32_device.h" to your source files
///
///
///
///
///-----------------------------------------------------------------------------

#ifndef __MM32_DEVICE_H
#define __MM32_DEVICE_H

#include "mm32_reg.h"

#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))

#endif // __MM32_DEVICE_H 

