/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS Robin Nano V3 supports up to 2 hotends / E steppers."
#elif HAS_FSMC_TFT
  #error "MKS Robin Nano V3 doesn't support FSMC-based TFT displays."
#endif

#define BOARD_INFO_NAME "MKS Robin Nano V3"

#define USES_DIAG_JUMPERS
/*
#ifndef X_CS_PIN
  #define X_CS_PIN                          PD5
#endif
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD7
#endif
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD4
#endif
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD9
#endif
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD8
#endif
//#ifndef E2_CS_PIN
//  #define E2_CS_PIN                         PA13
//#endif
*/
//MSU

#ifndef X_CS_PIN
  #define X_CS_PIN                          PD5
#endif
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD7
#endif
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD4
#endif
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD8
#endif
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PA13
#endif
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PD9
#endif


//
// SPI pins for TMC2130 stepper drivers
// This board only supports SW SPI for stepper drivers
//
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
#endif
#if !defined(TMC_SPI_MOSI) || TMC_SPI_MOSI == -1
  #undef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PD14
#endif
#if !defined(TMC_SPI_MISO) || TMC_SPI_MISO == -1
  #undef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PD1
#endif
#if !defined(TMC_SPI_SCK) || TMC_SPI_SCK == -1
  #undef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PD0
#endif

#include "pins_MKS_ROBIN_NANO_V3_common.h"
