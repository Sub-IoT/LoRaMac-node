# 
# OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
# lowpower wireless sensor communication
#
# Copyright 2017 University of Antwerp
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#Module specific parameters and options can be defined using
#MODULE_OPTION and MODULE_PARAMETER
#See cmake/module_macros.cmake for more information

MODULE_PARAM(${MODULE_PREFIX}_US_MINIMUM_DATARATE "DR_1" STRING "Minimum datarate used for US LoRAWAN")
SET_PROPERTY(CACHE ${MODULE_PREFIX}_US_MINIMUM_DATARATE PROPERTY STRINGS "DR_0;DR_1")

MODULE_PARAM(${MODULE_PREFIX}_REGION "EU868" STRING "Region for LoRaWAN")
SET_PROPERTY(CACHE ${MODULE_PREFIX}_REGION PROPERTY STRINGS "EU868;US915")


MODULE_OPTION(${MODULE_PREFIX}_MULTICAST_ON "Enables multicast mode and includes multicast keys and functions at compile time." FALSE)

MODULE_OPTION(LORAMAC_CLASSB_ENABLED "Enables Class B mode and includes required functions at compile time" FALSE)

MODULE_HEADER_DEFINE(
    ID ${MODULE_PREFIX}_US_MINIMUM_DATARATE
    ${MODULE_PREFIX}_REGION
    BOOL ${MODULE_PREFIX}_MULTICAST_ON
    LORAMAC_CLASSB_ENABLED
)

#Generate the 'module_defs.h'
MODULE_BUILD_SETTINGS_FILE()

#Export the module-specific header files to the application by using
EXPORT_GLOBAL_INCLUDE_DIRECTORIES(.)

#the US915 build contains one extra region file
IF(${${MODULE_PREFIX}_REGION} STREQUAL "US915")
    SET(US_BASE "src/mac/region/RegionBaseUS.c")
ELSE()
    SET(US_BASE "")
ENDIF()

#By convention, each module should generate a single 'static' library that can be included by the application
ADD_LIBRARY(lorawan STATIC
src/mac/LoRaMac.c
src/mac/LoRaMacAdr.c
src/mac/LoRaMacClassB.c
src/mac/LoRaMacCommands.c
src/mac/LoRaMacConfirmQueue.c
src/mac/LoRaMacCrypto.c
src/mac/LoRaMacParser.c
src/mac/LoRaMacSerializer.c
src/mac/region/Region.c
src/mac/region/RegionCommon.c
${US_BASE}
src/mac/region/Region${${MODULE_PREFIX}_REGION}.c
src/peripherals/soft-se/cmac.c   
src/peripherals/soft-se/soft-se-hal.c
src/peripherals/soft-se/soft-se.c
src/system/timeServer.c
src/boards/mcu/utilities.c
src/system/delay.c
src/system/systime.c
src/radio/sx1276/sx1276.c
ST/BSP/MLM32L07X01/mlm32l07x01.c    # TODO only for lrwan1 board
ST/hw_rtc.c
ST/hw_gpio.c
ST/hw_spi.c
ST/low_power.c
ST/stm32l0xx_hw.c   # TODO remove?
lorawan_stack.c
)



ADD_DEFINITIONS("-DUSE_B_L072Z_LRWAN1" "-DSTM32L072xx" "-DREGION_${${MODULE_PREFIX}_REGION}" "-DDEBUG" "-DTRACE") # TODO tmp

GET_PROPERTY(__global_include_dirs GLOBAL PROPERTY GLOBAL_INCLUDE_DIRECTORIES)
target_include_directories(lorawan PUBLIC
    src/mac
    src/system
    src/boards
    src/peripherals/soft-se
    src/radio
    src/radio/sx1276
    ST
    ${__global_include_dirs}
    ${CMAKE_BINARY_DIR}/framework/ #framework_defs.h
    ${CMAKE_CURRENT_BINARY_DIR} # MODULE_LORAWAN_defs.h
)

#TARGET_LINK_LIBRARIES(lorawan m)
