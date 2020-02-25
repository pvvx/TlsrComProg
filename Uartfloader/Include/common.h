
#pragma once

#include "app_config.h"
#include "compiler.h"
#include "types.h"
#include "bit.h"
#if CHIP_TYPE == MCU_CORE_8266
#include "register_8266.h"
#include "gpio_default_8266.h"
#else
#include "register_8267.h"
#include "gpio_default_8267.h"
#endif
#include "gpio.h"

#define USE_EXT_FLASH 0
#define CS_EXT_FLASH GPIO_PB5



