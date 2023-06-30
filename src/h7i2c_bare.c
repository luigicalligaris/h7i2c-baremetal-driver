
/*********************************************************************************************/
/* STM32H7 I2C bare-metal driver                                                             */
/* Copyright (c) 2022-2023, Luigi Calligaris                                                 */
/* All rights reserved.                                                                      */
/*                                                                                           */
/* This software is distributed under the BSD (3-clause) license, which is reproduced below: */
/* ----------------------------------------------------------------------------------------- */
/*                                                                                           */
/* Redistribution and use in source and binary forms, with or without modification,          */
/* are permitted provided that the following conditions are met:                             */
/*                                                                                           */
/* * Redistributions of source code must retain the above copyright notice, this             */
/*   list of conditions and the following disclaimer.                                        */
/*                                                                                           */
/* * Redistributions in binary form must reproduce the above copyright notice, this          */
/*   list of conditions and the following disclaimer in the documentation and/or             */
/*   other materials provided with the distribution.                                         */
/*                                                                                           */
/* * Neither the name of SPRACE nor the one of UNESP nor the names of its                    */
/*   contributors may be used to endorse or promote products derived from                    */
/*   this software without specific prior written permission.                                */
/*                                                                                           */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND           */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED             */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                    */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR          */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES            */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;              */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON            */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                   */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS             */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                              */
/*                                                                                           */
/*********************************************************************************************/

#include "main.h"

#include <string.h>

#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h745xx.h"

#include "h7i2c_config.h"

#include "h7i2c_bare.h"
#include "h7i2c_bare_priv.h"


#if H7I2C_PERIPH_ENABLE_I2C1 == 1
uint8_t h7i2c_mutex_i2c1 = H7I2C_I2C_MUTEX_UNLOCKED;

h7i2c_driver_instance_state_t h7i2c_state_i2c1 =
{
  .fsm_state     = H7I2C_FSM_STATE_UNINITIALIZED,
  .i2c_base      = (void*) I2C1_BASE,
  .slave_address = 0x00000000,
  .cr1_value     = 0x00000000, .cr2_value = 0x00000000,
  .wr_todo       = 0x00000000, .wr_done   = 0x00000000, .wr_data = NULL,
  .rd_todo       = 0x00000000, .rd_done   = 0x00000000, .rd_data = NULL,
  .timestart     = 0x00000000, .timeout   = 0x00000000
};

h7i2c_periph_init_config_t current_periph_init_config_i2c1 = 
{
  .pin_scl  = H7I2C_PIN_I2C1_SCL_PB6,
  .pin_sda  = H7I2C_PIN_I2C1_SDA_PB7,
  .timingr  = 0x90204DFD,
  .timeoutr = 0x000084C4
};
#endif

#if H7I2C_PERIPH_ENABLE_I2C2 == 1
uint8_t h7i2c_mutex_i2c2 = H7I2C_I2C_MUTEX_UNLOCKED;

h7i2c_driver_instance_state_t h7i2c_state_i2c2 =
{
  .fsm_state     = H7I2C_FSM_STATE_UNINITIALIZED,
  .i2c_base      = (void*) I2C2_BASE,
  .slave_address = 0x00000000,
  .cr1_value     = 0x00000000, .cr2_value = 0x00000000,
  .wr_todo       = 0x00000000, .wr_done   = 0x00000000, .wr_data = NULL,
  .rd_todo       = 0x00000000, .rd_done   = 0x00000000, .rd_data = NULL,
  .timestart     = 0x00000000, .timeout   = 0x00000000
};

h7i2c_periph_init_config_t current_periph_init_config_i2c2 = 
{
  .pin_scl  = H7I2C_PIN_I2C2_SCL_PB10,
  .pin_sda  = H7I2C_PIN_I2C2_SDA_PB11,
  .timingr  = 0x90204DFD,
  .timeoutr = 0x000084C4
};
#endif

#if H7I2C_PERIPH_ENABLE_I2C3 == 1
uint8_t h7i2c_mutex_i2c3 = H7I2C_I2C_MUTEX_UNLOCKED;

h7i2c_driver_instance_state_t h7i2c_state_i2c3 =
{
  .fsm_state     = H7I2C_FSM_STATE_UNINITIALIZED,
  .i2c_base      = (void*) I2C3_BASE,
  .slave_address = 0x00000000,
  .cr1_value     = 0x00000000, .cr2_value = 0x00000000,
  .wr_todo       = 0x00000000, .wr_done   = 0x00000000, .wr_data = NULL,
  .rd_todo       = 0x00000000, .rd_done   = 0x00000000, .rd_data = NULL,
  .timestart     = 0x00000000, .timeout   = 0x00000000
};

h7i2c_periph_init_config_t current_periph_init_config_i2c3 = 
{
  .pin_scl  = H7I2C_PIN_I2C3_SCL_PA8,
  .pin_sda  = H7I2C_PIN_I2C3_SDA_PC9,
  .timingr  = 0x90204DFD,
  .timeoutr = 0x000084C4
};
#endif

#if H7I2C_PERIPH_ENABLE_I2C4 == 1
uint8_t h7i2c_mutex_i2c4 = H7I2C_I2C_MUTEX_UNLOCKED;

h7i2c_driver_instance_state_t h7i2c_state_i2c4 =
{
  .fsm_state     = H7I2C_FSM_STATE_UNINITIALIZED,
  .i2c_base      = (void*) I2C4_BASE,
  .slave_address = 0x00000000,
  .cr1_value     = 0x00000000, .cr2_value = 0x00000000,
  .wr_todo       = 0x00000000, .wr_done   = 0x00000000, .wr_data = NULL,
  .rd_todo       = 0x00000000, .rd_done   = 0x00000000, .rd_data = NULL,
  .timestart     = 0x00000000, .timeout   = 0x00000000
};

h7i2c_periph_init_config_t current_periph_init_config_i2c4 = 
{
  .pin_scl  = H7I2C_PIN_I2C4_SCL_PF14,
  .pin_sda  = H7I2C_PIN_I2C4_SDA_PF15,
  .timingr  = 0x90204DFD,
  .timeoutr = 0x000084C4
};
#endif


inline static h7i2c_driver_instance_state_t* h7i2c_get_driver_instance(h7i2c_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      return &h7i2c_state_i2c1;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      return &h7i2c_state_i2c2;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      return &h7i2c_state_i2c3;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      return &h7i2c_state_i2c4;
#endif
    default:
      return NULL;
  };
}


inline int h7i2c_is_peripheral_managed_by_this_driver(h7i2c_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      return 1;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      return 1;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      return 1;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      return 1;
#endif
    default:
      return 0;
  };
}


h7i2c_i2c_fsm_state_t h7i2c_get_state(h7i2c_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      return h7i2c_state_i2c1.fsm_state;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      return h7i2c_state_i2c2.fsm_state;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      return h7i2c_state_i2c3.fsm_state;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      return h7i2c_state_i2c4.fsm_state;
#endif
    default:
      break;
  };
  return H7I2C_FSM_STATE_UNMANAGED_BY_DRIVER;
}

int h7i2c_is_in_error(h7i2c_periph_t peripheral)
{
  switch(h7i2c_get_state(peripheral))
  {
    case H7I2C_FSM_STATE_ERROR_NACKF:
    case H7I2C_FSM_STATE_ERROR_BERR:
    case H7I2C_FSM_STATE_ERROR_ARLO:
    case H7I2C_FSM_STATE_ERROR_OVR:
    case H7I2C_FSM_STATE_ERROR_PECERR:
    case H7I2C_FSM_STATE_ERROR_TIMEOUT:
      return 1;
    default:
      break;
  }
  return 0;
}

int h7i2c_is_ready(h7i2c_periph_t peripheral)
{
  switch(h7i2c_get_state(peripheral))
  {
    case H7I2C_FSM_STATE_UNINITIALIZED:
    case H7I2C_FSM_STATE_IDLE:
      return 1;
    default:
      break;
  }
  return 0;
}

__weak h7i2c_i2c_ret_code_t h7i2c_wait_until_ready(h7i2c_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // Blocking loop, waiting for the I2C peripheral to become free, to get into error state or to timeout
  // This version loops until timeout if mutex is busy
  while (HAL_GetTick() - timestart < timeout)
  {
    if (h7i2c_is_in_error(peripheral))
      return H7I2C_RET_CODE_PERIPH_IN_ERR_STATE;

    if (h7i2c_is_ready(peripheral))
      return H7I2C_RET_CODE_OK;
  }
  return H7I2C_RET_CODE_BUSY;
}


h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_lock_impl(h7i2c_periph_t peripheral)
{
  uint8_t* p_mutex = NULL;

  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      p_mutex = &h7i2c_mutex_i2c1;
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      p_mutex = &h7i2c_mutex_i2c2;
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      p_mutex = &h7i2c_mutex_i2c3;
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      p_mutex = &h7i2c_mutex_i2c4;
      break;
#endif
    default:
      return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  // Cortex-M exclusive monitor read: sets the exclusive monitor flag on address p_mutex
  if (H7I2C_I2C_MUTEX_UNLOCKED == __LDREXB(p_mutex))
  {
    // Cortex-M exclusive monitor write: only writes and returns 0 if exclusive 
    // monitor flag is still set, otherwise return nonzero and write nothing
    if (0 == __STREXB(H7I2C_I2C_MUTEX_LOCKED, p_mutex))
    {
      __DMB();// Data Memory Barrier
      return H7I2C_RET_CODE_OK;
    }
  }
  return H7I2C_RET_CODE_BUSY;
}

__weak h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_lock(h7i2c_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // This is a simple infinite loop. As this is bare metal code it's supposed not to be an issue, 
  // but in general it is inefficient. The RTOS implementation solves this with a yield call.
  while (HAL_GetTick() - timestart < timeout)
  {
    switch ( h7i2c_i2c_mutex_lock_impl(peripheral) )
    {
      // We got the mutex
      case H7I2C_RET_CODE_OK:
        return H7I2C_RET_CODE_OK;

      // This device is not managed by the driver
      case H7I2C_RET_CODE_UNMANAGED_BY_DRIVER:
        return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

      // We havent't got the mutex (yet)
      case H7I2C_RET_CODE_BUSY:
      default:
        continue;
    }
  }

  // We timed out
  return H7I2C_RET_CODE_BUSY;
}

h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_release(h7i2c_periph_t peripheral)
{
  uint8_t* p_mutex = NULL;

  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      p_mutex = &h7i2c_mutex_i2c1;
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      p_mutex = &h7i2c_mutex_i2c2;
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      p_mutex = &h7i2c_mutex_i2c3;
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      p_mutex = &h7i2c_mutex_i2c4;
      break;
#endif
    default:
      return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  *p_mutex = H7I2C_I2C_MUTEX_UNLOCKED;

  return H7I2C_RET_CODE_OK;
}

h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_release_fromISR(h7i2c_periph_t peripheral)
{
  return h7i2c_i2c_mutex_release(peripheral);
}




h7i2c_i2c_ret_code_t h7i2c_i2c_reset_peripheral_full(h7i2c_periph_t peripheral)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  I2C_TypeDef* const i2cx = (I2C_TypeDef*) instance->i2c_base;

  // Clear the Control Register 1
  MODIFY_REG(i2cx->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE
    | I2C_CR1_NACKIE    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE
    | I2C_CR1_ANFOFF    | I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC
    | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN     | I2C_CR1_GCEN      | I2C_CR1_SMBHEN
    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN     | I2C_CR1_DNF,
    0x00000000);

  // Clear the Control Register 2
  MODIFY_REG(i2cx->CR2,
      I2C_CR2_RD_WRN   | I2C_CR2_ADD10    | I2C_CR2_HEAD10R  | I2C_CR2_RELOAD
    | I2C_CR2_AUTOEND  | I2C_CR2_SADD     | I2C_CR2_NBYTES   , 0x00000000);

  // Clear the Own Address 1
  MODIFY_REG(i2cx->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE | I2C_OAR1_OA1EN, 0x00000000);

  // Clear the Own Address 2
  MODIFY_REG(i2cx->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK  | I2C_OAR2_OA2EN, 0x00000000);

  // Clear the I2C timing register
  MODIFY_REG(i2cx->TIMINGR, I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL, 0x00000000);

  // Clear the I2C timeout register
  MODIFY_REG(i2cx->TIMEOUTR,
    I2C_TIMEOUTR_TEXTEN | I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIMEOUTB,
    0x00000000);

  // Clear the interrupts
  SET_BIT(i2cx->ICR, I2C_ICR_ADDRCF  | I2C_ICR_NACKCF  | I2C_ICR_STOPCF  | I2C_ICR_BERRCF
    | I2C_ICR_ARLOCF  | I2C_ICR_OVRCF   | I2C_ICR_PECCF   | I2C_ICR_TIMOUTCF| I2C_ICR_ALERTCF);

  return H7I2C_RET_CODE_OK;
}


h7i2c_i2c_ret_code_t h7i2c_i2c_reset_peripheral_soft(h7i2c_periph_t peripheral)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  I2C_TypeDef* i2cx = (I2C_TypeDef*) instance->i2c_base;

  CLEAR_BIT(i2cx->CR1, I2C_CR1_PE);
  ((void) READ_BIT(i2cx->CR1, I2C_CR1_PE)); // the cast to void is to suppress the "value computed is not used [-Wunused-value]" warning
  SET_BIT(i2cx->CR1, I2C_CR1_PE);

  return H7I2C_RET_CODE_OK;
}


h7i2c_i2c_ret_code_t h7i2c_i2c_reset_driver(h7i2c_periph_t peripheral)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  instance->fsm_state = H7I2C_FSM_STATE_IDLE;
  h7i2c_i2c_mutex_release(peripheral);


  return H7I2C_RET_CODE_OK;
}




h7i2c_i2c_ret_code_t h7i2c_i2c_init(h7i2c_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      h7i2c_i2c_init_by_config(peripheral, &current_periph_init_config_i2c1);
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      h7i2c_i2c_init_by_config(peripheral, &current_periph_init_config_i2c2);
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      h7i2c_i2c_init_by_config(peripheral, &current_periph_init_config_i2c3);
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      h7i2c_i2c_init_by_config(peripheral, &current_periph_init_config_i2c4);
      break;
#endif
    default:
      break;
  };
  return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;
}


h7i2c_i2c_ret_code_t h7i2c_i2c_init_by_config(h7i2c_periph_t peripheral, h7i2c_periph_init_config_t* init_config)
{
  if (!init_config)
    return H7I2C_RET_CODE_INVALID_ARGS;

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
      PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C1CLKSOURCE_D2PCLK1;
      memcpy(&current_periph_init_config_i2c1, init_config, sizeof(h7i2c_periph_init_config_t));
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
      PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C2CLKSOURCE_D2PCLK1;
      memcpy(&current_periph_init_config_i2c2, init_config, sizeof(h7i2c_periph_init_config_t));
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
      PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C3CLKSOURCE_D2PCLK1;
      memcpy(&current_periph_init_config_i2c3, init_config, sizeof(h7i2c_periph_init_config_t));
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
      PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
      memcpy(&current_periph_init_config_i2c4, init_config, sizeof(h7i2c_periph_init_config_t));
      break;
#endif
    default:
      return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      // When the M7 core pauses we want the I2C1 timeout counter to pause as well
      MODIFY_REG(DBGMCU->APB1LFZ1, DBGMCU_APB1LFZ1_DBG_I2C1, DBGMCU_APB1LFZ1_DBG_I2C1);

      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_scl)
      {
        case H7I2C_PIN_I2C1_SCL_PB6:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C1 = 4 = 0b0100 (see datasheet chapt 5) to pin PB6  (I2C1_SCL)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 24, 0b0100 << 24);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB6
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB6
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 12, 0b01 << 12);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB6
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB6
          MODIFY_REG(GPIOB->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7I2C_PIN_I2C1_SCL_PB8:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C1 = 4 = 0b0100 (see datasheet chapt 5) to pin PB8  (I2C1_SCL)
          MODIFY_REG(GPIOB->AFR[1], 0b1111      , 0b0100      );
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB8
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 16, 0b11 << 16);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB8
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 16, 0b01 << 16);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB8
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  8, 0b1 <<  8);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB8
          MODIFY_REG(GPIOB->MODER, 0b11 << 16, 0b10 << 16);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_sda)
      {
        case H7I2C_PIN_I2C1_SDA_PB7:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C1 = 4 = 0b0100 (see datasheet chapt 5) to pin PB7  (I2C1_SDA)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 28, 0b0100 << 28);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB7
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB7
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 14, 0b01 << 14);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB7
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB7
          MODIFY_REG(GPIOB->MODER, 0b11 << 14, 0b10 << 14);
          break;

        case H7I2C_PIN_I2C1_SDA_PB9:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C1 = 4 = 0b0100 (see datasheet chapt 5) to pin PB9  (I2C1_SCL)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 <<  4, 0b0100 <<  4);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB9
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 18, 0b11 << 18);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB9
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 18, 0b01 << 18);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB9
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  9, 0b1 <<  9);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB9
          MODIFY_REG(GPIOB->MODER, 0b11 << 18, 0b10 << 18);
          break;

        default:
          Error_Handler();
      }

      //
      // *** I2C SETUP ***
      //

      __HAL_RCC_I2C1_CLK_ENABLE();

      // The timing register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C1->TIMINGR, I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC,
        init_config->timingr);

      // The timeout register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C1->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TIMEOUTB | I2C_TIMEOUTR_TEXTEN,
        init_config->timeoutr);

      // Disable all own addresses. The peripheral will only be used as master.
      MODIFY_REG(I2C1->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE | I2C_OAR1_OA1EN,
          ( (0x000U << I2C_OAR1_OA1_Pos    ) & I2C_OAR1_OA1     )
        | ( (0b0    << I2C_OAR1_OA1MODE_Pos) & I2C_OAR1_OA1MODE )
        | ( (0b0    << I2C_OAR1_OA1EN_Pos  ) & I2C_OAR1_OA1EN   )
      );
      MODIFY_REG(I2C1->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK  | I2C_OAR2_OA2EN,
          ( (0x000U << I2C_OAR2_OA2_Pos   ) & I2C_OAR2_OA2    )
        | ( (0b000  << I2C_OAR2_OA2MSK_Pos) & I2C_OAR2_OA2MSK )
        | ( (0b0    << I2C_OAR2_OA2EN_Pos ) & I2C_OAR2_OA2EN  )
      );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6, 1);
      HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
      HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6, 0);
      HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

      h7i2c_state_i2c1.fsm_state = H7I2C_FSM_STATE_IDLE;

      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      // When the M7 core pauses we want the I2C2 timeout counter to pause as well
      MODIFY_REG(DBGMCU->APB1LFZ1, DBGMCU_APB1LFZ1_DBG_I2C2, DBGMCU_APB1LFZ1_DBG_I2C2);

      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_scl)
      {
        case H7I2C_PIN_I2C2_SCL_PB10:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C2 = 4 = 0b0100 (see datasheet chapt 5) to pin PB10 (I2C2_SCL)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 <<  8, 0b0100 <<  8);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB10
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 20, 0b11 << 20);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB10
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 20, 0b01 << 20);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB10
          MODIFY_REG(GPIOB->OTYPER, 0b1 << 10, 0b1 << 10);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB10
          MODIFY_REG(GPIOB->MODER, 0b11 << 20, 0b10 << 20);
          break;

        case H7I2C_PIN_I2C2_SCL_PF1:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function I2C2 = 4 = 0b0100 (see datasheet chapt 5) to pin PF1  (I2C2_SCL)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 <<  4, 0b0100 <<  4);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF1
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 <<  2, 0b11 <<  2);
          // GPIOF PUPDR: Set pull-up = 0b01 to pin PF1
          MODIFY_REG(GPIOF->PUPDR, 0b11 <<  2, 0b01 <<  2);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF1
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  1, 0b1 <<  1);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF1
          MODIFY_REG(GPIOF->MODER, 0b11 <<  2, 0b10 <<  2);
          break;

        case H7I2C_PIN_I2C2_SCL_PH4:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function I2C2 = 4 = 0b0100 (see datasheet chapt 5) to pin PH4  (I2C2_SCL)
          MODIFY_REG(GPIOH->AFR[0], 0b1111 << 16, 0b0100 << 16);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH4
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOH PUPDR: Set pull-up = 0b01 to pin PH4
          MODIFY_REG(GPIOH->PUPDR, 0b11 <<  8, 0b01 <<  8);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH4
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  4, 0b1 <<  4);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH4
          MODIFY_REG(GPIOH->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_sda)
      {
        case H7I2C_PIN_I2C2_SDA_PB11:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C2 = 4 = 0b0100 (see datasheet chapt 5) to pin PB11 (I2C2_SDA)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 << 12, 0b0100 << 12);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB11
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 22, 0b11 << 22);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB11
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 22, 0b01 << 22);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB11
          MODIFY_REG(GPIOB->OTYPER, 0b1 << 11, 0b1 << 11);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB11
          MODIFY_REG(GPIOB->MODER, 0b11 << 22, 0b10 << 22);
          break;

        case H7I2C_PIN_I2C2_SDA_PF0:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function I2C2 = 4 = 0b0100 (see datasheet chapt 5) to pin PF0  (I2C2_SCL)
          MODIFY_REG(GPIOF->AFR[0], 0b1111      , 0b0100      );
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF0
          MODIFY_REG(GPIOF->OSPEEDR, 0b11      , 0b11      );
          // GPIOF PUPDR: Set pull-up = 0b01 to pin PF0
          MODIFY_REG(GPIOF->PUPDR, 0b11      , 0b01      );
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF0
          MODIFY_REG(GPIOF->OTYPER, 0b1      , 0b1      );
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF0
          MODIFY_REG(GPIOF->MODER, 0b11      , 0b10      );
          break;

        case H7I2C_PIN_I2C2_SDA_PH5:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function I2C2 = 4 = 0b0100 (see datasheet chapt 5) to pin PH5  (I2C2_SCL)
          MODIFY_REG(GPIOH->AFR[0], 0b1111 << 20, 0b0100 << 20);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PB8
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOH PUPDR: Set pull-up = 0b01 to pin PB8
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 10, 0b01 << 10);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PB8
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  5, 0b1 <<  5);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PB8
          MODIFY_REG(GPIOH->MODER, 0b11 << 10, 0b10 << 10);
          break;

        default:
          Error_Handler();
      }

      //
      // *** I2C SETUP ***
      //

      __HAL_RCC_I2C2_CLK_ENABLE();

      // The timing register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C2->TIMINGR, I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC,
        init_config->timingr);

      // The timeout register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C2->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TIMEOUTB | I2C_TIMEOUTR_TEXTEN,
        init_config->timeoutr);

      // Disable all own addresses. The peripheral will only be used as master.
      MODIFY_REG(I2C2->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE | I2C_OAR1_OA1EN,
          ( (0x000U << I2C_OAR1_OA1_Pos    ) & I2C_OAR1_OA1     )
        | ( (0b0    << I2C_OAR1_OA1MODE_Pos) & I2C_OAR1_OA1MODE )
        | ( (0b0    << I2C_OAR1_OA1EN_Pos  ) & I2C_OAR1_OA1EN   )
      );
      MODIFY_REG(I2C2->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK  | I2C_OAR2_OA2EN,
          ( (0x000U << I2C_OAR2_OA2_Pos   ) & I2C_OAR2_OA2    )
        | ( (0b000  << I2C_OAR2_OA2MSK_Pos) & I2C_OAR2_OA2MSK )
        | ( (0b0    << I2C_OAR2_OA2EN_Pos ) & I2C_OAR2_OA2EN  )
      );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(I2C2_EV_IRQn, 6, 3);
      HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
      HAL_NVIC_SetPriority(I2C2_ER_IRQn, 6, 2);
      HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

      h7i2c_state_i2c2.fsm_state = H7I2C_FSM_STATE_IDLE;

      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      // When the M7 core pauses we want the I2C3 timeout counter to pause as well
      MODIFY_REG(DBGMCU->APB1LFZ1, DBGMCU_APB1LFZ1_DBG_I2C3, DBGMCU_APB1LFZ1_DBG_I2C3);

      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_scl)
      {
        case H7I2C_PIN_I2C3_SCL_PA8:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function I2C3 = 4 = 0b0100 (see datasheet chapt 5) to pin PA8  (I2C3_SCL)
          MODIFY_REG(GPIOA->AFR[1], 0b1111      , 0b0100      );
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA8
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 16, 0b11 << 16);
          // GPIOA PUPDR: Set pull-up = 0b01 to pin PA8
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 16, 0b01 << 16);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA8
          MODIFY_REG(GPIOA->OTYPER, 0b1 <<  8, 0b1 <<  8);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA8
          MODIFY_REG(GPIOA->MODER, 0b11 << 16, 0b10 << 16);
          break;

        case H7I2C_PIN_I2C3_SCL_PH7:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function I2C3 = 4 = 0b0100 (see datasheet chapt 5) to pin PH7  (I2C3_SCL)
          MODIFY_REG(GPIOH->AFR[1], 0b1111 << 28, 0b0100 << 28);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH7
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOH PUPDR: Set pull-up = 0b01 to pin PH7
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 14, 0b01 << 14);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH7
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH7
          MODIFY_REG(GPIOH->MODER, 0b11 << 14, 0b10 << 14);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_sda)
      {
        case H7I2C_PIN_I2C3_SDA_PC9:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function I2C3 = 4 = 0b0100 (see datasheet chapt 5) to pin PC9  (I2C3_SDA)
          MODIFY_REG(GPIOC->AFR[1], 0b1111 << 4, 0b0100 << 4);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC9
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 18, 0b11 << 18);
          // GPIOC PUPDR: Set pull-up = 0b01 to pin PC9
          MODIFY_REG(GPIOC->PUPDR, 0b11 << 18, 0b01 << 18);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PC9
          MODIFY_REG(GPIOC->OTYPER, 0b1 <<  9, 0b1 <<  9);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PC9
          MODIFY_REG(GPIOC->MODER, 0b11 << 18, 0b10 << 18);
          break;

        case H7I2C_PIN_I2C3_SDA_PH8:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function I2C3 = 4 = 0b0100 (see datasheet chapt 5) to pin PH8  (I2C3_SCL)
          MODIFY_REG(GPIOH->AFR[1], 0b1111      , 0b0100      );
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH8
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 16, 0b11 << 16);
          // GPIOH PUPDR: Set pull-up = 0b01 to pin PH8
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 16, 0b01 << 16);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH8
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  8, 0b1 <<  8);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH8
          MODIFY_REG(GPIOH->MODER, 0b11 << 16, 0b10 << 16);
          break;

        default:
          Error_Handler();
      }

      //
      // *** I2C SETUP ***
      //

      __HAL_RCC_I2C3_CLK_ENABLE();

      // The timing register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C3->TIMINGR, I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC,
        init_config->timingr);

      // The timeout register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C3->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TIMEOUTB | I2C_TIMEOUTR_TEXTEN,
        init_config->timeoutr);

      // Disable all own addresses. The peripheral will only be used as master.
      MODIFY_REG(I2C3->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE | I2C_OAR1_OA1EN,
          ( (0x000U << I2C_OAR1_OA1_Pos    ) & I2C_OAR1_OA1     )
        | ( (0b0    << I2C_OAR1_OA1MODE_Pos) & I2C_OAR1_OA1MODE )
        | ( (0b0    << I2C_OAR1_OA1EN_Pos  ) & I2C_OAR1_OA1EN   )
      );
      MODIFY_REG(I2C3->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK  | I2C_OAR2_OA2EN,
          ( (0x000U << I2C_OAR2_OA2_Pos   ) & I2C_OAR2_OA2    )
        | ( (0b000  << I2C_OAR2_OA2MSK_Pos) & I2C_OAR2_OA2MSK )
        | ( (0b0    << I2C_OAR2_OA2EN_Pos ) & I2C_OAR2_OA2EN  )
      );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(I2C3_EV_IRQn, 6, 5);
      HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
      HAL_NVIC_SetPriority(I2C3_ER_IRQn, 6, 4);
      HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

      h7i2c_state_i2c3.fsm_state = H7I2C_FSM_STATE_IDLE;

      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      // When the M7 core pauses we want the I2C4 timeout counter to pause as well
      MODIFY_REG(DBGMCU->APB4FZ1, DBGMCU_APB4FZ1_DBG_I2C4, DBGMCU_APB4FZ1_DBG_I2C4);

      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_scl)
      {
        case H7I2C_PIN_I2C4_SCL_PB6:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C4 = 6 = 0b0110 (see datasheet chapt 5) to pin PB6  (I2C4_SCL)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 24, 0b0110 << 24);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB6
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB6
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 12, 0b01 << 12);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB6
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB6
          MODIFY_REG(GPIOB->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7I2C_PIN_I2C4_SCL_PB8:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C4 = 6 = 0b0110 (see datasheet chapt 5) to pin PB8  (I2C4_SCL)
          MODIFY_REG(GPIOB->AFR[1], 0b1111      , 0b0110      );
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB8
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 16, 0b11 << 16);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB8
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 16, 0b01 << 16);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB8
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  8, 0b1 <<  8);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB8
          MODIFY_REG(GPIOB->MODER, 0b11 << 16, 0b10 << 16);
          break;

        case H7I2C_PIN_I2C4_SCL_PD12:
          __HAL_RCC_GPIOD_CLK_ENABLE();

          // GPIOD AFRL: Set alternate function I2C4 = 4 = 0b0100 (see datasheet chapt 5) to pin PD12 (I2C4_SCL)
          MODIFY_REG(GPIOD->AFR[1], 0b1111 << 16, 0b0100 << 16);
          // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PH7
          MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOD PUPDR: Set pull-up = 0b01 to pin PH7
          MODIFY_REG(GPIOD->PUPDR, 0b11 << 24, 0b01 << 24);
          // GPIOD OTYPEDR: Set open drain = 0b1 to pin PH7
          MODIFY_REG(GPIOD->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOD MODER: Set alternate mode = 0b10 to pins PH7
          MODIFY_REG(GPIOD->MODER, 0b11 << 24, 0b10 << 24);
          break;

        case H7I2C_PIN_I2C4_SCL_PF14:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function I2C4 = 4 = 0b0100 (see datasheet chapt 5) to pin PF14 (I2C4_SCL)
          MODIFY_REG(GPIOF->AFR[1], 0b1111 << 24, 0b0100 << 24);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF14
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 28, 0b11 << 28);
          // GPIOF PUPDR: Set pull-up = 0b01 to pin PF14
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 28, 0b01 << 28);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF14
          MODIFY_REG(GPIOF->OTYPER, 0b1 << 14, 0b1 << 14);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF14
          MODIFY_REG(GPIOF->MODER, 0b11 << 28, 0b10 << 28);
          break;

        case H7I2C_PIN_I2C4_SCL_PH11:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function I2C4 = 4 = 0b0100 (see datasheet chapt 5) to pin PH11  (I2C4_SCL)
          MODIFY_REG(GPIOH->AFR[1], 0b1111 << 12, 0b0100 << 12);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH11
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 22, 0b11 << 22);
          // GPIOH PUPDR: Set pull-up = 0b01 to pin PH11
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 22, 0b01 << 22);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH11
          MODIFY_REG(GPIOH->OTYPER, 0b1 << 11, 0b1 << 11);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH11
          MODIFY_REG(GPIOH->MODER, 0b11 << 22, 0b10 << 22);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_sda)
      {
        case H7I2C_PIN_I2C4_SDA_PB7:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C4 = 6 = 0b0110 (see datasheet chapt 5) to pin PB7  (I2C4_SDA)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 28, 0b0110 << 28);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB7
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB7
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 14, 0b01 << 14);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB7
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB7
          MODIFY_REG(GPIOB->MODER, 0b11 << 14, 0b10 << 14);
          break;

        case H7I2C_PIN_I2C4_SDA_PB9:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function I2C4 = 6 = 0b0110 (see datasheet chapt 5) to pin PB9  (I2C4_SCL)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 <<  4, 0b0110 <<  4);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB9
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 18, 0b11 << 18);
          // GPIOB PUPDR: Set pull-up = 0b01 to pin PB9
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 18, 0b01 << 18);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB9
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  9, 0b1 <<  9);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB9
          MODIFY_REG(GPIOB->MODER, 0b11 << 18, 0b10 << 18);
          break;

        case H7I2C_PIN_I2C4_SDA_PD13:
          __HAL_RCC_GPIOD_CLK_ENABLE();

          // GPIOD AFRL: Set alternate function I2C4 = 4 = 0b0100 (see datasheet chapt 5) to pin PD13 (I2C4_SCL)
          MODIFY_REG(GPIOD->AFR[1], 0b1111 << 20, 0b0100 << 20);
          // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD13
          MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 26, 0b11 << 26);
          // GPIOD PUPDR: Set pull-up = 0b01 to pin PD13
          MODIFY_REG(GPIOD->PUPDR, 0b11 << 26, 0b01 << 26);
          // GPIOD OTYPEDR: Set open drain = 0b1 to pin PD13
          MODIFY_REG(GPIOD->OTYPER, 0b1 << 13, 0b1 << 13);
          // GPIOD MODER: Set alternate mode = 0b10 to pins PD13
          MODIFY_REG(GPIOD->MODER, 0b11 << 26, 0b10 << 26);
          break;

        case H7I2C_PIN_I2C4_SDA_PF15:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function I2C4 = 4 = 0b0100 (see datasheet chapt 5) to pin PF15 (I2C4_SCL)
          MODIFY_REG(GPIOF->AFR[1], 0b1111 << 28, 0b0100 << 28);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF15
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 30, 0b11 << 30);
          // GPIOF PUPDR: Set pull-up = 0b01 to pin PF15
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 30, 0b01 << 30);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF15
          MODIFY_REG(GPIOF->OTYPER, 0b1 << 15, 0b1 << 15);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF15
          MODIFY_REG(GPIOF->MODER, 0b11 << 30, 0b10 << 30);
          break;

        case H7I2C_PIN_I2C4_SDA_PH12:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function I2C4 = 4 = 0b0100 (see datasheet chapt 5) to pin PH12 (I2C4_SCL)
          MODIFY_REG(GPIOH->AFR[1], 0b1111 << 16, 0b0100 << 16);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH12
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOH PUPDR: Set pull-up = 0b01 to pin PH12
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 24, 0b01 << 24);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH12
          MODIFY_REG(GPIOH->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH12
          MODIFY_REG(GPIOH->MODER, 0b11 << 24, 0b10 << 24);
          break;

        default:
          Error_Handler();
      }

      //
      // *** I2C SETUP ***
      //

      __HAL_RCC_I2C4_CLK_ENABLE();

      // The timing register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C4->TIMINGR, I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC,
        init_config->timingr);

      // The timeout register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
      // given the chosen frequency and delay settings.
      MODIFY_REG(I2C4->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TIMEOUTB | I2C_TIMEOUTR_TEXTEN,
        init_config->timeoutr);

      // Disable all own addresses. The peripheral will only be used as master.
      MODIFY_REG(I2C4->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE | I2C_OAR1_OA1EN,
          ( (0x000U << I2C_OAR1_OA1_Pos    ) & I2C_OAR1_OA1     )
        | ( (0b0    << I2C_OAR1_OA1MODE_Pos) & I2C_OAR1_OA1MODE )
        | ( (0b0    << I2C_OAR1_OA1EN_Pos  ) & I2C_OAR1_OA1EN   )
      );
      MODIFY_REG(I2C4->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK  | I2C_OAR2_OA2EN,
          ( (0x000U << I2C_OAR2_OA2_Pos   ) & I2C_OAR2_OA2    )
        | ( (0b000  << I2C_OAR2_OA2MSK_Pos) & I2C_OAR2_OA2MSK )
        | ( (0b0    << I2C_OAR2_OA2EN_Pos ) & I2C_OAR2_OA2EN  )
      );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(I2C4_EV_IRQn, 6, 7);
      HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);
      HAL_NVIC_SetPriority(I2C4_ER_IRQn, 6, 6);
      HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);

      h7i2c_state_i2c4.fsm_state = H7I2C_FSM_STATE_IDLE;

      break;
#endif
    default:
      return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  return H7I2C_RET_CODE_OK;
}


void h7i2c_deinit(h7i2c_periph_t peripheral)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return;

  h7i2c_i2c_reset_peripheral_full(peripheral);

  switch(peripheral)
  {
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
    case H7I2C_I2C1:
      // Disable the I2C1 interrupts in NVIC
      HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
      HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

      // Disable the peripheral
      CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);

      // Disable the peripheral clock
      __HAL_RCC_I2C1_CLK_DISABLE();

      // Deinit the GPIOs used by I2C1
      switch(current_periph_init_config_i2c1.pin_scl)
      {
        case H7I2C_PIN_I2C1_SCL_PB6:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
          break;

        case H7I2C_PIN_I2C1_SCL_PB8:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
          break;

        default:
          break;
      }
      switch(current_periph_init_config_i2c1.pin_sda)
      {
        case H7I2C_PIN_I2C1_SDA_PB7:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
          break;

        case H7I2C_PIN_I2C1_SDA_PB9:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
          break;

        default:
          break;
      }
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
    case H7I2C_I2C2:
      // Disable the I2C2 interrupts in NVIC
      HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
      HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);

      // Disable the peripheral
      CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);

      // Disable the peripheral clock
      __HAL_RCC_I2C2_CLK_DISABLE();

      // Deinit the GPIOs used by I2C2
      switch(current_periph_init_config_i2c2.pin_scl)
      {
        case H7I2C_PIN_I2C2_SCL_PB10:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
          break;

        case H7I2C_PIN_I2C2_SCL_PF1:
          HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1);
          break;

        case H7I2C_PIN_I2C2_SCL_PH4:
          HAL_GPIO_DeInit(GPIOH, GPIO_PIN_4);
          break;

        default:
          break;
      }
      switch(current_periph_init_config_i2c2.pin_sda)
      {
        case H7I2C_PIN_I2C2_SDA_PB11:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
          break;

        case H7I2C_PIN_I2C2_SDA_PF0:
          HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0);
          break;

        case H7I2C_PIN_I2C2_SDA_PH5:
          HAL_GPIO_DeInit(GPIOH, GPIO_PIN_5);
          break;

        default:
          break;
      }
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
    case H7I2C_I2C3:
      // Disable the I2C3 interrupts in NVIC
      HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
      HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);

      // Disable the peripheral
      CLEAR_BIT(I2C3->CR1, I2C_CR1_PE);

      // Disable the peripheral clock
      __HAL_RCC_I2C3_CLK_DISABLE();

      // Deinit the GPIOs used by I2C3
      switch(current_periph_init_config_i2c3.pin_scl)
      {
        case H7I2C_PIN_I2C3_SCL_PA8:
          HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
          break;

        case H7I2C_PIN_I2C3_SCL_PH7:
          HAL_GPIO_DeInit(GPIOH, GPIO_PIN_7);
          break;

        default:
          break;
      }
      switch(current_periph_init_config_i2c3.pin_sda)
      {
        case H7I2C_PIN_I2C3_SDA_PC9:
          HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
          break;

        case H7I2C_PIN_I2C3_SDA_PH8:
          HAL_GPIO_DeInit(GPIOH, GPIO_PIN_8);
          break;

        default:
          break;
      }
      break;
#endif
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
    case H7I2C_I2C4:
      // Disable the I2C4 interrupts in NVIC
      HAL_NVIC_DisableIRQ(I2C4_EV_IRQn);
      HAL_NVIC_DisableIRQ(I2C4_ER_IRQn);

      // Disable the peripheral
      CLEAR_BIT(I2C4->CR1, I2C_CR1_PE);

      // Disable the peripheral clock
      __HAL_RCC_I2C4_CLK_DISABLE();

      // Deinit the GPIOs used by I2C4
      switch(current_periph_init_config_i2c4.pin_scl)
      {
        case H7I2C_PIN_I2C4_SCL_PB6:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
          break;

        case H7I2C_PIN_I2C4_SCL_PB8:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
          break;

        case H7I2C_PIN_I2C4_SCL_PD12:
          HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);
          break;

        case H7I2C_PIN_I2C4_SCL_PF14:
          HAL_GPIO_DeInit(GPIOF, GPIO_PIN_14);
          break;

        case H7I2C_PIN_I2C4_SCL_PH11:
          HAL_GPIO_DeInit(GPIOH, GPIO_PIN_11);
          break;

        default:
          break;
      }
      switch(current_periph_init_config_i2c4.pin_sda)
      {
        case H7I2C_PIN_I2C4_SDA_PB7:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
          break;

        case H7I2C_PIN_I2C4_SDA_PB9:
          HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
          break;

        case H7I2C_PIN_I2C4_SDA_PD13:
          HAL_GPIO_DeInit(GPIOD, GPIO_PIN_13);
          break;

        case H7I2C_PIN_I2C4_SDA_PF15:
          HAL_GPIO_DeInit(GPIOF, GPIO_PIN_15);
          break;

        case H7I2C_PIN_I2C4_SDA_PH12:
          HAL_GPIO_DeInit(GPIOH, GPIO_PIN_12);
          break;

        default:
          break;
      }
      break;
#endif
    default:
      return;
  };
}


h7i2c_i2c_ret_code_t h7i2c_clear_error_state(h7i2c_periph_t peripheral)
{
  uint32_t const timeout = 100;

  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  if (h7i2c_i2c_mutex_lock(peripheral, timeout) == H7I2C_RET_CODE_OK)
  {
    switch (instance->fsm_state)
      {
        case H7I2C_FSM_STATE_ERROR_NACKF:
        case H7I2C_FSM_STATE_ERROR_BERR:
        case H7I2C_FSM_STATE_ERROR_ARLO:
        case H7I2C_FSM_STATE_ERROR_OVR:
        case H7I2C_FSM_STATE_ERROR_PECERR:
        case H7I2C_FSM_STATE_ERROR_TIMEOUT:
          instance->fsm_state = H7I2C_FSM_STATE_IDLE;
          h7i2c_i2c_mutex_release(peripheral);
          return H7I2C_RET_CODE_OK;
        default:
          h7i2c_i2c_mutex_release(peripheral);
          return H7I2C_RET_CODE_CLEARED_A_NONERROR_STATE;
      }
  }
  return H7I2C_RET_CODE_BUSY;
}


void H7I2C_EV_IRQHandler_Impl(h7i2c_periph_t peripheral)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);
  I2C_TypeDef* hardware = (I2C_TypeDef *) instance->i2c_base;

  uint32_t const isr = hardware->ISR;
  instance->cr1_value = hardware->CR1;
  instance->cr2_value = hardware->CR2;

  uint32_t const fsm_state_copy = instance->fsm_state;

  // Reminder: RXNE is cleared by reading the I2C_RXDR register
  if (READ_BIT(isr, I2C_ISR_RXNE) != 0)
  {
    instance->rd_data[instance->rd_done] = hardware->RXDR;
    --(instance->rd_todo);
    ++(instance->rd_done);
    if (instance->rd_todo == 0U)
      CLEAR_BIT(instance->cr1_value, I2C_CR1_RXIE);
  }

  // Reminder: TXIS/TXE is cleared by writing the I2C_TXDR register
  if (READ_BIT(isr, I2C_ISR_TXIS) != 0)
  {
    if (READ_BIT(isr, I2C_ISR_TXE) != 0)
    {
      hardware->TXDR = instance->wr_data[instance->wr_done];
      --(instance->wr_todo);
      ++(instance->wr_done);
      if (instance->wr_todo == 0U)
        CLEAR_BIT(instance->cr1_value, I2C_CR1_TXIE);
    }
  }

  // Reminder: TC is cleared by writing START = 1 or STOP = 1
  if (READ_BIT(isr, I2C_ISR_TC) != 0)
  {
    switch(fsm_state_copy)
    {
      case H7I2C_FSM_STATE_WRITEREAD_WRITESTEP:
        CLEAR_BIT(instance->cr1_value, I2C_CR1_TXIE  );
        SET_BIT  (instance->cr1_value, I2C_CR1_RXIE  );
        SET_BIT  (instance->cr2_value, I2C_CR2_RD_WRN | I2C_CR2_START);
        SET_BIT  (instance->cr2_value, I2C_CR2_AUTOEND);
        instance->fsm_state = H7I2C_FSM_STATE_WRITEREAD_READSTEP;
        if (instance->rd_todo > 255UL)
        {
          MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, (255UL << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
          SET_BIT   (instance->cr2_value, I2C_CR2_RELOAD);
        }
        else
        {
          MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, (instance->rd_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
          CLEAR_BIT (instance->cr2_value, I2C_CR2_RELOAD);
        }
        break;

      case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP:
        CLEAR_BIT(instance->cr1_value, I2C_CR1_TXIE  );
        SET_BIT  (instance->cr1_value, I2C_CR1_RXIE  );
        SET_BIT  (instance->cr2_value, I2C_CR2_RD_WRN | I2C_CR2_START);
        // We just read one byte, which encodes the number of RX bytes which will follow
        MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, (1U << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
        CLEAR_BIT (instance->cr2_value, I2C_CR2_RELOAD);
        instance->fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXNBYTES;
        break;

      case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXNBYTES:
        SET_BIT  (instance->cr2_value, I2C_CR2_START);
        instance->rd_todo = instance->rd_data[0];
        instance->rd_done = 1U;
        MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, (instance->rd_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
        CLEAR_BIT (instance->cr2_value, I2C_CR2_RELOAD);
        instance->fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXDATA;
        break;

      case H7I2C_FSM_STATE_WRITEONLY:
        CLEAR_BIT(instance->cr1_value, I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE);
        SET_BIT  (instance->cr2_value, I2C_CR2_STOP);
        break;

      case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXDATA:
      case H7I2C_FSM_STATE_WRITEREAD_READSTEP:
      case H7I2C_FSM_STATE_READONLY:
        CLEAR_BIT(instance->cr1_value, I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE);
        SET_BIT  (instance->cr2_value, I2C_CR2_STOP);
        break;

      default:
        break;
    }
  }

  // Reminder: TCR cleared by writing I2C_CR2 with NBYTES[7:0] != 0
  if ( READ_BIT(isr, I2C_ISR_TCR) != 0 )
  {
    switch(fsm_state_copy)
    {
      case H7I2C_FSM_STATE_WRITEREAD_WRITESTEP:
      case H7I2C_FSM_STATE_WRITEONLY:
      case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP:
        if (instance->wr_todo > 255UL)
        {
          MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
            ( (255UL << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | I2C_CR2_RELOAD);
        }
        else
        {
          MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
            ( (instance->wr_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | 0U);
        }
        break;

      case H7I2C_FSM_STATE_WRITEREAD_READSTEP:
      case H7I2C_FSM_STATE_READONLY:
        if (instance->rd_todo > 255UL)
        {
          MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
            ( (255UL << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | I2C_CR2_RELOAD);
        }
        else
        {
          MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
            ( (instance->rd_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | 0U);
        }
        break;

      default:
        break;
    }
  }

  uint32_t icr = 0u;

  // Reminder: STOPF cleared by writing STOPCF = 1
  if (READ_BIT(isr, I2C_ISR_STOPF) != 0)
  {
    SET_BIT(icr, I2C_ICR_STOPCF);
    CLEAR_BIT(instance->cr1_value, I2C_CR1_PE);
  }

  // Reminder: ADDR is cleared by writing ADDRCF = 1
  if (READ_BIT(isr, I2C_ISR_ADDR) != 0)
  {
    // In principle we should not get here, as this is disabled
    SET_BIT(icr, I2C_ICR_ADDRCF);
  }

  // Reminder: NACKF is cleared by writing NACKCF = 1
  if (READ_BIT(isr, I2C_ISR_NACKF) != 0)
  {
    SET_BIT(icr, I2C_ICR_NACKCF);
    instance->fsm_state = H7I2C_FSM_STATE_ERROR_NACKF;
  }


  // Commit to the hardware (CR1, CR2, ICR registers) the state changes
  MODIFY_REG(hardware->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE    | I2C_CR1_NACKIE
    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE     | I2C_CR1_DNF       | I2C_CR1_ANFOFF
    | I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC       | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN
    | I2C_CR1_GCEN      | I2C_CR1_SMBHEN    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN,
    instance->cr1_value
  );

  MODIFY_REG(hardware->CR2,
      I2C_CR2_SADD      | I2C_CR2_RD_WRN    | I2C_CR2_ADD10      | I2C_CR2_HEAD10R      | I2C_CR2_START
    | I2C_CR2_STOP      | I2C_CR2_NACK      | I2C_CR2_NBYTES     | I2C_CR2_RELOAD       | I2C_CR2_AUTOEND
    | I2C_CR2_PECBYTE,
    instance->cr2_value
  );

  hardware->ICR = icr;

  ((void) READ_REG(hardware->CR1));
  ((void) READ_REG(hardware->CR2));

  if (READ_BIT(isr, I2C_ISR_STOPF) != 0)
  {
    instance->fsm_state = H7I2C_FSM_STATE_IDLE;
    h7i2c_i2c_mutex_release_fromISR(peripheral);
  }
}


void H7I2C_ER_IRQHandler_Impl(h7i2c_periph_t peripheral)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);
  I2C_TypeDef* hardware = (I2C_TypeDef *) instance->i2c_base;

  uint32_t const isr = hardware->ISR;
  instance->cr1_value = hardware->CR1;
  instance->cr2_value = hardware->CR2;

  uint32_t icr       = 0u;

  // BERR is cleared by writing BERRCF = 1
  if ( READ_BIT(isr, I2C_ISR_BERR) != 0 )
  {
    icr |= I2C_ICR_BERRCF;
    instance->fsm_state = H7I2C_FSM_STATE_ERROR_BERR;
    SET_BIT(instance->cr2_value, I2C_CR2_STOP);
  }

  // ARLO is cleared by writing ARLOCF = 1
  if ( READ_BIT(isr, I2C_ISR_ARLO) != 0 )
  {
    icr |= I2C_ICR_ARLOCF;
    instance->fsm_state = H7I2C_FSM_STATE_ERROR_ARLO;
    SET_BIT(instance->cr2_value, I2C_CR2_STOP);
  }

  // OVR is cleared by writing OVRCF = 1
  if ( READ_BIT(isr, I2C_ISR_OVR) != 0 )
  {
    icr |= I2C_ICR_OVRCF;
    instance->fsm_state = H7I2C_FSM_STATE_ERROR_OVR;
    SET_BIT(instance->cr2_value, I2C_CR2_STOP);
  }

  // PECERR is cleared by writing PECCF = 1
  if ( READ_BIT(isr, I2C_ISR_PECERR) != 0 )
  {
    icr |= I2C_ICR_PECCF;
    instance->fsm_state = H7I2C_FSM_STATE_ERROR_PECERR;
    SET_BIT(instance->cr2_value, I2C_CR2_STOP);
  }

  // TIMEOUT is cleared by writing TIMEOUTCF = 1
  if ( READ_BIT(isr, I2C_ISR_TIMEOUT) != 0 )
  {
    icr |= I2C_ICR_TIMOUTCF;
    instance->fsm_state = H7I2C_FSM_STATE_ERROR_TIMEOUT;
    SET_BIT(instance->cr2_value, I2C_CR2_STOP);
  }

  // ALERT is cleared by writing ALERTCF = 1
  if ( READ_BIT(isr, I2C_ISR_ALERT) != 0 )
  {
    icr |= I2C_ICR_ALERTCF;
    SET_BIT(instance->cr2_value, I2C_CR2_STOP);
  }

  // Commit to the hardware (CR1, CR2, ICR registers) the state changes
  MODIFY_REG(hardware->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE    | I2C_CR1_NACKIE
    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE     | I2C_CR1_DNF       | I2C_CR1_ANFOFF
    | I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC       | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN
    | I2C_CR1_GCEN      | I2C_CR1_SMBHEN    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN,
    instance->cr1_value
  );

  MODIFY_REG(hardware->CR2,
      I2C_CR2_SADD      | I2C_CR2_RD_WRN    | I2C_CR2_ADD10      | I2C_CR2_HEAD10R      | I2C_CR2_START
    | I2C_CR2_STOP      | I2C_CR2_NACK      | I2C_CR2_NBYTES     | I2C_CR2_RELOAD       | I2C_CR2_AUTOEND
    | I2C_CR2_PECBYTE,
    instance->cr2_value
  );

  hardware->ICR = icr;

  ((void) READ_REG(hardware->CR1));
  ((void) READ_REG(hardware->CR2));

  if (READ_BIT(instance->cr2_value, I2C_CR2_STOP) != 0)
    h7i2c_i2c_mutex_release_fromISR(peripheral);
}


#if H7I2C_PERIPH_ENABLE_I2C1 == 1
void I2C1_EV_IRQHandler(void)
{
  H7I2C_EV_IRQHandler_Impl(H7I2C_I2C1);
}
void I2C1_ER_IRQHandler(void)
{
  H7I2C_ER_IRQHandler_Impl(H7I2C_I2C1);
}
#endif

#if H7I2C_PERIPH_ENABLE_I2C2 == 1
void I2C2_EV_IRQHandler(void)
{
  H7I2C_EV_IRQHandler_Impl(H7I2C_I2C2);
}
void I2C2_ER_IRQHandler(void)
{
  H7I2C_ER_IRQHandler_Impl(H7I2C_I2C2);
}
#endif

#if H7I2C_PERIPH_ENABLE_I2C3 == 1
void I2C3_EV_IRQHandler(void)
{
  H7I2C_EV_IRQHandler_Impl(H7I2C_I2C3);
}
void I2C3_ER_IRQHandler(void)
{
  H7I2C_ER_IRQHandler_Impl(H7I2C_I2C3);
}
#endif

#if H7I2C_PERIPH_ENABLE_I2C4 == 1
void I2C4_EV_IRQHandler(void)
{
  H7I2C_EV_IRQHandler_Impl(H7I2C_I2C4);
}
void I2C4_ER_IRQHandler(void)
{
  H7I2C_ER_IRQHandler_Impl(H7I2C_I2C4);
}
#endif


static int h7i2c_i2c_pre_transaction_check(h7i2c_periph_t peripheral, uint32_t timeout)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  // Do not run if the driver is in error state
  switch (instance->fsm_state)
  {
    case H7I2C_FSM_STATE_ERROR_NACKF:
    case H7I2C_FSM_STATE_ERROR_BERR:
    case H7I2C_FSM_STATE_ERROR_ARLO:
    case H7I2C_FSM_STATE_ERROR_OVR:
    case H7I2C_FSM_STATE_ERROR_PECERR:
    case H7I2C_FSM_STATE_ERROR_TIMEOUT:
      return H7I2C_RET_CODE_PERIPH_IN_ERR_STATE;
    default:
      break;
  }

  // Lazy initialization. This branch usually should happen just once
  if (instance->fsm_state == H7I2C_FSM_STATE_UNINITIALIZED)
 	 	h7i2c_i2c_init(peripheral);

  instance->fsm_state = H7I2C_FSM_STATE_SETUP_TRANSFER;

  return H7I2C_RET_CODE_OK;
}


static int h7i2c_i2c_write_read_transaction(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t* wr_buf, uint8_t* rd_buf, uint32_t timeout)
{
  if ( (wr_size == 0UL || wr_buf == NULL) && (rd_size == 0UL || rd_buf == NULL) )
    return H7I2C_RET_CODE_INVALID_ARGS;

  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  if (h7i2c_i2c_mutex_lock(peripheral, timeout) != H7I2C_RET_CODE_OK)
    return H7I2C_RET_CODE_BUSY;

  int const check_ret_val = h7i2c_i2c_pre_transaction_check(peripheral, timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  {
    h7i2c_i2c_mutex_release(peripheral);
    return check_ret_val;
  }

  instance->timestart      = HAL_GetTick();
  instance->timeout        = timeout;
  instance->slave_address  = dev_address;

  instance->wr_todo        = wr_size;
  instance->wr_done        = 0;
  instance->wr_data        = wr_buf;

  instance->rd_todo        = rd_size;
  instance->rd_done        = 0;
  instance->rd_data        = rd_buf;

	if (instance->wr_todo > 0UL && instance->wr_data != NULL)
  // We will do a write first
  {
    if (instance->rd_todo > 0UL && instance->rd_data != NULL)
  		// We will do a write, then a read
    	instance->fsm_state = H7I2C_FSM_STATE_WRITEREAD_WRITESTEP;
    else
    	// We will do only the write
    	instance->fsm_state = H7I2C_FSM_STATE_WRITEONLY;
  }
  else
  // We will not do a write and only do a read (both null were checked at the beginning)
  {
    instance->fsm_state = H7I2C_FSM_STATE_READONLY;
  }

  // Default value for CR1
  instance->cr1_value =
    ( (1UL << I2C_CR1_PE_Pos       ) & I2C_CR1_PE        ) | // PE        = peripheral enable
    ( (0UL << I2C_CR1_TXIE_Pos     ) & I2C_CR1_TXIE      ) | // TXIE      = transmit event interrupt enable
    ( (0UL << I2C_CR1_RXIE_Pos     ) & I2C_CR1_RXIE      ) | // RXIE      = receive event interrupt enable
    ( (0UL << I2C_CR1_ADDRIE_Pos   ) & I2C_CR1_ADDRIE    ) | // ADDRIE    = own slave address interrupt enable
    ( (1UL << I2C_CR1_NACKIE_Pos   ) & I2C_CR1_NACKIE    ) | // NACKIE    = NACK detection interrupt enable
    ( (1UL << I2C_CR1_STOPIE_Pos   ) & I2C_CR1_STOPIE    ) | // STOPIE    = STOP detection interrupt enable
    ( (1UL << I2C_CR1_TCIE_Pos     ) & I2C_CR1_TCIE      ) | // TCIE      = master transfer complete
    ( (1UL << I2C_CR1_ERRIE_Pos    ) & I2C_CR1_ERRIE     ) | // ERRIE     = error interrupt enable (all errors)
    ( (0UL << I2C_CR1_DNF_Pos      ) & I2C_CR1_DNF       ) | // DNF       = digital noise filter period (0-15 clock periods)
    ( (0UL << I2C_CR1_ANFOFF_Pos   ) & I2C_CR1_ANFOFF    ) | // ANFOFF    = analog noise filter DISABLE (negative logic)
    ( (0UL << I2C_CR1_TXDMAEN_Pos  ) & I2C_CR1_TXDMAEN   ) | // TXDMAEN   = enable TX DMA signal generation
    ( (0UL << I2C_CR1_RXDMAEN_Pos  ) & I2C_CR1_RXDMAEN   ) | // RXDMAEN   = enable RX DMA signal generation
    ( (0UL << I2C_CR1_SBC_Pos      ) & I2C_CR1_SBC       ) | // SBC       = enable hardware byte control in slave mode
    ( (0UL << I2C_CR1_NOSTRETCH_Pos) & I2C_CR1_NOSTRETCH ) | // NOSTRETCH = clock stretching disable in slave mode
    ( (0UL << I2C_CR1_WUPEN_Pos    ) & I2C_CR1_WUPEN     ) | // WUPEN     = wake-up from MCU stop mode on slave address match enable
    ( (0UL << I2C_CR1_GCEN_Pos     ) & I2C_CR1_GCEN      ) | // GCEN      = general (i.e. broadcast) call slave 0x00 address match enable
    ( (0UL << I2C_CR1_SMBHEN_Pos   ) & I2C_CR1_SMBHEN    ) | // SMBHEN    = SMBus host address enable
    ( (0UL << I2C_CR1_SMBDEN_Pos   ) & I2C_CR1_SMBDEN    ) | // SMBDEN    = SMBus device default address enable
    ( (0UL << I2C_CR1_ALERTEN_Pos  ) & I2C_CR1_ALERTEN   ) | // ALERTEN   = SMBus alert enable
    ( (0UL << I2C_CR1_PECEN_Pos    ) & I2C_CR1_PECEN     ) ; // PECEN     = PEC byte enable

  // Default value for CR2
  instance->cr2_value =
    ( (0UL << I2C_CR2_SADD_Pos   ) & I2C_CR2_SADD   ) | // SADD    = slave address
    ( (0UL << I2C_CR2_RD_WRN_Pos ) & I2C_CR2_RD_WRN ) | // RD_WRN  = 0 (write) or 1 (read)
    ( (0UL << I2C_CR2_ADD10_Pos  ) & I2C_CR2_ADD10  ) | // ADD10   = 0 (7-bit addr) or 1 (10-bit addr)
    ( (0UL << I2C_CR2_HEAD10R_Pos) & I2C_CR2_HEAD10R) | // HEAD10R = 0 (send full 10-bit addr + r/w bit in 10-bit mode) or 1 (send 7-bit addr + r/w bit in 10-bit mode)
    ( (0UL << I2C_CR2_START_Pos  ) & I2C_CR2_START  ) | // START   = generate start bit and header, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_STOP_Pos   ) & I2C_CR2_STOP   ) | // STOP    = generate stop bit after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NACK_Pos   ) & I2C_CR2_NACK   ) | // NACK    = send a NACK after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES ) | // NBYTES  = number of bytes to transfer (8 bits, if more than 255 you need to reload)
    ( (0UL << I2C_CR2_RELOAD_Pos ) & I2C_CR2_RELOAD ) | // RELOAD  = reload mode, trigger TCR when NBYTES gets to zero *and stretch SCL*
    ( (0UL << I2C_CR2_AUTOEND_Pos) & I2C_CR2_AUTOEND) | // AUTOEND = generate a stop bit after the last byte (when NBYTES = 0) is transferred
    ( (0UL << I2C_CR2_PECBYTE_Pos) & I2C_CR2_PECBYTE) ; // PECBYTE = send the PEC byte after the last data byte  (this is relevant for SMBUS)

	MODIFY_REG(instance->cr2_value,  I2C_CR2_SADD, instance->slave_address << I2C_CR2_SADD_Pos   );


  switch (instance->fsm_state)
  {
    case H7I2C_FSM_STATE_WRITEREAD_WRITESTEP:
      SET_BIT(instance->cr1_value, I2C_CR1_TXIE);
      CLEAR_BIT(instance->cr2_value, I2C_CR2_RD_WRN );

      if (instance->wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(instance->cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, instance->wr_todo << I2C_CR2_NBYTES_Pos);
      }
      break;

    case H7I2C_FSM_STATE_WRITEONLY:
      SET_BIT(instance->cr1_value, I2C_CR1_TXIE);
      CLEAR_BIT(instance->cr2_value, I2C_CR2_RD_WRN );
      //SET_BIT(instance->cr2_value, I2C_CR2_AUTOEND);

      if (instance->wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(instance->cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, instance->wr_todo << I2C_CR2_NBYTES_Pos);
      }
      break;

    case H7I2C_FSM_STATE_READONLY:
      SET_BIT(instance->cr1_value, I2C_CR1_RXIE);
      SET_BIT(instance->cr2_value, I2C_CR2_RD_WRN);
      //SET_BIT(instance->cr2_value, I2C_CR2_AUTOEND);

      if (instance->wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(instance->cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, instance->rd_todo << I2C_CR2_NBYTES_Pos);
      }
      break;

    // We should not get to the ones below
    case H7I2C_FSM_STATE_WRITEREAD_READSTEP:
    case H7I2C_FSM_STATE_IDLE:
    default:
      break;
  }

  // Commit the new register values to hardware
  I2C_TypeDef* hardware = (I2C_TypeDef*) instance->i2c_base;

  hardware->CR1 = instance->cr1_value;
  READ_REG(hardware->CR1);

  hardware->CR2 = instance->cr2_value;
  READ_REG(hardware->CR2);

  SET_BIT(hardware->CR2, I2C_CR2_START);

  return H7I2C_RET_CODE_OK;
}

static int h7i2c_i2c_empty_write_transaction(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  if (h7i2c_i2c_mutex_lock(peripheral, timeout) != H7I2C_RET_CODE_OK)
    return H7I2C_RET_CODE_BUSY;

  int const check_ret_val = h7i2c_i2c_pre_transaction_check(peripheral, timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  {
    h7i2c_i2c_mutex_release(peripheral);
    return check_ret_val;
  }

  instance->timestart      = HAL_GetTick();
  instance->timeout        = timeout;
  instance->slave_address  = dev_address;

  instance->wr_todo        = 0;
  instance->wr_done        = 0;
  instance->wr_data        = NULL;

  instance->rd_todo        = 0;
  instance->rd_done        = 0;
  instance->rd_data        = NULL;

  instance->fsm_state = H7I2C_FSM_STATE_EMPTY_WRITE;

  // Default value for CR1
  instance->cr1_value =
    ( (1UL << I2C_CR1_PE_Pos       ) & I2C_CR1_PE        ) | // PE        = peripheral enable
    ( (0UL << I2C_CR1_TXIE_Pos     ) & I2C_CR1_TXIE      ) | // TXIE      = transmit event interrupt enable
    ( (0UL << I2C_CR1_RXIE_Pos     ) & I2C_CR1_RXIE      ) | // RXIE      = receive event interrupt enable
    ( (0UL << I2C_CR1_ADDRIE_Pos   ) & I2C_CR1_ADDRIE    ) | // ADDRIE    = own slave address interrupt enable
    ( (1UL << I2C_CR1_NACKIE_Pos   ) & I2C_CR1_NACKIE    ) | // NACKIE    = NACK detection interrupt enable
    ( (1UL << I2C_CR1_STOPIE_Pos   ) & I2C_CR1_STOPIE    ) | // STOPIE    = STOP detection interrupt enable
    ( (1UL << I2C_CR1_TCIE_Pos     ) & I2C_CR1_TCIE      ) | // TCIE      = master transfer complete
    ( (1UL << I2C_CR1_ERRIE_Pos    ) & I2C_CR1_ERRIE     ) | // ERRIE     = error interrupt enable (all errors)
    ( (0UL << I2C_CR1_DNF_Pos      ) & I2C_CR1_DNF       ) | // DNF       = digital noise filter period (0-15 clock periods)
    ( (0UL << I2C_CR1_ANFOFF_Pos   ) & I2C_CR1_ANFOFF    ) | // ANFOFF    = analog noise filter DISABLE (negative logic)
    ( (0UL << I2C_CR1_TXDMAEN_Pos  ) & I2C_CR1_TXDMAEN   ) | // TXDMAEN   = enable TX DMA signal generation
    ( (0UL << I2C_CR1_RXDMAEN_Pos  ) & I2C_CR1_RXDMAEN   ) | // RXDMAEN   = enable RX DMA signal generation
    ( (0UL << I2C_CR1_SBC_Pos      ) & I2C_CR1_SBC       ) | // SBC       = enable hardware byte control in slave mode
    ( (0UL << I2C_CR1_NOSTRETCH_Pos) & I2C_CR1_NOSTRETCH ) | // NOSTRETCH = clock stretching disable in slave mode
    ( (0UL << I2C_CR1_WUPEN_Pos    ) & I2C_CR1_WUPEN     ) | // WUPEN     = wake-up from MCU stop mode on slave address match enable
    ( (0UL << I2C_CR1_GCEN_Pos     ) & I2C_CR1_GCEN      ) | // GCEN      = general (i.e. broadcast) call slave 0x00 address match enable
    ( (0UL << I2C_CR1_SMBHEN_Pos   ) & I2C_CR1_SMBHEN    ) | // SMBHEN    = SMBus host address enable
    ( (0UL << I2C_CR1_SMBDEN_Pos   ) & I2C_CR1_SMBDEN    ) | // SMBDEN    = SMBus device default address enable
    ( (0UL << I2C_CR1_ALERTEN_Pos  ) & I2C_CR1_ALERTEN   ) | // ALERTEN   = SMBus alert enable
    ( (0UL << I2C_CR1_PECEN_Pos    ) & I2C_CR1_PECEN     ) ; // PECEN     = PEC byte enable

  // Default value for CR2
  instance->cr2_value =
    ( (0UL << I2C_CR2_SADD_Pos   ) & I2C_CR2_SADD   ) | // SADD    = slave address
    ( (0UL << I2C_CR2_RD_WRN_Pos ) & I2C_CR2_RD_WRN ) | // RD_WRN  = 0 (write) or 1 (read)
    ( (0UL << I2C_CR2_ADD10_Pos  ) & I2C_CR2_ADD10  ) | // ADD10   = 0 (7-bit addr) or 1 (10-bit addr)
    ( (0UL << I2C_CR2_HEAD10R_Pos) & I2C_CR2_HEAD10R) | // HEAD10R = 0 (send full 10-bit addr + r/w bit in 10-bit mode) or 1 (send 7-bit addr + r/w bit in 10-bit mode)
    ( (0UL << I2C_CR2_START_Pos  ) & I2C_CR2_START  ) | // START   = generate start bit and header, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_STOP_Pos   ) & I2C_CR2_STOP   ) | // STOP    = generate stop bit after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NACK_Pos   ) & I2C_CR2_NACK   ) | // NACK    = send a NACK after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES ) | // NBYTES  = number of bytes to transfer (8 bits, if more than 255 you need to reload)
    ( (0UL << I2C_CR2_RELOAD_Pos ) & I2C_CR2_RELOAD ) | // RELOAD  = reload mode, trigger TCR when NBYTES gets to zero *and stretch SCL*
    ( (1UL << I2C_CR2_AUTOEND_Pos) & I2C_CR2_AUTOEND) | // AUTOEND = generate a stop bit after the last byte (when NBYTES = 0) is transferred
    ( (0UL << I2C_CR2_PECBYTE_Pos) & I2C_CR2_PECBYTE) ; // PECBYTE = send the PEC byte after the last data byte  (this is relevant for SMBUS)

  MODIFY_REG(instance->cr2_value,  I2C_CR2_SADD, (instance->slave_address << I2C_CR2_SADD_Pos   ));

  // Commit the new register values to hardware
  I2C_TypeDef* hardware = (I2C_TypeDef*) instance->i2c_base;

  hardware->CR1 = instance->cr1_value;
  READ_REG(hardware->CR1);

  hardware->CR2 = instance->cr2_value;
  READ_REG(hardware->CR2);

  SET_BIT(hardware->CR2, I2C_CR2_START);

  return H7I2C_RET_CODE_OK;
}

static int h7i2c_i2c_empty_read_transaction(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout)
{
  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  if (h7i2c_i2c_mutex_lock(peripheral, timeout) != H7I2C_RET_CODE_OK)
    return H7I2C_RET_CODE_BUSY;

  int const check_ret_val = h7i2c_i2c_pre_transaction_check(peripheral, timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  {
    h7i2c_i2c_mutex_release(peripheral);
    return check_ret_val;
  }

  instance->timestart      = HAL_GetTick();
  instance->timeout        = timeout;
  instance->slave_address  = dev_address;

  instance->wr_todo        = 0;
  instance->wr_done        = 0;
  instance->wr_data        = NULL;

  instance->rd_todo        = 0;
  instance->rd_done        = 0;
  instance->rd_data        = NULL;

  instance->fsm_state = H7I2C_FSM_STATE_EMPTY_READ;

  // Default value for CR1
  instance->cr1_value =
    ( (1UL << I2C_CR1_PE_Pos       ) & I2C_CR1_PE        ) | // PE        = peripheral enable
    ( (0UL << I2C_CR1_TXIE_Pos     ) & I2C_CR1_TXIE      ) | // TXIE      = transmit event interrupt enable
    ( (0UL << I2C_CR1_RXIE_Pos     ) & I2C_CR1_RXIE      ) | // RXIE      = receive event interrupt enable
    ( (0UL << I2C_CR1_ADDRIE_Pos   ) & I2C_CR1_ADDRIE    ) | // ADDRIE    = own slave address interrupt enable
    ( (1UL << I2C_CR1_NACKIE_Pos   ) & I2C_CR1_NACKIE    ) | // NACKIE    = NACK detection interrupt enable
    ( (1UL << I2C_CR1_STOPIE_Pos   ) & I2C_CR1_STOPIE    ) | // STOPIE    = STOP detection interrupt enable
    ( (1UL << I2C_CR1_TCIE_Pos     ) & I2C_CR1_TCIE      ) | // TCIE      = master transfer complete
    ( (1UL << I2C_CR1_ERRIE_Pos    ) & I2C_CR1_ERRIE     ) | // ERRIE     = error interrupt enable (all errors)
    ( (0UL << I2C_CR1_DNF_Pos      ) & I2C_CR1_DNF       ) | // DNF       = digital noise filter period (0-15 clock periods)
    ( (0UL << I2C_CR1_ANFOFF_Pos   ) & I2C_CR1_ANFOFF    ) | // ANFOFF    = analog noise filter DISABLE (negative logic)
    ( (0UL << I2C_CR1_TXDMAEN_Pos  ) & I2C_CR1_TXDMAEN   ) | // TXDMAEN   = enable TX DMA signal generation
    ( (0UL << I2C_CR1_RXDMAEN_Pos  ) & I2C_CR1_RXDMAEN   ) | // RXDMAEN   = enable RX DMA signal generation
    ( (0UL << I2C_CR1_SBC_Pos      ) & I2C_CR1_SBC       ) | // SBC       = enable hardware byte control in slave mode
    ( (0UL << I2C_CR1_NOSTRETCH_Pos) & I2C_CR1_NOSTRETCH ) | // NOSTRETCH = clock stretching disable in slave mode
    ( (0UL << I2C_CR1_WUPEN_Pos    ) & I2C_CR1_WUPEN     ) | // WUPEN     = wake-up from MCU stop mode on slave address match enable
    ( (0UL << I2C_CR1_GCEN_Pos     ) & I2C_CR1_GCEN      ) | // GCEN      = general (i.e. broadcast) call slave 0x00 address match enable
    ( (0UL << I2C_CR1_SMBHEN_Pos   ) & I2C_CR1_SMBHEN    ) | // SMBHEN    = SMBus host address enable
    ( (0UL << I2C_CR1_SMBDEN_Pos   ) & I2C_CR1_SMBDEN    ) | // SMBDEN    = SMBus device default address enable
    ( (0UL << I2C_CR1_ALERTEN_Pos  ) & I2C_CR1_ALERTEN   ) | // ALERTEN   = SMBus alert enable
    ( (0UL << I2C_CR1_PECEN_Pos    ) & I2C_CR1_PECEN     ) ; // PECEN     = PEC byte enable

  // Default value for CR2
  instance->cr2_value =
    ( (0UL << I2C_CR2_SADD_Pos   ) & I2C_CR2_SADD   ) | // SADD    = slave address
    ( (1UL << I2C_CR2_RD_WRN_Pos ) & I2C_CR2_RD_WRN ) | // RD_WRN  = 0 (write) or 1 (read)
    ( (0UL << I2C_CR2_ADD10_Pos  ) & I2C_CR2_ADD10  ) | // ADD10   = 0 (7-bit addr) or 1 (10-bit addr)
    ( (0UL << I2C_CR2_HEAD10R_Pos) & I2C_CR2_HEAD10R) | // HEAD10R = 0 (send full 10-bit addr + r/w bit in 10-bit mode) or 1 (send 7-bit addr + r/w bit in 10-bit mode)
    ( (0UL << I2C_CR2_START_Pos  ) & I2C_CR2_START  ) | // START   = generate start bit and header, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_STOP_Pos   ) & I2C_CR2_STOP   ) | // STOP    = generate stop bit after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NACK_Pos   ) & I2C_CR2_NACK   ) | // NACK    = send a NACK after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES ) | // NBYTES  = number of bytes to transfer (8 bits, if more than 255 you need to reload)
    ( (0UL << I2C_CR2_RELOAD_Pos ) & I2C_CR2_RELOAD ) | // RELOAD  = reload mode, trigger TCR when NBYTES gets to zero *and stretch SCL*
    ( (1UL << I2C_CR2_AUTOEND_Pos) & I2C_CR2_AUTOEND) | // AUTOEND = generate a stop bit after the last byte (when NBYTES = 0) is transferred
    ( (0UL << I2C_CR2_PECBYTE_Pos) & I2C_CR2_PECBYTE) ; // PECBYTE = send the PEC byte after the last data byte  (this is relevant for SMBUS)

  MODIFY_REG(instance->cr2_value,  I2C_CR2_SADD, (instance->slave_address << I2C_CR2_SADD_Pos   ));

  // Commit the new register values to hardware
  I2C_TypeDef* hardware = (I2C_TypeDef*) instance->i2c_base;

  hardware->CR1 = instance->cr1_value;
  READ_REG(hardware->CR1);

  hardware->CR2 = instance->cr2_value;
  READ_REG(hardware->CR2);

  SET_BIT(hardware->CR2, I2C_CR2_START);

  return H7I2C_RET_CODE_OK;
}



static int h7i2c_smbus_write_slave_driven_read_transaction(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t wr_size, uint8_t* wr_buf, uint8_t* rd_buf, uint32_t timeout)
{
  if (rd_buf == NULL)
    return H7I2C_RET_CODE_INVALID_ARGS;

  h7i2c_driver_instance_state_t* instance = h7i2c_get_driver_instance(peripheral);

  if (!instance)
    return H7I2C_RET_CODE_UNMANAGED_BY_DRIVER;

  if (h7i2c_i2c_mutex_lock(peripheral, timeout) != H7I2C_RET_CODE_OK)
    return H7I2C_RET_CODE_BUSY;

  int const check_ret_val = h7i2c_i2c_pre_transaction_check(peripheral, timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  {
    h7i2c_i2c_mutex_release(peripheral);
    return check_ret_val;
  }

  instance->timestart      = HAL_GetTick();
  instance->timeout        = timeout;
  instance->slave_address  = dev_address;

  instance->wr_todo        = wr_size;
  instance->wr_done        = 0;
  instance->wr_data        = wr_buf;

  instance->rd_todo        = 1;
  instance->rd_done        = 0;
  instance->rd_data        = rd_buf;

  if (instance->rd_data != NULL)
  {
    // We will do a write first
    if (instance->wr_todo > 0UL && instance->wr_data != NULL)
      // We will do a write, then a read
      instance->fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP;
    else
      // We will do only the read
      instance->fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXNBYTES;
  }

  // Default value for CR1
  instance->cr1_value =
    ( (1UL << I2C_CR1_PE_Pos       ) & I2C_CR1_PE        ) | // PE        = peripheral enable
    ( (0UL << I2C_CR1_TXIE_Pos     ) & I2C_CR1_TXIE      ) | // TXIE      = transmit event interrupt enable
    ( (0UL << I2C_CR1_RXIE_Pos     ) & I2C_CR1_RXIE      ) | // RXIE      = receive event interrupt enable
    ( (0UL << I2C_CR1_ADDRIE_Pos   ) & I2C_CR1_ADDRIE    ) | // ADDRIE    = own slave address interrupt enable
    ( (1UL << I2C_CR1_NACKIE_Pos   ) & I2C_CR1_NACKIE    ) | // NACKIE    = NACK detection interrupt enable
    ( (1UL << I2C_CR1_STOPIE_Pos   ) & I2C_CR1_STOPIE    ) | // STOPIE    = STOP detection interrupt enable
    ( (1UL << I2C_CR1_TCIE_Pos     ) & I2C_CR1_TCIE      ) | // TCIE      = master transfer complete
    ( (1UL << I2C_CR1_ERRIE_Pos    ) & I2C_CR1_ERRIE     ) | // ERRIE     = error interrupt enable (all errors)
    ( (0UL << I2C_CR1_DNF_Pos      ) & I2C_CR1_DNF       ) | // DNF       = digital noise filter period (0-15 clock periods)
    ( (0UL << I2C_CR1_ANFOFF_Pos   ) & I2C_CR1_ANFOFF    ) | // ANFOFF    = analog noise filter DISABLE (negative logic)
    ( (0UL << I2C_CR1_TXDMAEN_Pos  ) & I2C_CR1_TXDMAEN   ) | // TXDMAEN   = enable TX DMA signal generation
    ( (0UL << I2C_CR1_RXDMAEN_Pos  ) & I2C_CR1_RXDMAEN   ) | // RXDMAEN   = enable RX DMA signal generation
    ( (0UL << I2C_CR1_SBC_Pos      ) & I2C_CR1_SBC       ) | // SBC       = enable hardware byte control in slave mode
    ( (0UL << I2C_CR1_NOSTRETCH_Pos) & I2C_CR1_NOSTRETCH ) | // NOSTRETCH = clock stretching disable in slave mode
    ( (0UL << I2C_CR1_WUPEN_Pos    ) & I2C_CR1_WUPEN     ) | // WUPEN     = wake-up from MCU stop mode on slave address match enable
    ( (0UL << I2C_CR1_GCEN_Pos     ) & I2C_CR1_GCEN      ) | // GCEN      = general (i.e. broadcast) call slave 0x00 address match enable
    ( (0UL << I2C_CR1_SMBHEN_Pos   ) & I2C_CR1_SMBHEN    ) | // SMBHEN    = SMBus host address enable
    ( (0UL << I2C_CR1_SMBDEN_Pos   ) & I2C_CR1_SMBDEN    ) | // SMBDEN    = SMBus device default address enable
    ( (0UL << I2C_CR1_ALERTEN_Pos  ) & I2C_CR1_ALERTEN   ) | // ALERTEN   = SMBus alert enable
    ( (0UL << I2C_CR1_PECEN_Pos    ) & I2C_CR1_PECEN     ) ; // PECEN     = PEC byte enable

  // Default value for CR2
  instance->cr2_value =
    ( (0UL << I2C_CR2_SADD_Pos   ) & I2C_CR2_SADD   ) | // SADD    = slave address
    ( (0UL << I2C_CR2_RD_WRN_Pos ) & I2C_CR2_RD_WRN ) | // RD_WRN  = 0 (write) or 1 (read)
    ( (0UL << I2C_CR2_ADD10_Pos  ) & I2C_CR2_ADD10  ) | // ADD10   = 0 (7-bit addr) or 1 (10-bit addr)
    ( (0UL << I2C_CR2_HEAD10R_Pos) & I2C_CR2_HEAD10R) | // HEAD10R = 0 (send full 10-bit addr + r/w bit in 10-bit mode) or 1 (send 7-bit addr + r/w bit in 10-bit mode)
    ( (0UL << I2C_CR2_START_Pos  ) & I2C_CR2_START  ) | // START   = generate start bit and header, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_STOP_Pos   ) & I2C_CR2_STOP   ) | // STOP    = generate stop bit after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NACK_Pos   ) & I2C_CR2_NACK   ) | // NACK    = send a NACK after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES ) | // NBYTES  = number of bytes to transfer (8 bits, if more than 255 you need to reload)
    ( (0UL << I2C_CR2_RELOAD_Pos ) & I2C_CR2_RELOAD ) | // RELOAD  = reload mode, trigger TCR when NBYTES gets to zero *and stretch SCL*
    ( (0UL << I2C_CR2_AUTOEND_Pos) & I2C_CR2_AUTOEND) | // AUTOEND = generate a stop bit after the last byte (when NBYTES = 0) is transferred
    ( (0UL << I2C_CR2_PECBYTE_Pos) & I2C_CR2_PECBYTE) ; // PECBYTE = send the PEC byte after the last data byte  (this is relevant for SMBUS)

  MODIFY_REG(instance->cr2_value,  I2C_CR2_SADD, (instance->slave_address << I2C_CR2_SADD_Pos   ));

  switch (instance->fsm_state)
  {
    case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP:
      SET_BIT(instance->cr1_value,  I2C_CR1_TXIE);

      if (instance->wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(instance->cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(instance->cr2_value, I2C_CR2_NBYTES, instance->wr_todo << I2C_CR2_NBYTES_Pos);
      }
      break;

    default:
      break;
  }

  // Commit the new register values to hardware
  I2C_TypeDef* hardware = (I2C_TypeDef*) instance->i2c_base;

  hardware->CR1 = instance->cr1_value;
  READ_REG(hardware->CR1);

  hardware->CR2 = instance->cr2_value;
  READ_REG(hardware->CR2);

  SET_BIT(hardware->CR2, I2C_CR2_START);

  return H7I2C_RET_CODE_OK;
}




////////////////////////////
//                        //
// NON-BLOCKING FUNCTIONS //
//                        //
////////////////////////////
h7i2c_i2c_ret_code_t h7i2c_i2c_write_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, data_size, 0U, data_buf, NULL, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_i2c_read_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 0U, data_size, NULL, data_buf, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_i2c_write_then_read_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t *wr_buf, uint8_t *rd_buf, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, wr_size, rd_size, wr_buf, rd_buf, timeout);
}


h7i2c_i2c_ret_code_t h7i2c_smbus_quickcommand_write_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout)
{
  return h7i2c_i2c_empty_write_transaction(peripheral, dev_address, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_smbus_quickcommand_read_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout)
{
  return h7i2c_i2c_empty_read_transaction(peripheral, dev_address, timeout);
}


h7i2c_i2c_ret_code_t h7i2c_smbus_sendbyte_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 1U, 0U, byte, NULL, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_smbus_receivebyte_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 0U, 1U, NULL, byte, timeout);
}


h7i2c_i2c_ret_code_t h7i2c_smbus_writebyte_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  uint8_t tx_buf[2];
  tx_buf[0] = command;
  tx_buf[1] = byte[0];
	return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 2U, 0U, tx_buf, NULL, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_smbus_readbyte_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 1U, 1U, &command, byte, timeout);
}


h7i2c_i2c_ret_code_t h7i2c_smbus_writeword_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
  uint8_t wr_buf[3];
  wr_buf[0] = command;
  wr_buf[1] = (word[0] & 0x00FF);
  wr_buf[2] = (word[0] & 0xFF00) >> 8U;
	return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 3U, 0U, wr_buf, NULL, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_smbus_readword_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
  uint8_t rd_buf[2];
  uint32_t ret = h7i2c_i2c_write_read_transaction(peripheral, dev_address, 1U, 2U, &command, rd_buf, timeout);
  word[0] = ((uint16_t)rd_buf[0]) + ( ((uint16_t)rd_buf[1]) << 8U );
  return ret;
}


h7i2c_i2c_ret_code_t h7i2c_smbus_processcall_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* wr_word, uint16_t* rd_word, uint32_t timeout)
{
  uint8_t tx_buf[3];
  uint8_t rx_buf[2];
  tx_buf[0] = command;
  tx_buf[1] = (wr_word[0] & 0x00FF);
  tx_buf[2] = (wr_word[0] & 0xFF00) >> 8U;
  uint32_t ret = h7i2c_i2c_write_read_transaction(peripheral, dev_address, 3U, 2U, tx_buf, rx_buf, timeout);
  rd_word[0] = ((uint16_t)rx_buf[0]) + ( ((uint16_t)rx_buf[1]) << 8U );
  return ret;
}


h7i2c_i2c_ret_code_t h7i2c_smbus_blockwrite_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* wr_buf, uint32_t timeout)
{
  uint8_t tx_buf[257];
  tx_buf[0] = command;
  tx_buf[1] = wr_size;
  memcpy(&(tx_buf[2]), wr_buf, wr_size);
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 2U + wr_size, 0U, tx_buf, NULL, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_smbus_blockread_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* rd_size, uint8_t* rd_buf, uint32_t timeout)
{
  uint8_t rx_buf[256];
  uint32_t ret = h7i2c_smbus_write_slave_driven_read_transaction(peripheral, dev_address, 1U, &command, rd_buf, timeout);
  *rd_size = rx_buf[0];
  memcpy(&(rx_buf[1]), rd_buf, *rd_size);
  return ret;
}


h7i2c_i2c_ret_code_t h7i2c_smbus_blockwrite_blockread_processcall_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command,
  uint8_t wr_size, uint8_t* rd_size, uint16_t *wr_buf, uint16_t *rd_buf, uint32_t timeout)
{
  uint8_t tx_buf[257];
  uint8_t rx_buf[256];
  tx_buf[0] = command;
  tx_buf[1] = wr_size;
  uint32_t ret = h7i2c_smbus_write_slave_driven_read_transaction(peripheral, dev_address, wr_size, tx_buf, rx_buf, timeout);
  *rd_size = rx_buf[0];
  memcpy(&(rx_buf[1]), rd_buf, *rd_size);
  return ret;
}


h7i2c_i2c_ret_code_t h7i2c_smbus_write32_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  uint8_t wr_buf[5];
  wr_buf[0] = command;
  wr_buf[1] = (word32[0] & 0x000000FFUL);
  wr_buf[2] = (word32[0] & 0x0000FF00UL) >>  8U;
  wr_buf[3] = (word32[0] & 0x00FF0000UL) >> 16U;
  wr_buf[4] = (word32[0] & 0xFF000000UL) >> 24U;
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 5U, 0U, wr_buf, NULL, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_smbus_read32_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  uint8_t rd_buf[4];
  uint32_t ret = h7i2c_i2c_write_read_transaction(peripheral, dev_address, 1U, 4U, &command, rd_buf, timeout);
  word32[0] =
        ( ((uint32_t)rd_buf[0])        )
      + ( ((uint32_t)rd_buf[1]) <<  8U )
      + ( ((uint32_t)rd_buf[2]) << 16U )
      + ( ((uint32_t)rd_buf[3]) << 24U );
  return ret;
}


h7i2c_i2c_ret_code_t h7i2c_smbus_write64_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
  uint8_t wr_buf[9];
  wr_buf[0] = command;
  wr_buf[1] = (word64[0] & 0x00000000000000FFULL);
  wr_buf[2] = (word64[0] & 0x000000000000FF00ULL) >>  8U;
  wr_buf[3] = (word64[0] & 0x0000000000FF0000ULL) >> 16U;
  wr_buf[4] = (word64[0] & 0x00000000FF000000ULL) >> 24U;
  wr_buf[5] = (word64[0] & 0x000000FF00000000ULL) >> 32U;
  wr_buf[6] = (word64[0] & 0x0000FF0000000000ULL) >> 40U;
  wr_buf[7] = (word64[0] & 0x00FF000000000000ULL) >> 48U;
  wr_buf[8] = (word64[0] & 0xFF00000000000000ULL) >> 56U;
  return h7i2c_i2c_write_read_transaction(peripheral, dev_address, 9U, 0U, wr_buf, NULL, timeout);
}

h7i2c_i2c_ret_code_t h7i2c_smbus_read64_nonblocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
  uint8_t rd_buf[8];
  uint32_t ret = h7i2c_i2c_write_read_transaction(peripheral, dev_address, 1U, 8U, &command, rd_buf, timeout);
  word64[0] =
        ( ((uint64_t)rd_buf[0])        )
      + ( ((uint64_t)rd_buf[1]) <<  8U )
      + ( ((uint64_t)rd_buf[2]) << 16U )
      + ( ((uint64_t)rd_buf[3]) << 24U )
      + ( ((uint64_t)rd_buf[4]) << 32U )
      + ( ((uint64_t)rd_buf[5]) << 40U )
      + ( ((uint64_t)rd_buf[6]) << 48U )
      + ( ((uint64_t)rd_buf[7]) << 56U );
  return ret;
}




////////////////////////
//                    //
// BLOCKING FUNCTIONS //
//                    //
////////////////////////
h7i2c_i2c_ret_code_t h7i2c_i2c_write(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_i2c_write_nonblocking(peripheral, dev_address, data_size, data_buf, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_i2c_read(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_i2c_read_nonblocking(peripheral, dev_address, data_size, data_buf, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_i2c_write_then_read(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t *wr_buf, uint8_t *rd_buf, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_i2c_write_then_read_nonblocking(peripheral, dev_address, wr_size, rd_size, wr_buf, rd_buf, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_quickcommand_write(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_quickcommand_write_nonblocking(peripheral, dev_address, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_quickcommand_read(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_quickcommand_read_nonblocking(peripheral, dev_address, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_sendbyte(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_sendbyte_nonblocking(peripheral, dev_address, byte, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_receivebyte(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_receivebyte_nonblocking(peripheral, dev_address, byte, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_writebyte(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_writebyte_nonblocking(peripheral, dev_address, command, byte, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_readbyte(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_readbyte_nonblocking(peripheral, dev_address, command, byte, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_writeword(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_writeword_nonblocking(peripheral, dev_address, command, word, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_readword(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_readword_nonblocking(peripheral, dev_address, command, word, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_processcall(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* wr_word, uint16_t* rd_word, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_processcall_nonblocking(peripheral, dev_address, command, wr_word, rd_word, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_blockwrite(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* wr_buf, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_blockwrite_nonblocking(peripheral, dev_address, command, wr_size, wr_buf, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_blockread(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* rd_size, uint8_t* rd_buf, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_blockread_nonblocking(peripheral, dev_address, command, rd_size, rd_buf, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_blockwrite_blockread_processcall(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t wr_size,
  uint8_t* rd_size, uint16_t *wr_buf, uint16_t *rd_buf, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_blockwrite_blockread_processcall_nonblocking(peripheral, dev_address, command, wr_size, rd_size, wr_buf, rd_buf, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_write32(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_write32_nonblocking(peripheral, dev_address, command, word32, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_read32(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_read32_nonblocking(peripheral, dev_address, command, word32, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_write64(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_write64_nonblocking(peripheral, dev_address, command, word64, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}

h7i2c_i2c_ret_code_t h7i2c_smbus_read64(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
  h7i2c_i2c_ret_code_t ret = h7i2c_smbus_read64_nonblocking(peripheral, dev_address, command, word64, timeout);
  if (ret != H7I2C_RET_CODE_OK)
  {
    h7i2c_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return h7i2c_wait_until_ready(peripheral, timeout);
  }
}
