
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

#include "h7i2c_config.h"


#if H7I2C_USE_FREERTOS_IMPL == 1

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "h7i2c_rtos.h"
#include "h7i2c_bare.h"


#if H7I2C_PERIPH_ENABLE_I2C1 == 1
extern h7i2c_driver_instance_state_t H7I2C_I2C1;
#endif

#if H7I2C_PERIPH_ENABLE_I2C2 == 1
extern h7i2c_driver_instance_state_t H7I2C_I2C2;
#endif

#if H7I2C_PERIPH_ENABLE_I2C3 == 1
extern h7i2c_driver_instance_state_t H7I2C_I2C3;
#endif

#if H7I2C_PERIPH_ENABLE_I2C4 == 1
extern h7i2c_driver_instance_state_t H7I2C_I2C4;
#endif


h7i2c_rtos_driver_instance_state_t H7I2C_RTOS_I2C1 =
#if H7I2C_PERIPH_ENABLE_I2C1 == 1
{
  .bare_driver_instance = &H7I2C_I2C1,
  .mutex_rtos = NULL
};
#else
{
  .bare_driver_instance = NULL,
  .mutex_rtos = NULL
};
#endif


h7i2c_rtos_driver_instance_state_t H7I2C_RTOS_I2C2 =
#if H7I2C_PERIPH_ENABLE_I2C2 == 1
{
  .bare_driver_instance = &H7I2C_I2C2,
  .mutex_rtos = NULL
};
#else
{
  .bare_driver_instance = NULL,
  .mutex_rtos = NULL
};
#endif


h7i2c_rtos_driver_instance_state_t H7I2C_RTOS_I2C3 =
#if H7I2C_PERIPH_ENABLE_I2C3 == 1
{
  .bare_driver_instance = &H7I2C_I2C3,
  .mutex_rtos = NULL
};
#else
{
  .bare_driver_instance = NULL,
  .mutex_rtos = NULL
};
#endif


h7i2c_rtos_driver_instance_state_t H7I2C_RTOS_I2C4 =
#if H7I2C_PERIPH_ENABLE_I2C4 == 1
{
  .bare_driver_instance = &H7I2C_I2C4,
  .mutex_rtos = NULL
};
#else
{
  .bare_driver_instance = NULL,
  .mutex_rtos = NULL
};
#endif


int h7i2c_i2c_mutex_lock(h7i2c_driver_instance_state_t* instance, uint32_t timeout)
{
  h7i2c_rtos_driver_instance_state_t* instance_rtos = NULL;

  switch ((uint32_t) instance->i2c_base)
  {
    case I2C1_BASE:
      instance_rtos = &H7I2C_RTOS_I2C1;
      break;

    case I2C2_BASE:
      instance_rtos = &H7I2C_RTOS_I2C2;
      break;

    case I2C3_BASE:
      instance_rtos = &H7I2C_RTOS_I2C3;
      break;

    case I2C4_BASE:
      instance_rtos = &H7I2C_RTOS_I2C4;
      break;

    default:
      Error_Handler();
  }

  uint32_t const timestart = HAL_GetTick();

  while (instance_rtos->mutex_rtos == NULL)
  {
    if (HAL_GetTick() - timestart >= timeout)
    {
      return H7I2C_RET_CODE_BUSY;
    }
    instance_rtos->mutex_rtos = xSemaphoreCreateMutexStatic(&(instance_rtos->mutex_rtos_buffer));
  }

  uint32_t const timenow = HAL_GetTick();
  if (timenow - timestart >= timeout)
  {
    return H7I2C_RET_CODE_BUSY;
  }

  timeout -= timenow - timestart;
  if (pdTRUE == xSemaphoreTake(instance_rtos->mutex_rtos, (TickType_t) timeout))
  {
    return H7I2C_RET_CODE_OK;
  }

  return H7I2C_RET_CODE_BUSY;
}


void h7i2c_i2c_mutex_release(h7i2c_driver_instance_state_t* instance)
{
  h7i2c_rtos_driver_instance_state_t* instance_rtos = NULL;

  switch ((uint32_t) instance->i2c_base)
  {
    case I2C1_BASE:
      instance_rtos = &H7I2C_RTOS_I2C1;
      break;

    case I2C2_BASE:
      instance_rtos = &H7I2C_RTOS_I2C2;
      break;

    case I2C3_BASE:
      instance_rtos = &H7I2C_RTOS_I2C3;
      break;

    case I2C4_BASE:
      instance_rtos = &H7I2C_RTOS_I2C4;
      break;

    default:
      Error_Handler();
  }

  if (instance_rtos->mutex_rtos != NULL)
    xSemaphoreGive(instance_rtos->mutex_rtos);
}


void h7i2c_i2c_mutex_release_fromISR(h7i2c_driver_instance_state_t* instance)
{
  h7i2c_rtos_driver_instance_state_t* instance_rtos = NULL;

  switch ((uint32_t) instance->i2c_base)
  {
    case I2C1_BASE:
      instance_rtos = &H7I2C_RTOS_I2C1;
      break;

    case I2C2_BASE:
      instance_rtos = &H7I2C_RTOS_I2C2;
      break;

    case I2C3_BASE:
      instance_rtos = &H7I2C_RTOS_I2C3;
      break;

    case I2C4_BASE:
      instance_rtos = &H7I2C_RTOS_I2C4;
      break;

    default:
      Error_Handler();
  }

  if (instance_rtos->mutex_rtos != NULL)
  {
    //signed BaseType_t xHigherPriorityTaskWoken;
    signed long xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(instance_rtos->mutex_rtos, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}


int h7i2c_i2c_rtos_waitidle(h7i2c_driver_instance_state_t* instance, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // Blocking loop, waiting for the I2C peripheral to become free, to get into error state or to timeout
  while (1)
  {
    switch (h7i2c_get_state(instance))
    {
      case H7I2C_FSM_STATE_ERROR_NACKF:
      case H7I2C_FSM_STATE_ERROR_BERR:
      case H7I2C_FSM_STATE_ERROR_ARLO:
      case H7I2C_FSM_STATE_ERROR_OVR:
      case H7I2C_FSM_STATE_ERROR_PECERR:
      case H7I2C_FSM_STATE_ERROR_TIMEOUT:
        return H7I2C_RET_CODE_PERIPH_IN_ERR_STATE;

      case H7I2C_FSM_STATE_UNINITIALIZED:
      case H7I2C_FSM_STATE_IDLE:
        return H7I2C_RET_CODE_OK;

      default:
        if (HAL_GetTick() - timestart < timeout)
        {
          taskYIELD();
          break;
        }
        else
        {
          return H7I2C_RET_CODE_TIMEOUT;
        }
    }
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_i2c_write_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_i2c_write(instance, dev_address, data_size, data_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_i2c_read_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_i2c_read(instance, dev_address, data_size, data_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_i2c_write_then_read_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t *wr_buf, uint8_t *rd_buf, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_i2c_write_then_read(instance, dev_address, wr_size, rd_size, wr_buf, rd_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_quickcommand_write_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_quickcommand_write(instance, dev_address, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_quickcommand_read_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_quickcommand_read(instance, dev_address, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_sendbyte_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_sendbyte(instance, dev_address, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_receivebyte_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_receivebyte(instance, dev_address, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_writebyte_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_writebyte(instance, dev_address, command, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_readbyte_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_readbyte(instance, dev_address, command, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_writeword_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_writeword(instance, dev_address, command, word, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_readword_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_readword(instance, dev_address, command, word, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_processcall_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint16_t* wr_word, uint16_t* rd_word, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_processcall(instance, dev_address, command, wr_word, rd_word, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_blockwrite_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* wr_buf, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_blockwrite(instance, dev_address, command, wr_size, wr_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_blockread_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint8_t* rd_size, uint8_t* rd_buf, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_blockread(instance, dev_address, command, rd_size, rd_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_blockwritereadprocesscall_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* rd_size, uint16_t *wr_buf, uint16_t *rd_buf, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_blockwritereadprocesscall(instance, dev_address, command, wr_size, rd_size, wr_buf, rd_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_write32_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_write32(instance, dev_address, command, word32, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_read32_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_read32(instance, dev_address, command, word32, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_write64_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_write64(instance, dev_address, command, word64, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_read64_rtos_blocking(h7i2c_driver_instance_state_t* instance, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  retval = h7i2c_smbus_read64(instance, dev_address, command, word64, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  if (HAL_GetTick() - timestart > timeout)
  {
    return H7I2C_RET_CODE_TIMEOUT;
  }

  timeout = timeout - (HAL_GetTick() - timestart);

  retval = h7i2c_i2c_rtos_waitidle(instance, timeout);
  if (retval != H7I2C_RET_CODE_OK)
  {
    return retval;
  }

  return H7I2C_RET_CODE_OK;
}


#endif // H7I2C_USE_FREERTOS_IMPL
