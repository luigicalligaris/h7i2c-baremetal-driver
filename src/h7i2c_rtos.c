

/*********************************************************************************************/
/* STM32H7 I2C bare-metal driver                                                             */
/* Copyright (c) 2022, Luigi Calligaris                                                      */
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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "h7i2c_rtos.h"
#include "h7i2c_bare.h"


static SemaphoreHandle_t h7i2c_i2c_mutex_rtos = NULL;
static StaticSemaphore_t h7i2c_i2c_mutex_rtos_buffer;

int h7i2c_i2c_mutex_lock(uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

  while (h7i2c_i2c_mutex_rtos == NULL)
  {
    if (HAL_GetTick() - timestart >= timeout)
    {
      return H7I2C_RET_CODE_BUSY;
    }
    h7i2c_i2c_mutex_rtos = xSemaphoreCreateMutexStatic(&h7i2c_i2c_mutex_rtos_buffer);
  }

  uint32_t const timenow = HAL_GetTick();
  if (timenow - timestart >= timeout)
  {
  	return H7I2C_RET_CODE_BUSY;
  }

  timeout -= timenow - timestart;
  if (pdTRUE == xSemaphoreTake(h7i2c_i2c_mutex_rtos, (TickType_t) timeout))
  {
  	return H7I2C_RET_CODE_OK;
  }

  return H7I2C_RET_CODE_BUSY;
}

void h7i2c_i2c_mutex_release()
{
	if (h7i2c_i2c_mutex_rtos != NULL)
		xSemaphoreGive(h7i2c_i2c_mutex_rtos);
}

void h7i2c_i2c_mutex_release_fromISR()
{
	if (h7i2c_i2c_mutex_rtos != NULL)
	{
		//signed BaseType_t xHigherPriorityTaskWoken;
		signed long xHigherPriorityTaskWoken;
		xSemaphoreGiveFromISR(h7i2c_i2c_mutex_rtos, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


int h7i2c_i2c_rtos_waitidle(uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

  // Blocking loop, waiting for the I2C peripheral to become free, to get into error state or to timeout
  while (1)
  {
    switch (h7i2c_get_state())
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

int h7i2c_i2c_write_rtos_blocking(uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

	retval = h7i2c_i2c_write(dev_address, data_size, data_buf, timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
	}

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

  return H7I2C_RET_CODE_OK;
}

int h7i2c_i2c_read_rtos_blocking(uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

	retval = h7i2c_i2c_read(dev_address, data_size, data_buf, timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
	}

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

  return H7I2C_RET_CODE_OK;
}

int h7i2c_i2c_write_then_read_rtos_blocking(uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t *wr_buf, uint8_t *rd_buf, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

	retval = h7i2c_i2c_write_then_read(dev_address, wr_size, rd_size, wr_buf, rd_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
	}

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
	}

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_quickcommand_write_rtos_blocking(uint16_t dev_address, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

  h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

	retval = h7i2c_smbus_quickcommand_write(dev_address, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
	}

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
	}

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_quickcommand_read_rtos_blocking(uint16_t dev_address, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
	}

	retval = h7i2c_smbus_quickcommand_read(dev_address, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
	}

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
	}

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_sendbyte_rtos_blocking(uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_sendbyte(dev_address, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_receivebyte_rtos_blocking(uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_receivebyte(dev_address, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_writebyte_rtos_blocking(uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_writebyte(dev_address, command, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_readbyte_rtos_blocking(uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_readbyte(dev_address, command, byte, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_writeword_rtos_blocking(uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_writeword(dev_address, command, word, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_readword_rtos_blocking(uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_readword(dev_address, command, word, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_processcall_rtos_blocking(uint16_t dev_address, uint8_t command, uint16_t* wr_word, uint16_t* rd_word, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_processcall(dev_address, command, wr_word, rd_word, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_blockwrite_rtos_blocking(uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* wr_buf, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_blockwrite(dev_address, command, wr_size, wr_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_blockread_rtos_blocking(uint16_t dev_address, uint8_t command, uint8_t* rd_size, uint8_t* rd_buf, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_blockread(dev_address, command, rd_size, rd_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_blockwritereadprocesscall_rtos_blocking(uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* rd_size, uint16_t *wr_buf, uint16_t *rd_buf, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_blockwritereadprocesscall(dev_address, command, wr_size, rd_size, wr_buf, rd_buf, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_write32_rtos_blocking(uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_write32(dev_address, command, word32, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_read32_rtos_blocking(uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_read32(dev_address, command, word32, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}

int h7i2c_smbus_write64_rtos_blocking(uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_write64(dev_address, command, word64, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}


int h7i2c_smbus_read64_rtos_blocking(uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	h7i2c_i2c_ret_code_t retval;
	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

	retval = h7i2c_smbus_read64(dev_address, command, word64, timeout);
  if (retval != H7I2C_RET_CODE_OK)
	{
  	return retval;
  }

	if (HAL_GetTick() - timestart > timeout)
	{
		return H7I2C_RET_CODE_TIMEOUT;
  }

	timeout = timeout - (HAL_GetTick() - timestart);

	retval = h7i2c_i2c_rtos_waitidle(timeout);
	if (retval != H7I2C_RET_CODE_OK)
	{
		return retval;
  }

  return H7I2C_RET_CODE_OK;
}




