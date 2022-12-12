
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
/* * Neither the name of the {organization} nor the names of its                             */
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

#ifndef INC_H7I2C_BARE_H_
#define INC_H7I2C_BARE_H_

#include "stdint.h"

#include "main.h"

typedef enum
{
  H7I2C_FSM_STATE_UNINITIALIZED,
  H7I2C_FSM_STATE_IDLE,
  H7I2C_FSM_STATE_SETUP_TRANSFER,
  H7I2C_FSM_STATE_DEINITIALIZING,
  H7I2C_FSM_STATE_EMPTY_WRITE,
  H7I2C_FSM_STATE_EMPTY_READ,
  H7I2C_FSM_STATE_WRITEREAD_WRITESTEP,
  H7I2C_FSM_STATE_WRITEREAD_READSTEP,
  H7I2C_FSM_STATE_WRITEONLY,
  H7I2C_FSM_STATE_READONLY,
  H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP,
	H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXNBYTES,
	H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXDATA,
  H7I2C_FSM_STATE_ERROR_NACKF,
  H7I2C_FSM_STATE_ERROR_BERR,
  H7I2C_FSM_STATE_ERROR_ARLO,
  H7I2C_FSM_STATE_ERROR_OVR,
  H7I2C_FSM_STATE_ERROR_PECERR,
  H7I2C_FSM_STATE_ERROR_TIMEOUT
} h7i2c_i2c_fsm_state_t;

typedef enum
{
	H7I2C_RET_CODE_OK,
	H7I2C_RET_CODE_BUSY,
	H7I2C_RET_CODE_TIMEOUT,
	H7I2C_RET_CODE_PERIPH_IN_ERR_STATE,
	H7I2C_RET_CODE_INVALID_ARGS,
	H7I2C_RET_CODE_ERROR
} h7i2c_i2c_ret_code_t;



void h7i2c_i2c_init();
void h7i2c_deinit();

int h7i2c_get_state();
int h7i2c_clear_error_state();

int h7i2c_i2c_write(uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout);
int h7i2c_i2c_read(uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout);
int h7i2c_i2c_write_then_read(uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t *wr_buf, uint8_t *rd_buf, uint32_t timeout);

int h7i2c_smbus_quickcommand_write(uint16_t dev_address, uint32_t timeout);
int h7i2c_smbus_quickcommand_read(uint16_t dev_address, uint32_t timeout);

int h7i2c_smbus_sendbyte(uint16_t dev_address, uint8_t* byte, uint32_t timeout);
int h7i2c_smbus_receivebyte(uint16_t dev_address, uint8_t* byte, uint32_t timeout);

int h7i2c_smbus_writebyte(uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout);
int h7i2c_smbus_readbyte(uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout);

int h7i2c_smbus_writeword(uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout);
int h7i2c_smbus_readword(uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout);

int h7i2c_smbus_processcall(uint16_t dev_address, uint8_t command, uint16_t* wr_word, uint16_t* rd_word, uint32_t timeout);

int h7i2c_smbus_blockwrite(uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* wr_buf, uint32_t timeout);
int h7i2c_smbus_blockread(uint16_t dev_address, uint8_t command, uint8_t* rd_size, uint8_t* rd_buf, uint32_t timeout);

int h7i2c_smbus_blockwritereadprocesscall(uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* rd_size, uint16_t *wr_buf, uint16_t *rd_buf, uint32_t timeout);

int h7i2c_smbus_write32(uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout);
int h7i2c_smbus_read32(uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout);

int h7i2c_smbus_write64(uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout);
int h7i2c_smbus_read64(uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout);

#endif // INC_H7I2C_BARE_H_
