
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


#ifndef INC_H7I2C_RTOS_H_
#define INC_H7I2C_RTOS_H_

#include <stdint.h>

#include "h7i2c_config.h"
#include "h7i2c_bare.h"


int h7i2c_i2c_write_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout);
int h7i2c_i2c_read_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout);

int h7i2c_i2c_write_then_read_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t *wr_buf, uint8_t *rd_buf, uint32_t timeout);

int h7i2c_smbus_quickcommand_write_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout);
int h7i2c_smbus_quickcommand_read_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint32_t timeout);

int h7i2c_smbus_sendbyte_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t* byte, uint32_t timeout);
int h7i2c_smbus_receivebyte_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t* byte, uint32_t timeout);

int h7i2c_smbus_writebyte_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout);
int h7i2c_smbus_readbyte_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout);

int h7i2c_smbus_writeword_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout);
int h7i2c_smbus_readword_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout);

int h7i2c_smbus_processcall_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint16_t* wr_word, uint16_t* rd_word, uint32_t timeout);

int h7i2c_smbus_blockwrite_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* wr_buf, uint32_t timeout);
int h7i2c_smbus_blockread_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t* rd_size, uint8_t* rd_buf, uint32_t timeout);

int h7i2c_smbus_blockwritereadprocesscall_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* rd_size, uint16_t *wr_buf, uint16_t *rd_buf, uint32_t timeout);

int h7i2c_smbus_write32_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout);
int h7i2c_smbus_read32_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout);

int h7i2c_smbus_write64_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout);
int h7i2c_smbus_read64_rtos_blocking(h7i2c_periph_t peripheral, uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout);


#endif /* INC_H7I2C_RTOS_H_ */
