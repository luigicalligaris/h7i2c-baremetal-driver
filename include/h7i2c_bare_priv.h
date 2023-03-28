
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

#ifndef INC_H7I2C_BARE_PRIV_H_
#define INC_H7I2C_BARE_PRIV_H_

#include "h7i2c_config.h"
#include "h7i2c_bare.h"

enum {H7I2C_I2C_MUTEX_UNLOCKED = 0, H7I2C_I2C_MUTEX_LOCKED = 1};

typedef struct h7i2c_driver_instance_state_t
{
  h7i2c_i2c_fsm_state_t fsm_state;

  void*    i2c_base;

  uint32_t slave_address;

  uint32_t cr1_value;
  uint32_t cr2_value;

  uint32_t wr_todo;
  uint32_t wr_done;
  uint8_t* wr_data;

  uint32_t rd_todo;
  uint32_t rd_done;
  uint8_t* rd_data;

  uint32_t timestart;
  uint32_t timeout;
} h7i2c_driver_instance_state_t;

h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_lock(h7i2c_periph_t peripheral, uint32_t timeout);
h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_lock_impl(h7i2c_periph_t peripheral);
h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_release(h7i2c_periph_t peripheral);
h7i2c_i2c_ret_code_t h7i2c_i2c_mutex_release_fromISR(h7i2c_periph_t peripheral);

#endif // INC_H7I2C_BARE_PRIV_H_
