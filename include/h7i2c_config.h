
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

#ifndef INC_H7I2C_CONFIG_H_
#define INC_H7I2C_CONFIG_H_


// Use these defines to put the peripheral under the responsibility of this driver.
// You shall mind about collisions with the STM32Cube driver (if you use this driver,
// the peripheral should be unconfigured in the IOC file).
#define H7I2C_PERIPH_ENABLE_I2C1 0
#define H7I2C_PERIPH_ENABLE_I2C2 0
#define H7I2C_PERIPH_ENABLE_I2C3 0
#define H7I2C_PERIPH_ENABLE_I2C4 0

// Do you want to use the FreeRTOS-compatible function implementations?
#define H7I2C_USE_FREERTOS_IMPL 0


// Do not edit this logic if you don't understand it
#if H7I2C_PERIPH_ENABLE_I2C1 == 1 || H7I2C_PERIPH_ENABLE_I2C2 == 1 || H7I2C_PERIPH_ENABLE_I2C3 == 1 || H7I2C_PERIPH_ENABLE_I2C4 == 1
#define H7I2C_PERIPH_ENABLE_ANY 1
#else
#define H7I2C_PERIPH_ENABLE_ANY 0
#endif


#endif // INC_H7I2C_CONFIG_H_
