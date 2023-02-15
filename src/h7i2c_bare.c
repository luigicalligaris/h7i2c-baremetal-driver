
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

#include "h7i2c_bare.h"
#include <string.h>

#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"

h7i2c_i2c_fsm_state_t  h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_UNINITIALIZED;
uint32_t             h7i2c_i2c_slave_address;

uint32_t             h7i2c_i2c_cr1_value;
uint32_t             h7i2c_i2c_cr2_value;

uint32_t             h7i2c_i2c_wr_todo;
uint32_t             h7i2c_i2c_wr_done;
uint8_t*             h7i2c_i2c_wr_data;

uint32_t             h7i2c_i2c_rd_todo;
uint32_t             h7i2c_i2c_rd_done;
uint8_t*             h7i2c_i2c_rd_data;

uint32_t             h7i2c_i2c_timestart;
uint32_t             h7i2c_i2c_timeout;


#define H7I2C_I2C_MUTEX_UNLOCKED 0
#define H7I2C_I2C_MUTEX_LOCKED   1
uint8_t h7i2c_i2c_mutex = H7I2C_I2C_MUTEX_UNLOCKED;

__weak int h7i2c_i2c_mutex_lock(uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	if (H7I2C_I2C_MUTEX_UNLOCKED == __LDREXB(&h7i2c_i2c_mutex))
	{
		if (0 == __STREXB(H7I2C_I2C_MUTEX_LOCKED, &h7i2c_i2c_mutex))
		{
			__DMB();// Data Memory Barrier
			return H7I2C_RET_CODE_OK;
		}
		else
		{
			return H7I2C_RET_CODE_BUSY;
		}
	}
	else
	{
		while (HAL_GetTick() - timestart < timeout)
		{
			if (H7I2C_I2C_MUTEX_UNLOCKED == __LDREXB(&h7i2c_i2c_mutex))
			{
				if (0 == __STREXB(H7I2C_I2C_MUTEX_LOCKED, &h7i2c_i2c_mutex))
				{
					__DMB(); // Data Memory Barrier
					return H7I2C_RET_CODE_OK;
				}
			}
	  }
	}
	return H7I2C_RET_CODE_BUSY;
}

__weak void h7i2c_i2c_mutex_release()
{
	if (H7I2C_I2C_MUTEX_LOCKED == __LDREXB(&h7i2c_i2c_mutex))
	{
		if (0 == __STREXB(H7I2C_I2C_MUTEX_UNLOCKED, &h7i2c_i2c_mutex))
		{
			__DMB();// Data Memory Barrier
			return;
		}
	}
}

__weak void h7i2c_i2c_mutex_release_fromISR()
{
	h7i2c_i2c_mutex_release();
}


static void h7i2c_i2c_reset_peripheral_full()
{
  // Clear the Control Register 1
  MODIFY_REG(I2C4->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE
    | I2C_CR1_NACKIE    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE
    | I2C_CR1_ANFOFF    | I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC
    | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN     | I2C_CR1_GCEN      | I2C_CR1_SMBHEN
    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN     | I2C_CR1_DNF,
    0x00000000);

  // Clear the Control Register 2
  MODIFY_REG(I2C4->CR2,
      I2C_CR2_RD_WRN   | I2C_CR2_ADD10    | I2C_CR2_HEAD10R  | I2C_CR2_RELOAD
    | I2C_CR2_AUTOEND  | I2C_CR2_SADD     | I2C_CR2_NBYTES   , 0x00000000);

  // Clear the Own Address 1
  MODIFY_REG(I2C4->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE | I2C_OAR1_OA1EN, 0x00000000);

  // Clear the Own Address 2
  MODIFY_REG(I2C4->OAR2, I2C_OAR2_OA2 | I2C_OAR2_OA2MSK  | I2C_OAR2_OA2EN, 0x00000000);

  // Clear the I2C timing register
  MODIFY_REG(I2C4->TIMINGR, I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL, 0x00000000);

  // Clear the I2C timeout register
  MODIFY_REG(I2C4->TIMEOUTR,
    I2C_TIMEOUTR_TEXTEN | I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIMEOUTB,
		0x00000000);

  // Clear the interrupts
  SET_BIT(I2C4->ICR, I2C_ICR_ADDRCF  | I2C_ICR_NACKCF  | I2C_ICR_STOPCF  | I2C_ICR_BERRCF
    | I2C_ICR_ARLOCF  | I2C_ICR_OVRCF   | I2C_ICR_PECCF   | I2C_ICR_TIMOUTCF| I2C_ICR_ALERTCF);
}

static void h7i2c_i2c_reset_peripheral_soft()
{
	CLEAR_BIT(I2C4->CR1, I2C_CR1_PE);
	((void) READ_BIT(I2C4->CR1, I2C_CR1_PE)); // the cast to void is to suppress the "value computed is not used [-Wunused-value]" warning
	SET_BIT(I2C4->CR1, I2C_CR1_PE);
}

static void h7i2c_i2c_reset_driver()
{
	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_IDLE;
	h7i2c_i2c_mutex_release();
}

void h7i2c_i2c_init()
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // When the M7 stops we want the I2C4 timeout to stop as well
  MODIFY_REG(DBGMCU->APB4FZ1, DBGMCU_APB4FZ1_DBG_I2C4, DBGMCU_APB4FZ1_DBG_I2C4);

  __HAL_RCC_GPIOF_CLK_ENABLE();

  // AFRH: Set alternate function I2C4 = 4 = 0b0100 (see datasheet tab 14) to pins 14 and 15
  MODIFY_REG(GPIOF->AFR[1], 0b1111 << 24 | 0b1111 << 28, 0b0100 << 24 | 0b0100 << 28);

  // OSPEEDR: Set very high speed = 0b11 to pins 14 and 15
  MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 28 | 0b11 << 30, 0b11 << 28 | 0b11 << 30);

  // PUPDR: Set pull-up = 0b01 to pins 14 and 15
  MODIFY_REG(GPIOF->PUPDR, 0b11 << 28 | 0b11 << 30, 0b01 << 28 | 0b01 << 30);

  // OTYPEDR: Set open drain = 0b1 to pins 14 and 15
  MODIFY_REG(GPIOF->OTYPER, 0b1 << 14 | 0b1 << 15, 0b1 << 14 | 0b1 << 15);

  // MODER: Set alternate mode = 0b10 to pins 14 and 15
  MODIFY_REG(GPIOF->MODER, 0b11 << 28 | 0b11 << 30, 0b10 << 28 | 0b10 << 30);

  __HAL_RCC_I2C4_CLK_ENABLE();

  // The timing register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
  // given the chosen frequency and delay settings.
  MODIFY_REG(I2C4->TIMINGR, I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC,
    0x90204DFD);

  // The timeout register value is copypasted from the IOC editor or STM32CubeMX , which both have a calculator providing a register value
  // given the chosen frequency and delay settings.
  MODIFY_REG(I2C4->TIMEOUTR, I2C_TIMEOUTR_TIMEOUTA | I2C_TIMEOUTR_TIDLE | I2C_TIMEOUTR_TIMOUTEN | I2C_TIMEOUTR_TIMEOUTB | I2C_TIMEOUTR_TEXTEN,
    0x000084C4);

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

  HAL_NVIC_SetPriority(I2C4_EV_IRQn, 6, 1);
  HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);
  HAL_NVIC_SetPriority(I2C4_ER_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);

  h7i2c_i2c_mutex_release();
}


void h7i2c_deinit()
{
	h7i2c_i2c_reset_peripheral_full();

  // Disable the I2C4 interrupts in NVIC
  HAL_NVIC_DisableIRQ(I2C4_EV_IRQn);
  HAL_NVIC_DisableIRQ(I2C4_ER_IRQn);

  // Disable the peripheral
  CLEAR_BIT(I2C4->CR1, I2C_CR1_PE);

  // Disable the peripheral clock
  __HAL_RCC_I2C4_CLK_DISABLE();

  // Deinit the GPIOs used by I2C4
  // PF14 ---> I2C4_SCL
  // PF15 ---> I2C4_SDA
  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_14);
  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_15);
}

int h7i2c_get_state()
{
	return h7i2c_i2c_fsm_state;
}

int h7i2c_clear_error_state()
{
	uint32_t const timeout = 300;
	if (h7i2c_i2c_mutex_lock(timeout) == H7I2C_RET_CODE_OK)
	{
		switch (h7i2c_i2c_fsm_state)
			{
	  		case H7I2C_FSM_STATE_ERROR_NACKF:
	  		case H7I2C_FSM_STATE_ERROR_BERR:
	  		case H7I2C_FSM_STATE_ERROR_ARLO:
	  		case H7I2C_FSM_STATE_ERROR_OVR:
	  		case H7I2C_FSM_STATE_ERROR_PECERR:
	  		case H7I2C_FSM_STATE_ERROR_TIMEOUT:
	  			h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_IDLE;
	  			h7i2c_i2c_mutex_release();
	  			return H7I2C_RET_CODE_OK;
	  		default:
	  			h7i2c_i2c_mutex_release();
	  			break;
			}
	}
	return H7I2C_RET_CODE_ERROR;
}

void I2C4_EV_IRQHandler(void)
{
  uint32_t const isr = I2C4->ISR;
  h7i2c_i2c_cr1_value  = I2C4->CR1;
  h7i2c_i2c_cr2_value  = I2C4->CR2;

  // Reminder: RXNE is cleared by reading the I2C_RXDR register
  if ( READ_BIT(isr, I2C_ISR_RXNE) != 0 )
  {
  	h7i2c_i2c_rd_data[h7i2c_i2c_rd_done] = I2C4->RXDR;
  	--h7i2c_i2c_rd_todo;
  	++h7i2c_i2c_rd_done;
    if (h7i2c_i2c_rd_todo == 0U)
   	 CLEAR_BIT(h7i2c_i2c_cr1_value, I2C_CR1_RXIE);
  }

  // Reminder: TXIS/TXE is cleared by writing the I2C_TXDR register
  if (READ_BIT(isr, I2C_ISR_TXIS) != 0)
  {
  	if (READ_BIT(isr, I2C_ISR_TXE) != 0)
  	{
      I2C4->TXDR = h7i2c_i2c_wr_data[h7i2c_i2c_wr_done];
      --h7i2c_i2c_wr_todo;
      ++h7i2c_i2c_wr_done;
      if (h7i2c_i2c_wr_todo == 0U)
    	  CLEAR_BIT(h7i2c_i2c_cr1_value, I2C_CR1_TXIE);
  	}
  }

  // Reminder: TC is cleared by writing START = 1 or STOP = 1
  if ( READ_BIT(isr, I2C_ISR_TC) != 0 )
  {
  	uint32_t const h7i2c_i2c_fsm_state_copy = h7i2c_i2c_fsm_state;
  	switch(h7i2c_i2c_fsm_state_copy)
  	{
  		case H7I2C_FSM_STATE_WRITEREAD_WRITESTEP:
  			CLEAR_BIT(h7i2c_i2c_cr1_value, I2C_CR1_TXIE  );
  			SET_BIT  (h7i2c_i2c_cr1_value, I2C_CR1_RXIE  );
  			SET_BIT  (h7i2c_i2c_cr2_value, I2C_CR2_RD_WRN | I2C_CR2_START);
  			SET_BIT  (h7i2c_i2c_cr2_value, I2C_CR2_AUTOEND);
  			h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_WRITEREAD_READSTEP;
  			if (h7i2c_i2c_rd_todo > 255ul)
  			{
  			  MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, (255UL << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
  			  SET_BIT   (h7i2c_i2c_cr2_value, I2C_CR2_RELOAD);
  			}
  			else
  			{
  			  MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, (h7i2c_i2c_rd_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
  			  CLEAR_BIT (h7i2c_i2c_cr2_value, I2C_CR2_RELOAD);
  			}
  			break;

  		case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP:
  			CLEAR_BIT(h7i2c_i2c_cr1_value, I2C_CR1_TXIE  );
  			SET_BIT  (h7i2c_i2c_cr1_value, I2C_CR1_RXIE  );
  			SET_BIT  (h7i2c_i2c_cr2_value, I2C_CR2_RD_WRN | I2C_CR2_START);
  			// We just read one byte, which encodes the number of RX bytes which will follow
  			MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, (1U << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
  			CLEAR_BIT (h7i2c_i2c_cr2_value, I2C_CR2_RELOAD);
  			h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXNBYTES;
  			break;

  		case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXNBYTES:
  			SET_BIT  (h7i2c_i2c_cr2_value, I2C_CR2_START);
  			h7i2c_i2c_rd_todo = h7i2c_i2c_rd_data[0];
				h7i2c_i2c_rd_done = 1U;
  			MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, (h7i2c_i2c_rd_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);
  			CLEAR_BIT (h7i2c_i2c_cr2_value, I2C_CR2_RELOAD);
  			h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXDATA;
  			break;

  		case H7I2C_FSM_STATE_WRITEONLY:
  			CLEAR_BIT(h7i2c_i2c_cr1_value, I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE);
  			SET_BIT  (h7i2c_i2c_cr2_value, I2C_CR2_STOP);
  			break;

  		case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXDATA:
  		case H7I2C_FSM_STATE_WRITEREAD_READSTEP:
  		case H7I2C_FSM_STATE_READONLY:
  			CLEAR_BIT(h7i2c_i2c_cr1_value, I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE);
  			SET_BIT  (h7i2c_i2c_cr2_value, I2C_CR2_STOP);
  			break;

  		default:
  			break;
  	}
  }

  // Reminder: TCR cleared by writing I2C_CR2 with NBYTES[7:0] != 0
  if ( READ_BIT(isr, I2C_ISR_TCR) != 0 )
  {
  	switch(h7i2c_i2c_fsm_state)
  	{
  		case H7I2C_FSM_STATE_WRITEREAD_WRITESTEP:
  		case H7I2C_FSM_STATE_WRITEONLY:
  		case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP:
  			if (h7i2c_i2c_wr_todo > 255UL)
  			{
  			  MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
						( (255UL << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | I2C_CR2_RELOAD);
  			}
  			else
  			{
  			  MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
						( (h7i2c_i2c_wr_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | 0U);
  			}
  			break;

  		case H7I2C_FSM_STATE_WRITEREAD_READSTEP:
  		case H7I2C_FSM_STATE_READONLY:
  			if (h7i2c_i2c_rd_todo > 255UL)
  			{
  			  MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
            ( (255UL << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | I2C_CR2_RELOAD);
  			}
  			else
  			{
  			  MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES | I2C_CR2_RELOAD,
					  ( (h7i2c_i2c_rd_todo << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | 0U);
  			}
  			break;

  		default:
  			break;
  	}
  }

  uint32_t icr = 0u;

  // Reminder: STOPF cleared by writing STOPCF = 1
  if ( READ_BIT(isr, I2C_ISR_STOPF) != 0 )
  {
    SET_BIT(icr, I2C_ICR_STOPCF);
    CLEAR_BIT(h7i2c_i2c_cr1_value , I2C_CR1_PE);
    h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_IDLE;
    h7i2c_i2c_mutex_release_fromISR();
  }

  // Reminder: ADDR is cleared by writing ADDRCF = 1
  if ( READ_BIT(isr, I2C_ISR_ADDR) != 0 )
  {
  	// In principle we should not get here, as this is disabled
    SET_BIT(icr, I2C_ICR_ADDRCF);
  }

  // Reminder: NACKF is cleared by writing NACKCF = 1
  if ( READ_BIT(isr, I2C_ISR_NACKF) != 0 )
  {
    SET_BIT(icr, I2C_ICR_NACKCF);
    h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_ERROR_NACKF;
  }

  MODIFY_REG(I2C4->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE    | I2C_CR1_NACKIE
    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE     | I2C_CR1_DNF       | I2C_CR1_ANFOFF
		| I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC       | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN
		| I2C_CR1_GCEN      | I2C_CR1_SMBHEN    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN,
		h7i2c_i2c_cr1_value
  );

  MODIFY_REG(I2C4->CR2,
		  I2C_CR2_SADD      | I2C_CR2_RD_WRN    | I2C_CR2_ADD10      | I2C_CR2_HEAD10R      | I2C_CR2_START
    | I2C_CR2_STOP      | I2C_CR2_NACK      | I2C_CR2_NBYTES     | I2C_CR2_RELOAD       | I2C_CR2_AUTOEND
		| I2C_CR2_PECBYTE,
		h7i2c_i2c_cr2_value
  );

  I2C4->ICR = icr;
}


void I2C4_ER_IRQHandler(void)
{
  uint32_t const isr = I2C4->ISR;
  uint32_t icr       = 0u;

  // BERR is cleared by writing BERRCF = 1
  if ( READ_BIT(isr, I2C_ISR_BERR) != 0 )
  {
    icr |= I2C_ICR_BERRCF;
    h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_ERROR_BERR;
  }

  // ARLO is cleared by writing ARLOCF = 1
  if ( READ_BIT(isr, I2C_ISR_ARLO) != 0 )
  {
    icr |= I2C_ICR_ARLOCF;
    h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_ERROR_ARLO;
  }

  // OVR is cleared by writing OVRCF = 1
  if ( READ_BIT(isr, I2C_ISR_OVR) != 0 )
  {
    icr |= I2C_ICR_OVRCF;
    h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_ERROR_OVR;
  }

  // PECERR is cleared by writing PECCF = 1
  if ( READ_BIT(isr, I2C_ISR_PECERR) != 0 )
  {
    icr |= I2C_ICR_PECCF;
    h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_ERROR_PECERR;
  }

  // TIMEOUT is cleared by writing TIMEOUTCF = 1
  if ( READ_BIT(isr, I2C_ISR_TIMEOUT) != 0 )
  {
    icr |= I2C_ICR_TIMOUTCF;
    h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_ERROR_TIMEOUT;
  }

  // ALERT is cleared by writing ALERTCF = 1
  if ( READ_BIT(isr, I2C_ISR_ALERT) != 0 )
  {
    icr |= I2C_ICR_ALERTCF;
  }

  I2C4->ICR = icr;
}


static int h7i2c_i2c_pre_transaction_check(uint32_t timeout)
{
	uint32_t const timestart = HAL_GetTick();

	// Do not run if the driver is in error state
  switch (h7i2c_i2c_fsm_state)
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
  if (h7i2c_i2c_fsm_state == H7I2C_FSM_STATE_UNINITIALIZED)
  {
 	  if (h7i2c_i2c_mutex_lock(timeout) != HAL_OK)
    {
  	  return H7I2C_RET_CODE_BUSY;
  	}
 	  {
 	  	h7i2c_i2c_init();
 	  	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_IDLE;
 	  	h7i2c_i2c_mutex_release();
 	  }
  }

  // Wait until timeout or until driver becomes ready
  while (1)
  {
  	if (HAL_GetTick() - timestart >= timeout)
  	{
  		return H7I2C_RET_CODE_BUSY;
  	}

  	if (h7i2c_i2c_fsm_state == H7I2C_FSM_STATE_IDLE)
  	{
  	  if (h7i2c_i2c_mutex_lock(timeout) != HAL_OK)
  	  {
  	  	return H7I2C_RET_CODE_BUSY;
  	  }
  	  else
  	  {
  	  	// If the state changed while we were taking the mutex, keep trying
  	  	if (h7i2c_i2c_fsm_state != H7I2C_FSM_STATE_IDLE)
  	  	{
  	  		h7i2c_i2c_mutex_release();
  	  		continue;
  	  	}

  	  	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_SETUP_TRANSFER;
  	  	h7i2c_i2c_mutex_release();
  	  	break;
  	  }
  	}
  }

  return H7I2C_RET_CODE_OK;
}

static int h7i2c_i2c_write_read_transaction(uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t* wr_buf, uint8_t* rd_buf, uint32_t timeout)
{
	int const check_ret_val = h7i2c_i2c_pre_transaction_check(timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  	return check_ret_val;

  h7i2c_i2c_timestart      = HAL_GetTick();
  h7i2c_i2c_timeout        = timeout;
  h7i2c_i2c_slave_address  = dev_address;

  h7i2c_i2c_wr_todo        = wr_size;
  h7i2c_i2c_wr_done        = 0;
  h7i2c_i2c_wr_data        = wr_buf;

  h7i2c_i2c_rd_todo        = rd_size;
  h7i2c_i2c_rd_done        = 0;
  h7i2c_i2c_rd_data        = rd_buf;

	if (h7i2c_i2c_wr_todo > 0UL && h7i2c_i2c_wr_data != NULL)
  {
  	// We will do a write first
    if (h7i2c_i2c_rd_todo > 0UL && h7i2c_i2c_rd_data != NULL)
  		// We will do a write, then a read
    	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_WRITEREAD_WRITESTEP;
    else
    	// We will do only the write
    	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_WRITEONLY;
  }
  else
  {
    // We will not do a write
    if (h7i2c_i2c_rd_todo > 0UL && h7i2c_i2c_rd_data != NULL)
    	// We will only do a read
     	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_READONLY;
    else
    {
     	// We will do nothing
     	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_IDLE;
     	return H7I2C_RET_CODE_INVALID_ARGS;
    }
  }

  // Default value for CR1
  h7i2c_i2c_cr1_value =
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
  h7i2c_i2c_cr2_value =
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

	MODIFY_REG(h7i2c_i2c_cr2_value,  I2C_CR2_SADD, (h7i2c_i2c_slave_address << I2C_CR2_SADD_Pos   ));


  switch (h7i2c_i2c_fsm_state)
  {
  	case H7I2C_FSM_STATE_WRITEREAD_WRITESTEP:
  		SET_BIT(h7i2c_i2c_cr1_value,  I2C_CR1_TXIE);
  		CLEAR_BIT(h7i2c_i2c_cr2_value, I2C_CR2_RD_WRN );

      if (h7i2c_i2c_wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, h7i2c_i2c_wr_todo << I2C_CR2_NBYTES_Pos);
      }
  		break;

  	case H7I2C_FSM_STATE_WRITEONLY:
  		SET_BIT(h7i2c_i2c_cr1_value, I2C_CR1_TXIE   );
  		CLEAR_BIT(h7i2c_i2c_cr2_value, I2C_CR2_RD_WRN );
  		//SET_BIT(h7i2c_i2c_cr2_value, I2C_CR2_AUTOEND);

      if (h7i2c_i2c_wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, h7i2c_i2c_wr_todo << I2C_CR2_NBYTES_Pos);
      }
  		break;

  	case H7I2C_FSM_STATE_READONLY:
  		SET_BIT(h7i2c_i2c_cr1_value, I2C_CR1_RXIE   );
  		SET_BIT(h7i2c_i2c_cr2_value, I2C_CR2_RD_WRN );
  		//SET_BIT(h7i2c_i2c_cr2_value, I2C_CR2_AUTOEND);

      if (h7i2c_i2c_wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, h7i2c_i2c_rd_todo << I2C_CR2_NBYTES_Pos);
      }
  		break;

  	// We should not get to the ones below
  	case H7I2C_FSM_STATE_WRITEREAD_READSTEP:
  	case H7I2C_FSM_STATE_IDLE:
  	default:
  		break;
  }

#if 0
  MODIFY_REG(I2C4->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE    | I2C_CR1_NACKIE
    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE     | I2C_CR1_DNF       | I2C_CR1_ANFOFF
		| I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC       | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN
		| I2C_CR1_GCEN      | I2C_CR1_SMBHEN    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN,
		h7i2c_i2c_cr1_value
  );

  MODIFY_REG(I2C4->CR2,
		  I2C_CR2_SADD      | I2C_CR2_RD_WRN    | I2C_CR2_ADD10      | I2C_CR2_HEAD10R      | I2C_CR2_START
    | I2C_CR2_STOP      | I2C_CR2_NACK      | I2C_CR2_NBYTES     | I2C_CR2_RELOAD       | I2C_CR2_AUTOEND
		| I2C_CR2_PECBYTE,
		h7i2c_i2c_cr2_value
  );
#else
  I2C4->CR1 = h7i2c_i2c_cr1_value;
  READ_REG(I2C4->CR1);
  I2C4->CR2 = h7i2c_i2c_cr2_value;
  READ_REG(I2C4->CR2);
#endif

  SET_BIT(I2C4->CR2, I2C_CR2_START );

  return H7I2C_RET_CODE_OK;
}

static int h7i2c_i2c_empty_write_transaction(uint16_t dev_address, uint32_t timeout)
{
	int const check_ret_val = h7i2c_i2c_pre_transaction_check(timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  	return check_ret_val;

  h7i2c_i2c_timestart      = HAL_GetTick();
  h7i2c_i2c_timeout        = timeout;
  h7i2c_i2c_slave_address  = dev_address;

  h7i2c_i2c_wr_todo        = 0;
  h7i2c_i2c_wr_done        = 0;
  h7i2c_i2c_wr_data        = NULL;

  h7i2c_i2c_rd_todo        = 0;
  h7i2c_i2c_rd_done        = 0;
  h7i2c_i2c_rd_data        = NULL;

  h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_EMPTY_WRITE;

  // Default value for CR1
  h7i2c_i2c_cr1_value =
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
  h7i2c_i2c_cr2_value =
    ( (0UL << I2C_CR2_SADD_Pos   ) & I2C_CR2_SADD   ) | // SADD    = slave address
    ( (0UL << I2C_CR2_RD_WRN_Pos ) & I2C_CR2_RD_WRN ) | // RD_WRN  = 0 (write) or 1 (read)
    ( (0UL << I2C_CR2_ADD10_Pos  ) & I2C_CR2_ADD10  ) | // ADD10   = 0 (7-bit addr) or 1 (10-bit addr)
    ( (0UL << I2C_CR2_HEAD10R_Pos) & I2C_CR2_HEAD10R) | // HEAD10R = 0 (send full 10-bit addr + r/w bit in 10-bit mode) or 1 (send 7-bit addr + r/w bit in 10-bit mode)
    ( (1UL << I2C_CR2_START_Pos  ) & I2C_CR2_START  ) | // START   = generate start bit and header, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_STOP_Pos   ) & I2C_CR2_STOP   ) | // STOP    = generate stop bit after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NACK_Pos   ) & I2C_CR2_NACK   ) | // NACK    = send a NACK after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES ) | // NBYTES  = number of bytes to transfer (8 bits, if more than 255 you need to reload)
    ( (0UL << I2C_CR2_RELOAD_Pos ) & I2C_CR2_RELOAD ) | // RELOAD  = reload mode, trigger TCR when NBYTES gets to zero *and stretch SCL*
    ( (1UL << I2C_CR2_AUTOEND_Pos) & I2C_CR2_AUTOEND) | // AUTOEND = generate a stop bit after the last byte (when NBYTES = 0) is transferred
    ( (0UL << I2C_CR2_PECBYTE_Pos) & I2C_CR2_PECBYTE) ; // PECBYTE = send the PEC byte after the last data byte  (this is relevant for SMBUS)

	MODIFY_REG(h7i2c_i2c_cr2_value,  I2C_CR2_SADD, (h7i2c_i2c_slave_address << I2C_CR2_SADD_Pos   ));

  MODIFY_REG(I2C4->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE    | I2C_CR1_NACKIE
    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE     | I2C_CR1_DNF       | I2C_CR1_ANFOFF
		| I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC       | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN
		| I2C_CR1_GCEN      | I2C_CR1_SMBHEN    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN,
		h7i2c_i2c_cr1_value
  );

  MODIFY_REG(I2C4->CR2,
		  I2C_CR2_SADD      | I2C_CR2_RD_WRN    | I2C_CR2_ADD10      | I2C_CR2_HEAD10R      | I2C_CR2_START
    | I2C_CR2_STOP      | I2C_CR2_NACK      | I2C_CR2_NBYTES     | I2C_CR2_RELOAD       | I2C_CR2_AUTOEND
		| I2C_CR2_PECBYTE,
		h7i2c_i2c_cr1_value
  );

  return H7I2C_RET_CODE_OK;
}

static int h7i2c_i2c_empty_read_transaction(uint16_t dev_address, uint32_t timeout)
{
	int const check_ret_val = h7i2c_i2c_pre_transaction_check(timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  	return check_ret_val;

  h7i2c_i2c_timestart      = HAL_GetTick();
  h7i2c_i2c_timeout        = timeout;
  h7i2c_i2c_slave_address  = dev_address;

  h7i2c_i2c_wr_todo        = 0;
  h7i2c_i2c_wr_done        = 0;
  h7i2c_i2c_wr_data        = NULL;

  h7i2c_i2c_rd_todo        = 0;
  h7i2c_i2c_rd_done        = 0;
  h7i2c_i2c_rd_data        = NULL;

  h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_EMPTY_READ;

  // Default value for CR1
  h7i2c_i2c_cr1_value =
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
  h7i2c_i2c_cr2_value =
    ( (0UL << I2C_CR2_SADD_Pos   ) & I2C_CR2_SADD   ) | // SADD    = slave address
    ( (1UL << I2C_CR2_RD_WRN_Pos ) & I2C_CR2_RD_WRN ) | // RD_WRN  = 0 (write) or 1 (read)
    ( (0UL << I2C_CR2_ADD10_Pos  ) & I2C_CR2_ADD10  ) | // ADD10   = 0 (7-bit addr) or 1 (10-bit addr)
    ( (0UL << I2C_CR2_HEAD10R_Pos) & I2C_CR2_HEAD10R) | // HEAD10R = 0 (send full 10-bit addr + r/w bit in 10-bit mode) or 1 (send 7-bit addr + r/w bit in 10-bit mode)
    ( (1UL << I2C_CR2_START_Pos  ) & I2C_CR2_START  ) | // START   = generate start bit and header, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_STOP_Pos   ) & I2C_CR2_STOP   ) | // STOP    = generate stop bit after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NACK_Pos   ) & I2C_CR2_NACK   ) | // NACK    = send a NACK after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES ) | // NBYTES  = number of bytes to transfer (8 bits, if more than 255 you need to reload)
    ( (0UL << I2C_CR2_RELOAD_Pos ) & I2C_CR2_RELOAD ) | // RELOAD  = reload mode, trigger TCR when NBYTES gets to zero *and stretch SCL*
    ( (1UL << I2C_CR2_AUTOEND_Pos) & I2C_CR2_AUTOEND) | // AUTOEND = generate a stop bit after the last byte (when NBYTES = 0) is transferred
    ( (0UL << I2C_CR2_PECBYTE_Pos) & I2C_CR2_PECBYTE) ; // PECBYTE = send the PEC byte after the last data byte  (this is relevant for SMBUS)

	MODIFY_REG(h7i2c_i2c_cr2_value,  I2C_CR2_SADD, (h7i2c_i2c_slave_address << I2C_CR2_SADD_Pos   ));

  MODIFY_REG(I2C4->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE    | I2C_CR1_NACKIE
    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE     | I2C_CR1_DNF       | I2C_CR1_ANFOFF
		| I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC       | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN
		| I2C_CR1_GCEN      | I2C_CR1_SMBHEN    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN,
		h7i2c_i2c_cr1_value
  );

  MODIFY_REG(I2C4->CR2,
		  I2C_CR2_SADD      | I2C_CR2_RD_WRN    | I2C_CR2_ADD10      | I2C_CR2_HEAD10R      | I2C_CR2_START
    | I2C_CR2_STOP      | I2C_CR2_NACK      | I2C_CR2_NBYTES     | I2C_CR2_RELOAD       | I2C_CR2_AUTOEND
		| I2C_CR2_PECBYTE,
		h7i2c_i2c_cr1_value
  );

  return H7I2C_RET_CODE_OK;
}



static int h7i2c_smbus_write_slave_driven_read_transaction(uint16_t dev_address, uint16_t wr_size, uint8_t* wr_buf, uint8_t* rd_buf, uint32_t timeout)
{
	int const check_ret_val = h7i2c_i2c_pre_transaction_check(timeout);

  if (check_ret_val != H7I2C_RET_CODE_OK)
  	return check_ret_val;

  h7i2c_i2c_timestart      = HAL_GetTick();
  h7i2c_i2c_timeout        = timeout;
  h7i2c_i2c_slave_address  = dev_address;

  h7i2c_i2c_wr_todo        = wr_size;
  h7i2c_i2c_wr_done        = 0;
  h7i2c_i2c_wr_data        = wr_buf;

  h7i2c_i2c_rd_todo        = 1;
  h7i2c_i2c_rd_done        = 0;
  h7i2c_i2c_rd_data        = rd_buf;

  if (h7i2c_i2c_rd_data != NULL)
  {
  	// We will do a write first
  	if (h7i2c_i2c_wr_todo > 0UL && h7i2c_i2c_wr_data != NULL)
  		// We will do a write, then a read
    	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP;
    else
    	// We will do only the read
    	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_READSTEP_RXNBYTES;
  }
  else
  {
   	// We will do nothing
   	h7i2c_i2c_fsm_state = H7I2C_FSM_STATE_IDLE;
   	return H7I2C_RET_CODE_INVALID_ARGS;
  }


  // Default value for CR1
  h7i2c_i2c_cr1_value =
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
  h7i2c_i2c_cr2_value =
    ( (0UL << I2C_CR2_SADD_Pos   ) & I2C_CR2_SADD   ) | // SADD    = slave address
    ( (0UL << I2C_CR2_RD_WRN_Pos ) & I2C_CR2_RD_WRN ) | // RD_WRN  = 0 (write) or 1 (read)
    ( (0UL << I2C_CR2_ADD10_Pos  ) & I2C_CR2_ADD10  ) | // ADD10   = 0 (7-bit addr) or 1 (10-bit addr)
    ( (0UL << I2C_CR2_HEAD10R_Pos) & I2C_CR2_HEAD10R) | // HEAD10R = 0 (send full 10-bit addr + r/w bit in 10-bit mode) or 1 (send 7-bit addr + r/w bit in 10-bit mode)
    ( (1UL << I2C_CR2_START_Pos  ) & I2C_CR2_START  ) | // START   = generate start bit and header, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_STOP_Pos   ) & I2C_CR2_STOP   ) | // STOP    = generate stop bit after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NACK_Pos   ) & I2C_CR2_NACK   ) | // NACK    = send a NACK after current byte, 0 (no) or 1 (yes)
    ( (0UL << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES ) | // NBYTES  = number of bytes to transfer (8 bits, if more than 255 you need to reload)
    ( (0UL << I2C_CR2_RELOAD_Pos ) & I2C_CR2_RELOAD ) | // RELOAD  = reload mode, trigger TCR when NBYTES gets to zero *and stretch SCL*
    ( (0UL << I2C_CR2_AUTOEND_Pos) & I2C_CR2_AUTOEND) | // AUTOEND = generate a stop bit after the last byte (when NBYTES = 0) is transferred
    ( (0UL << I2C_CR2_PECBYTE_Pos) & I2C_CR2_PECBYTE) ; // PECBYTE = send the PEC byte after the last data byte  (this is relevant for SMBUS)

	MODIFY_REG(h7i2c_i2c_cr2_value,  I2C_CR2_SADD, (h7i2c_i2c_slave_address << I2C_CR2_SADD_Pos   ));

  switch (h7i2c_i2c_fsm_state)
  {
  	case H7I2C_FSM_STATE_SLAVE_DRIVEN_READ_WRITESTEP:
  		SET_BIT(h7i2c_i2c_cr1_value,  I2C_CR1_TXIE);

      if (h7i2c_i2c_wr_todo > 255UL)
      {
        // We will need a reload
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_RELOAD | I2C_CR2_NBYTES, I2C_CR2_RELOAD | (255UL << I2C_CR2_NBYTES_Pos) );
      }
      else
      {
        MODIFY_REG(h7i2c_i2c_cr2_value, I2C_CR2_NBYTES, 255UL << I2C_CR2_NBYTES_Pos);
      }
  		break;

  	default:
  		break;
  }

  MODIFY_REG(I2C4->CR1,
      I2C_CR1_PE        | I2C_CR1_TXIE      | I2C_CR1_RXIE      | I2C_CR1_ADDRIE    | I2C_CR1_NACKIE
    | I2C_CR1_STOPIE    | I2C_CR1_TCIE      | I2C_CR1_ERRIE     | I2C_CR1_DNF       | I2C_CR1_ANFOFF
		| I2C_CR1_TXDMAEN   | I2C_CR1_RXDMAEN   | I2C_CR1_SBC       | I2C_CR1_NOSTRETCH | I2C_CR1_WUPEN
		| I2C_CR1_GCEN      | I2C_CR1_SMBHEN    | I2C_CR1_SMBDEN    | I2C_CR1_ALERTEN   | I2C_CR1_PECEN,
		h7i2c_i2c_cr1_value
  );

  MODIFY_REG(I2C4->CR2,
		  I2C_CR2_SADD      | I2C_CR2_RD_WRN    | I2C_CR2_ADD10      | I2C_CR2_HEAD10R      | I2C_CR2_START
    | I2C_CR2_STOP      | I2C_CR2_NACK      | I2C_CR2_NBYTES     | I2C_CR2_RELOAD       | I2C_CR2_AUTOEND
		| I2C_CR2_PECBYTE,
		h7i2c_i2c_cr1_value
  );


  return H7I2C_RET_CODE_OK;
}


int h7i2c_i2c_write(uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
	return h7i2c_i2c_write_read_transaction(dev_address, data_size, 0U, data_buf, NULL, timeout);
}

int h7i2c_i2c_read(uint16_t dev_address, uint16_t data_size, uint8_t *data_buf, uint32_t timeout)
{
	return h7i2c_i2c_write_read_transaction(dev_address, 0U, data_size, NULL, data_buf, timeout);
}

int h7i2c_i2c_write_then_read(uint16_t dev_address, uint16_t wr_size, uint16_t rd_size, uint8_t *wr_buf, uint8_t *rd_buf, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(dev_address, wr_size, rd_size, wr_buf, rd_buf, timeout);
}


int h7i2c_smbus_quickcommand_write(uint16_t dev_address, uint32_t timeout)
{
	return h7i2c_i2c_empty_write_transaction(dev_address, timeout);
}

int h7i2c_smbus_quickcommand_read(uint16_t dev_address, uint32_t timeout)
{
	return h7i2c_i2c_empty_read_transaction(dev_address, timeout);
}


int h7i2c_smbus_sendbyte(uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
	return h7i2c_i2c_write_read_transaction(dev_address, 1U, 0U, byte, NULL, timeout);
}

int h7i2c_smbus_receivebyte(uint16_t dev_address, uint8_t* byte, uint32_t timeout)
{
	return h7i2c_i2c_write_read_transaction(dev_address, 0U, 1U, NULL, byte, timeout);
}


int h7i2c_smbus_writebyte(uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  uint8_t tx_buf[2];
  tx_buf[0] = command;
  tx_buf[1] = byte[0];
	return h7i2c_i2c_write_read_transaction(dev_address, 2U, 0U, tx_buf, NULL, timeout);
}

int h7i2c_smbus_readbyte(uint16_t dev_address, uint8_t command, uint8_t* byte, uint32_t timeout)
{
  return h7i2c_i2c_write_read_transaction(dev_address, 1U, 1U, &command, byte, timeout);
}


int h7i2c_smbus_writeword(uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
  uint8_t wr_buf[3];
  wr_buf[0] = command;
  wr_buf[1] = (word[0] & 0x00FF);
  wr_buf[2] = (word[0] & 0xFF00) >> 8U;
	return h7i2c_i2c_write_read_transaction(dev_address, 3U, 0U, wr_buf, NULL, timeout);
}

int h7i2c_smbus_readword(uint16_t dev_address, uint8_t command, uint16_t* word, uint32_t timeout)
{
  uint8_t rd_buf[2];
  uint32_t ret = h7i2c_i2c_write_read_transaction(dev_address, 1U, 2U, &command, rd_buf, timeout);
  word[0] = ((uint16_t)rd_buf[0]) + ( ((uint16_t)rd_buf[1]) << 8U );
  return ret;
}


int h7i2c_smbus_processcall(uint16_t dev_address, uint8_t command, uint16_t* wr_word, uint16_t* rd_word, uint32_t timeout)
{
  uint8_t tx_buf[3];
  uint8_t rx_buf[2];
  tx_buf[0] = command;
  tx_buf[1] = (wr_word[0] & 0x00FF);
  tx_buf[2] = (wr_word[0] & 0xFF00) >> 8U;
  uint32_t ret = h7i2c_i2c_write_read_transaction(dev_address, 3U, 2U, tx_buf, rx_buf, timeout);
  rd_word[0] = ((uint16_t)rx_buf[0]) + ( ((uint16_t)rx_buf[1]) << 8U );
  return ret;
}


int h7i2c_smbus_blockwrite(uint16_t dev_address, uint8_t command, uint8_t wr_size, uint8_t* wr_buf, uint32_t timeout)
{
  uint8_t tx_buf[257];
  tx_buf[0] = command;
  tx_buf[1] = wr_size;
  memcpy(&(tx_buf[2]), wr_buf, wr_size);
  return h7i2c_i2c_write_read_transaction(dev_address, 2U + wr_size, 0U, tx_buf, NULL, timeout);
}

int h7i2c_smbus_blockread(uint16_t dev_address, uint8_t command, uint8_t* rd_size, uint8_t* rd_buf, uint32_t timeout)
{
  uint8_t rx_buf[256];
  uint32_t ret = h7i2c_smbus_write_slave_driven_read_transaction(dev_address, 1U, &command, rd_buf, timeout);
  *rd_size = rx_buf[0];
  memcpy(&(rx_buf[1]), rd_buf, *rd_size);
  return ret;
}


int h7i2c_smbus_blockwrite_blockread_processcall(uint16_t dev_address, uint8_t command,
  uint8_t wr_size, uint8_t* rd_size, uint16_t *wr_buf, uint16_t *rd_buf, uint32_t timeout)
{
  uint8_t tx_buf[257];
  uint8_t rx_buf[256];
  tx_buf[0] = command;
  tx_buf[1] = wr_size;
  uint32_t ret = h7i2c_smbus_write_slave_driven_read_transaction(dev_address, wr_size, tx_buf, rx_buf, timeout);
  *rd_size = rx_buf[0];
  memcpy(&(rx_buf[1]), rd_buf, *rd_size);
  return ret;
}


int h7i2c_smbus_write32(uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  uint8_t wr_buf[5];
  wr_buf[0] = command;
  wr_buf[1] = (word32[0] & 0x000000FFUL);
  wr_buf[2] = (word32[0] & 0x0000FF00UL) >>  8U;
  wr_buf[3] = (word32[0] & 0x00FF0000UL) >> 16U;
  wr_buf[4] = (word32[0] & 0xFF000000UL) >> 24U;
  return h7i2c_i2c_write_read_transaction(dev_address, 5U, 0U, wr_buf, NULL, timeout);
}

int h7i2c_smbus_read32(uint16_t dev_address, uint8_t command, uint32_t* word32, uint32_t timeout)
{
  uint8_t rd_buf[4];
  uint32_t ret = h7i2c_i2c_write_read_transaction(dev_address, 1U, 4U, &command, rd_buf, timeout);
  word32[0] =
        ( ((uint32_t)rd_buf[0])        )
      + ( ((uint32_t)rd_buf[1]) <<  8U )
      + ( ((uint32_t)rd_buf[2]) << 16U )
      + ( ((uint32_t)rd_buf[3]) << 24U );
  return ret;
}


int h7i2c_smbus_write64(uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
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
  return h7i2c_i2c_write_read_transaction(dev_address, 9U, 0U, wr_buf, NULL, timeout);
}

int h7i2c_smbus_read64(uint16_t dev_address, uint8_t command, uint64_t* word64, uint32_t timeout)
{
  uint8_t rd_buf[8];
  uint32_t ret = h7i2c_i2c_write_read_transaction(dev_address, 1U, 8U, &command, rd_buf, timeout);
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





