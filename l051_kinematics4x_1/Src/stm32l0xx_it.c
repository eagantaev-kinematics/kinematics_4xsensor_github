/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

extern SPI_HandleTypeDef hspi2;
extern int debug_flag;
extern int32_t *out_buffer;

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

void SPI2_IRQHandler(void)
{
	int i;

	uint8_t spi2_in_data;
	int16_t aux16;
	uint8_t aux8;

	if((SPI2->SR & SPI_SR_RXNE) != 0)
	{
		// disable spi2 rxne interrupt
		SPI2->CR2 &= ~SPI_CR2_RXNEIE;   // disable rxne interrupt
		// stop systick timer
		SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;

		// read from spi data register
		spi2_in_data = SPI2->DR;
		// proveryaem est' li zapros
		if(spi2_in_data == 0x21)
		{

			// otdaem 9 tekuschih znacheniy iz massiva
			for(i=0;i<9;i++)
			{
				if(debug_flag)
						debug_flag = 0;
				aux16 = (int16_t)(out_buffer[i]);
				//***** MSB *****
				aux8 = aux16 >> 8;
				//wait for txe
				while(!((SPI2->SR & SPI_SR_TXE) == SPI_SR_TXE));
				// write data to spi2
				SPI2->DR = aux8;
				// wait for rxne
				while(!((SPI2->SR & SPI_SR_RXNE) == SPI_SR_RXNE));
				// fictious data read
				spi2_in_data = SPI2->DR;
				//***** LSB *****
				aux8 = (uint8_t)aux16;
				//wait for txe
				while(!((SPI2->SR & SPI_SR_TXE) == SPI_SR_TXE));
				// write data to spi2
				SPI2->DR = aux8;
				// wait for rxne
				while(!((SPI2->SR & SPI_SR_RXNE) == SPI_SR_RXNE));
				// fictious data read
				spi2_in_data = SPI2->DR;
			}

		}

		// enable spi2 rxne interrupt
		SPI2->CR2 |= SPI_CR2_RXNEIE;   // enable rxne interrupt
		// start systick timer
		SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
