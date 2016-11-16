/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
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
// Includes ------------------------------------------------------------------
#include "stm32l0xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"

#include "eeprom_storage_obj.h"

#include "calibr_lab_00.h"  // fail s kalibrovochnymi dannymi
#define SAVE_CALIBRATION


#define NUMBER_OF_SENSORS 4

// Private constants ---------------------------------------------------------
const uint8_t MAGNET_I2C_ADDRESS = 0x0c;

// Private variables ---------------------------------------------------------
int debug_flag = 0;
UART_HandleTypeDef huart1;
uint8_t message[128];

int32_t buffer0[9];
int32_t buffer1[9];
int16_t aux16;
uint8_t aux8;

int32_t *out_buffer = (uint16_t *)buffer0;
int32_t *fill_buffer = (uint16_t *)buffer1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

// Private function prototypes -----------------------------------------------
void SystemClock_Config(void);

void short_delay()
{
    volatile int i;

    // test delay
    for(i=0; i<5; i++)
    {
	    i++;
	    i--;
    }
}

void long_delay()
{
    volatile long i;

    // test delay
    for(i=0; i<500000; i++)
    {
	    i++;
	    i--;
    }
}

void chipselhigh()
{

    int i;

    //short_delay();

    // set chipselects high
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

}


void set_chipselect(uint8_t address)
{
	//

	if(address == 0)
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	else if(address == 1)
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	else if(address == 2)
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	else if(address == 3)
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

}


void write_byte(int address, uint8_t data, int chipselect)
{

	uint16_t data_out;
    uint16_t read_data;

	chipselhigh();

	set_chipselect(chipselect);
	//short_delay();
	// wait for spi transmitter readiness
	while ((SPI1->SR & SPI_SR_TXE) == RESET );
	data_out = (((uint16_t)address)<<8) + (uint16_t)data;
    SPI1->DR = data_out;
    // wait while a transmission complete
	while ((SPI1->SR & SPI_SR_RXNE) == RESET );
    read_data = SPI1->DR;

    chipselhigh();
	//short_delay();
}


uint8_t read_byte(int address, int chipselect)
{

	uint8_t return_data;
    uint8_t data_aux;
	uint16_t data_out;
    uint16_t read_data;

	chipselhigh();

	set_chipselect(chipselect);
	//short_delay();
	// wait for spi transmitter readiness
	while ((SPI1->SR & SPI_SR_TXE) == RESET );
	data_aux = (uint8_t)0x80 | (uint8_t)address;
    data_out = (((uint16_t)data_aux)<<8) + (uint16_t)0x55;
    //debug
    //printf("send %d %d \r\n", (uint8_t)(data_out>>8), (uint8_t)(data_out));

	SPI1->DR = data_out; // send command (read register)

	// wait while a transmission complete
	while ((SPI1->SR & SPI_SR_RXNE) == RESET );
	read_data = SPI1->DR; // read register data
    return_data = (uint8_t)(read_data);

    //debug
    //printf("received %d %d \r\n", (uint8_t)(read_data>>8), (uint8_t)(read_data));

	chipselhigh();
	//short_delay();

	return return_data;
}



int16_t read_word(int high_address, int low_address, int chipselect)
{

	uint8_t high_byte;
	uint8_t low_byte;
	uint16_t buffer;
	int16_t result;

	high_byte = read_byte(high_address, chipselect);
	low_byte = read_byte(low_address, chipselect);

	buffer = ((uint16_t)high_byte)<<8;
	buffer |= (uint16_t)low_byte;
	result = (int16_t)buffer;

	return result;
}


// returns 0 if everything ok, returns 1 if timeout
int write_i2c_byte(uint8_t address, uint8_t value, int chipselect)
{
	uint8_t reg52_value;
	const uint8_t x80 = 0x80;

	int i = 0;
	int doJob;

	write_byte(49, MAGNET_I2C_ADDRESS, chipselect); // komanda zapis' v magnetometr
	write_byte(50, address, chipselect); // adres registra magnetometra
	write_byte(51, value, chipselect); // znachenie, kotoroe nado zapisat' v registr magnetometra
	write_byte(52, 0x80, chipselect); // zapuskaem obmen

	// wait for a transfer completion
	doJob = 1;
	while((i<100) && doJob)
	{
		reg52_value = read_byte(52, chipselect);

		if((reg52_value & x80) == 0) // transfer complete
			doJob = 0;
		else
		{
			i++;
			short_delay();
		}

	}

	if(doJob) // timeout occured
		return 1;
	else	// everything ok
		return 0;

}


uint8_t read_i2c_byte(uint8_t address, int chipselect)
{
	uint8_t reg52_value;
	const uint8_t x80 = 0x80;
	uint8_t magnetometer_data = 0;

	int i = 0;
	int doJob;

	write_byte(49, (uint8_t)0x80 | MAGNET_I2C_ADDRESS, chipselect); // komanda chtenie iz magnetometra
	write_byte(50, address, chipselect); // adres registra magnetometra
	write_byte(52, 0x80, chipselect); // zapuskaem obmen

	// wait for a transfer completion
	doJob = 1;
	//HAL_GPIO_WritePin(GPIOA, debug_oscil1_Pin, GPIO_PIN_SET);
	while((i<100) && doJob)
	{
		reg52_value = read_byte(52, chipselect);

		if((reg52_value & x80) == 0) // transfer complete
			doJob = 0;
		else
		{
			i++;
			short_delay();
		}

	}
	//HAL_GPIO_WritePin(GPIOA, debug_oscil1_Pin, GPIO_PIN_RESET);

	if(doJob) // timeout occured
		return 0xff;
	else	// everything ok
	{
		magnetometer_data = read_byte(53, chipselect);
		return magnetometer_data;
	}

}


// reads magnetometer data; returns them through parameters
void read_magnetometer_data(int16_t *hx, int16_t *hy, int16_t *hz,int chipselect)
{
	uint8_t high_byte;
	uint8_t low_byte;
	uint16_t buffer;

	high_byte = read_i2c_byte(0x04, chipselect);
	low_byte = read_i2c_byte(0x03, chipselect);
	buffer = ((uint16_t)high_byte)<<8;
	buffer |= (uint16_t)low_byte;
	*hx = (int16_t)buffer;

	high_byte = read_i2c_byte(0x06, chipselect);
	low_byte = read_i2c_byte(0x05, chipselect);
	buffer = ((uint16_t)high_byte)<<8;
	buffer |= (uint16_t)low_byte;
	*hy = (int16_t)buffer;

	high_byte = read_i2c_byte(0x08, chipselect);
	low_byte = read_i2c_byte(0x07, chipselect);
	buffer = ((uint16_t)high_byte)<<8;
	buffer |= (uint16_t)low_byte;
	*hz = (int16_t)buffer;

	//printf("st2 = %x\r\n", read_i2c_byte(0x09)); // read status2 (finish data reading)
	read_i2c_byte(0x09, chipselect); // read status2 (finish data reading)
}


void init_sensors()
{
	int i;

	for(i=0; i<NUMBER_OF_SENSORS; i++)
	{

		// reset MPU9250
        write_byte(107, (uint8_t)0x80, i); // write in register 107 (Power Management) value 0x80 (reset)
        long_delay();
        // configure serial interface
		// register 106 user control
		// set bit 6 enable fifo
		// set bit 5 enable i2c master
		// set bit 4 disable i2c interface
		// 0111 0000
		write_byte(106, (uint8_t)0x70, i); // write in register 106 value 0x70

	}


	for(i=0; i<NUMBER_OF_SENSORS; i++)
	{

		// configure accelerometer
		write_byte(28, (uint8_t)0x18, i); // write in register 28 (accel configuration) value 0x18 (16 g)
		// configure gyroscope
		write_byte(27, (uint8_t)0x10, i); // write in register 27 (gyro configuration) value 0x10 (1000 dps)

		// *** configure magnetometer ***
		// set 400 KHz clock
		write_byte(36, 0x0d, i);

		// make selftest
		write_i2c_byte(0x0c, 0x40, i);
		//printf("made magnetometer selftest\r\n");
		// 14 bit, nepreryvnye izmereniya rezhim 2 (100 Hz)
		if(!write_i2c_byte(0x0a, 0x06, i))
			//printf("i2c write OK\r\n");
			;
		else
			//printf("i2c write timeout\r\n");
			;

		sprintf((char *)message, "who am i = %x\r\n", read_i2c_byte(0x00, i));
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
		//print_register(106);

	}
	long_delay(); //********************************************************************************************



	// end configure data
	//************************************************************

}


// Private function prototypes -----------------------------------------------


// eeprom map *************************************************
// 0x08080000 mark (0x12345678)
// 0x08080004 gyro x calibration offset
// 0x08080008 gyro y calibration offset
// 0x0808000c gyro z calibration offset
// 0x08080010 accel x calibration offset
// 0x08080014 accel y calibration offset
// 0x08080018 accel z calibration offset
// 0x0808001c magnet x calibration offset
// 0x08080020 magnet y calibration offset
// 0x08080024 magnet z calibration offset

int main(void)
{

	int32_t gyro_calibration_x = 0;
	int32_t gyro_calibration_y = 0;
	int32_t gyro_calibration_z = 0;
	int32_t accel_calibration_x = 0;
	int32_t accel_calibration_y = 0;
	int32_t accel_calibration_z = 0;
	int32_t magnet_calibration_x = 0;
	int32_t magnet_calibration_y = 0;
	int32_t magnet_calibration_z = 0;




	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t hx, hy, hz;

	// MCU Configuration----------------------------------------------------------

	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_SPI1_Init();
	// enable spi1
	SPI1->CR1 |= SPI_CR1_SPE;
	MX_SPI2_Init();
	SPI2->CR2 &= ~SPI_CR2_TXEIE;   // disable txe interrupt
	SPI2->CR2 &= ~SPI_CR2_ERRIE;   // disable error interrupt
	SPI2->CR2 |= SPI_CR2_RXNEIE;   // enable rxne interrupt
	// enable spi2
	SPI2->CR1 |= SPI_CR1_SPE;
	MX_TIM2_Init();
	MX_USART1_UART_Init();

#ifdef SAVE_CALIBRATION
	gyro_calibration_x =	 GX;
	gyro_calibration_y =	 GY;
	gyro_calibration_z =	 GZ;
	accel_calibration_x =	 AX;
	accel_calibration_y =	 AY;
	accel_calibration_z =	 AZ;
	magnet_calibration_x =	 MX;
	magnet_calibration_y =	 MY;
	magnet_calibration_z =	 MZ;

	eeprom_write_mark();
	eeprom_write_int32_value(gyro_calibration_x, EEPROM_GYRO_X);
	eeprom_write_int32_value(gyro_calibration_y, EEPROM_GYRO_Y);
	eeprom_write_int32_value(gyro_calibration_z, EEPROM_GYRO_Z);
	eeprom_write_int32_value(accel_calibration_x, EEPROM_ACCEL_X);
	eeprom_write_int32_value(accel_calibration_y, EEPROM_ACCEL_Y);
	eeprom_write_int32_value(accel_calibration_z, EEPROM_ACCEL_Z);
	eeprom_write_int32_value(magnet_calibration_x, EEPROM_MAGNET_X);
	eeprom_write_int32_value(magnet_calibration_y, EEPROM_MAGNET_Y);
	eeprom_write_int32_value(magnet_calibration_z, EEPROM_MAGNET_Z);

#endif

	// read calibration
	uint32_t eeprom_mark = eeprom_read_mark();
	if(eeprom_mark == 0x12345678)  // est' kalibrovochnye dannye v pamyati
	{
		// chitaem kalibrovochnye dannye:
		gyro_calibration_x = eeprom_read_int32_value(EEPROM_GYRO_X);
		gyro_calibration_y = eeprom_read_int32_value(EEPROM_GYRO_Y);
		gyro_calibration_z = eeprom_read_int32_value(EEPROM_GYRO_Z);
		accel_calibration_x = eeprom_read_int32_value(EEPROM_ACCEL_X);
		accel_calibration_y = eeprom_read_int32_value(EEPROM_ACCEL_Y);
		accel_calibration_z = eeprom_read_int32_value(EEPROM_ACCEL_Z);
		magnet_calibration_x = eeprom_read_int32_value(EEPROM_MAGNET_X);
		magnet_calibration_y = eeprom_read_int32_value(EEPROM_MAGNET_Y);
		magnet_calibration_z = eeprom_read_int32_value(EEPROM_MAGNET_Z);
	}



	init_sensors();

	//***********************************************
	//uint32_t eeprom_mark = eeprom_read_mark();
	//eeprom_write_mark();
	//eeprom_mark = eeprom_read_mark();
	//eeprom_clear_mark();
	//eeprom_mark = eeprom_read_mark();



	int counter = 0;
	uint8_t spi2_in_data = 0x55;
	while (1)
	{

		int i,j,k;

		//************************************
		//*
		for(j=0; j<9; j++)
			fill_buffer[j] = 0;

		for(k=0; k<2; k++)
		{
			for(i=0; i<4; i++)
			{
				// get magnetometer data
				read_magnetometer_data(&hx, &hy, &hz, i);
				// rotate magnetometer axes
				int aux;
				aux = hx;
				hx = hy;
				hy = aux;
				hz = -hz;
				fill_buffer[6] += hx;
				fill_buffer[7] += hy;
				fill_buffer[8] += hz;
				// get accelerometer data
				accel_x = read_word(59, 60, i);
				accel_y = read_word(61, 62, i);
				accel_z = read_word(63, 64, i);
				fill_buffer[3] += accel_x;
				fill_buffer[4] += accel_y;
				fill_buffer[5] += accel_z;
				// get gyroscope data
				gyro_x = read_word(67, 68, i);
				gyro_y = read_word(69, 70, i);
				gyro_z = read_word(71, 72, i);
				fill_buffer[0] += gyro_x;
				fill_buffer[1] += gyro_y;
				fill_buffer[2] += gyro_z;
			}
			HAL_Delay(2);
		}

		for(i=0; i<9; i++)
		{
			fill_buffer[i] /= 8;
		}

		// calibration
		fill_buffer[0] += (int16_t)gyro_calibration_x;
		fill_buffer[1] += (int16_t)gyro_calibration_y;
		fill_buffer[2] += (int16_t)gyro_calibration_z;
		fill_buffer[3] += (int16_t)accel_calibration_x;
		fill_buffer[4] += (int16_t)accel_calibration_y;
		fill_buffer[5] += (int16_t)accel_calibration_z;
		fill_buffer[6] += (int16_t)magnet_calibration_x;
		fill_buffer[7] += (int16_t)magnet_calibration_y;
		fill_buffer[8] += (int16_t)magnet_calibration_z;

		// change buffers
		uint16_t *aux_pointer = out_buffer;
		out_buffer = fill_buffer;
		fill_buffer = aux_pointer;


		HAL_GPIO_TogglePin(GPIOA, debug_oscil1_Pin);


		/*
		sprintf(message, "*****************************************************\r\n", hx, hy, hz);
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
		sprintf(message, "%d   %d   %d\r\n", hx, hy, hz);
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
		sprintf(message, "%d   %d   %d\r\n", accel_x, accel_y, accel_z);
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
		sprintf(message, "%d   %d   %d\r\n", gyro_x, gyro_y, gyro_z);
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
		//*/

		//***********************************************************

	}// end while


}// end main

/// System Clock Configuration

void SystemClock_Config(void)
{

RCC_OscInitTypeDef RCC_OscInitStruct;
RCC_ClkInitTypeDef RCC_ClkInitStruct;
RCC_PeriphCLKInitTypeDef PeriphClkInit;

__PWR_CLK_ENABLE();

__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.HSICalibrationValue = 16;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
HAL_RCC_OscConfig(&RCC_OscInitStruct);

RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

// SysTick_IRQn interrupt configuration
HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

// USER CODE BEGIN 4

// USER CODE END 4

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
// USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
// USER CODE END 6 */

}

#endif






//***************************************************************************************************
/**
* @}
*/

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
