#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

char sensor_data[DATA_SIZE];
uint8_t data2[6];		// buffer to store data for spi

I2C_HandleTypeDef hi2c3; // Declare an I2C handle
UART_HandleTypeDef huart2;
SPI_HandleTypeDef hspi1;

void SPI_Read(uint8_t *pBuffer, uint16_t NumByteToRead)
{
    // Implement SPI read function
    HAL_SPI_Receive(&hspi1, pBuffer, NumByteToRead, HAL_MAX_DELAY);
}

void SPI_Write(uint8_t *pBuffer, uint16_t NumByteToWrite)
{
    // Implement SPI write function
    HAL_SPI_Transmit(&hspi1, pBuffer, NumByteToWrite, HAL_MAX_DELAY);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
  if(htim->Instance == TIM6)
  {
	  // get data from the accelerometer sensor using i2c
//	  GetDataFromSensor_usingI2C();

	  // get data from the accelerometer sensor using spi
	  GetDataFromSensor_usingSPI();
  }
}

//void GetDataFromSensor_usingI2C()
//{
//	uint8_t data1[6];		// buffer to store accelerometer data for i2c
//
//	uint8_t reg = 0x28 | 0x80;		//Register to read (OUT_X_L) | Auto-increment flag
//
//	HAL_I2C_Master_Transmit(&hi2c3, LIS3DSH_ADDR, &reg, 1, HAL_MAX_DELAY);
//	HAL_I2C_Master_Receive(&hi2c3, LIS3DSH_ADDR, data1, 6, HAL_MAX_DELAY);
//
//	// Extract accelerometer data (assuming 16-bit signed data)
//	x = ((int16_t)data1[0] << 8 | data1[1]);
//	y = ((int16_t)data1[2] << 8 | data1[3]);
//	z = ((int16_t)data1[4] << 8 | data1[5]);
//
//	sprintf(sensor_data, "X:%d, Y:%d, Z:%d\r\n", x, y, z);
//
//	 // transmit data over uart
//	HAL_UART_Transmit(&huart2, (uint8_t *)sensor_data, strlen(sensor_data), HAL_MAX_DELAY);
//}

int16_t x,y,z;
void GetDataFromSensor_usingSPI()
{
	uint8_t txData[2];
	uint8_t rxData[6];

	// Configure CTRL_REG4 to enable X, Y, Z axes
	txData[0] = LIS3DSH_CTRL_REG4;
	txData[1] = 0x67; // Configuration byte (specific to your requirements)
	SPI_Write(txData, 2);

	// Read accelerometer data from OUT_X_L (X-axis data register)
	txData[0] = LIS3DSH_OUT_X_L | 0x80; // Set MSB to indicate read operation
	SPI_Write(txData, 1);
	SPI_Read(rxData, 6);

	// Process received data (assuming it's 16-bit signed integers)
	x = (int16_t)((rxData[1] << 8) | rxData[0]); // X-axis data
	y = (int16_t)((rxData[3] << 8) | rxData[2]); // Y-axis data
	z = (int16_t)((rxData[5] << 8) | rxData[4]); // Z-axis data

	sprintf(sensor_data, "X:%d, Y:%d, Z:%d\r\n", x, y, z);

	 // transmit data over uart
	HAL_UART_Transmit(&huart2, (uint8_t *)sensor_data, strlen(sensor_data), HAL_MAX_DELAY);
}

