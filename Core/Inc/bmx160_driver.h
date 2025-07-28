#ifndef INC_BMX160_DRIVER_H_
#define INC_BMX160_DRIVER_H_

#include "main.h" // Required for HAL types like I2C_HandleTypeDef
#include <stdint.h> // Required for int16_t type

/**
  * @brief  Initializes the BMX160 sensor for Accelerometer and Gyroscope.
  * @param  i2c_handle: Pointer to the I2C handle used for communication.
  * @param  uart_handle: Pointer to the UART handle for printing error messages. Can be NULL if not needed.
  * @retval HAL_StatusTypeDef: HAL_OK if initialization is successful.
  */
HAL_StatusTypeDef BMX160_Init(I2C_HandleTypeDef *i2c_handle, UART_HandleTypeDef *uart_handle);

/**
  * @brief  Reads Accelerometer and Gyroscope data from the BMX160.
  * @param  i2c_handle: Pointer to the I2C handle used for communication.
  * @param  acc_x: Pointer to store Accelerometer X-axis data.
  * @param  acc_y: Pointer to store Accelerometer Y-axis data.
  * @param  acc_z: Pointer to store Accelerometer Z-axis data.
  * @param  gyr_x: Pointer to store Gyroscope X-axis data.
  * @param  gyr_y: Pointer to store Gyroscope Y-axis data.
  * @param  gyr_z: Pointer to store Gyroscope Z-axis data.
  * @retval None
  */
void BMX160_Read_Acc_Gyr(I2C_HandleTypeDef *i2c_handle, int16_t* acc_x, int16_t* acc_y, int16_t* acc_z, int16_t* gyr_x, int16_t* gyr_y, int16_t* gyr_z);


#endif /* INC_BMX160_DRIVER_H_ */
