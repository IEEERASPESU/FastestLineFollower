#include "bmx160_driver.h"
#include <stdio.h>
#include <string.h>

// BMX160 I2C Address
#define BMX160_I2C_ADDR         (0x68 << 1) // Use 8-bit address

// BMX160 Register Addresses
#define BMX160_CHIP_ID_REG      0x00
#define BMX160_CMD_REG          0x7E
#define BMX160_ACC_DATA_REG     0x12
#define BMX160_GYR_DATA_REG     0x0C

// BMX160 Commands
#define BMX160_ACC_NORMAL_MODE  0x11
#define BMX160_GYR_NORMAL_MODE  0x15
#define BMX160_SOFT_RESET_CMD   0xB6


HAL_StatusTypeDef BMX160_Init(I2C_HandleTypeDef *i2c_handle, UART_HandleTypeDef *uart_handle)
{
    uint8_t cmd;
    uint8_t chip_id = 0;
    char uart_buf[50];
    HAL_StatusTypeDef status;

    // --- 1. Soft-reset the BMX160 ---
    cmd = BMX160_SOFT_RESET_CMD;
    status = HAL_I2C_Mem_Write(i2c_handle, BMX160_I2C_ADDR, BMX160_CMD_REG, 1, &cmd, 1, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(100); // Wait for the sensor to restart

    // --- Sanity Check: Read Chip ID ---
    status = HAL_I2C_Mem_Read(i2c_handle, BMX160_I2C_ADDR, BMX160_CHIP_ID_REG, 1, &chip_id, 1, 100);
    if (status != HAL_OK) return status;

    if (chip_id != 0xD8) {
        // If a UART handle is provided, print an error message
        if (uart_handle != NULL) {
            sprintf(uart_buf, "BMX160 Chip ID Mismatch! Found: 0x%02X\r\n", chip_id);
            HAL_UART_Transmit(uart_handle, (uint8_t*)uart_buf, strlen(uart_buf), 200);
        }
        return HAL_ERROR; // Indicate failure
    }

    // --- 2. Power up the Accelerometer and Gyroscope to Normal Mode ---
    cmd = BMX160_ACC_NORMAL_MODE;
    status = HAL_I2C_Mem_Write(i2c_handle, BMX160_I2C_ADDR, BMX160_CMD_REG, 1, &cmd, 1, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(50);

    cmd = BMX160_GYR_NORMAL_MODE;
    status = HAL_I2C_Mem_Write(i2c_handle, BMX160_I2C_ADDR, BMX160_CMD_REG, 1, &cmd, 1, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(100); // Gyro needs more time to stabilize

    return HAL_OK; // Success
}

void BMX160_Read_Acc_Gyr(I2C_HandleTypeDef *i2c_handle, int16_t* acc_x, int16_t* acc_y, int16_t* acc_z, int16_t* gyr_x, int16_t* gyr_y, int16_t* gyr_z)
{
    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    // Read accelerometer and gyroscope data registers
    HAL_I2C_Mem_Read(i2c_handle, BMX160_I2C_ADDR, BMX160_ACC_DATA_REG, 1, accel_data, 6, 100);
    HAL_I2C_Mem_Read(i2c_handle, BMX160_I2C_ADDR, BMX160_GYR_DATA_REG, 1, gyro_data, 6, 100);

    // Process raw data into 16-bit signed integers
    // Data is in Little Endian format (LSB first, then MSB)
    *acc_x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    *acc_y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    *acc_z = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    *gyr_x = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    *gyr_y = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    *gyr_z = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);
}
