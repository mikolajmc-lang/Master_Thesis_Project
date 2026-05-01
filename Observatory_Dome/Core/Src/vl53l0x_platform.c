/*
 * vl53l0x_platform.c
 *
 *  Created on: May 1, 2026
 *      Author: 48694
 */

#include "main.h"
#include "i2c.h"
#include "vl53l0x_platform.h"

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 500);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
	uint8_t buffer[2];
	// np. 0xABCD
	buffer[0] = (uint8_t)(data >> 8); // 0xABCD -> 0x00AB -> (uint8_t)0xAB
	buffer[1] = (uint8_t)(data & 0xFF); //0xABCD -> 0x00CD -> (uint8_t)0xCD

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
	uint8_t buffer[4];

	buffer[0] = (uint8_t)(data >> 24);
	buffer[1] = (uint8_t)(data >> 16);
	buffer[2] = (uint8_t)(data >> 8);
	buffer[3] = (uint8_t)(data & 0xFF);

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;

}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 500);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;

}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, data, 1, 100);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
	uint8_t buffer[2];

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	*data = (uint16_t)(buffer[0] << 8 | buffer[1]);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
	uint8_t buffer[4];

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(Dev->I2CHandle, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);

	*data = (uint32_t)(buffer[0] << 24 | buffer[1] << 16 | buffer[2] << 8 | buffer[3]);

	if(status == HAL_OK)
		return VL53L0X_ERROR_NONE;
	else
		return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	VL53L0X_Error status;
	uint8_t data;

	status = VL53L0X_RdByte(Dev, index, &data);

	if(status == VL53L0X_ERROR_NONE)
	{
		data = (data & AndData) | OrData;

		status = VL53L0X_WrByte(Dev, index, data);
	}

	return status;
}



VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
	HAL_Delay(1);

	return VL53L0X_ERROR_NONE;
}



