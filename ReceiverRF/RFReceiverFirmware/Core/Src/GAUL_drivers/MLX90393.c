/*
 *  I2C MLX90393 Library
 */

#include "GAUL_drivers/MLX90393.h"


HAL_StatusTypeDef I2C_MLX90393_Init(I2C_MLX90393_HandleTypeDef *hi2cd) {
  HAL_StatusTypeDef res;
  uint8_t mlx_status;

  res = I2C_MLX90393_Reset(hi2cd, &mlx_status);

  return res;
}


HAL_StatusTypeDef I2C_MLX90393_WriteCommand(
  I2C_MLX90393_HandleTypeDef *hi2cd,
  uint8_t *cmd,
  uint8_t len
)
{
	return HAL_I2C_Master_Transmit(hi2cd->hi2c, (hi2cd->address << 1) | 0x1, cmd, len, HAL_MAX_DELAY);
}


HAL_StatusTypeDef I2C_MLX90393_Receive(
  I2C_MLX90393_HandleTypeDef *hi2cd,
  uint8_t *buf,
  size_t len
)
{
	return HAL_I2C_Master_Receive(hi2cd->hi2c, hi2cd->address << 1, buf, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef I2C_MLX90393_Reset(
  I2C_MLX90393_HandleTypeDef *hi2cd,
  uint8_t *status
)
{
	HAL_StatusTypeDef hal_status;
	uint8_t cmd[] = {MLX90393_CMD_RT << 4};

	hal_status = I2C_MLX90393_WriteCommand(hi2cd, cmd, 1);
	if (hal_status != HAL_OK) return hal_status;

	return I2C_MLX90393_Receive(hi2cd, status, 1);
}
