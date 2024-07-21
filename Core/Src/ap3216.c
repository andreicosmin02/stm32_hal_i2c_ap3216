#include "main.h"
#include "ap3216.h"

void AP3216_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[I2C_WRITE_DATA_SIZE];

    // Software Reset
    data[ADDRESS_DATA_INDEX] = SYS_REG_SYS_CONFIG;
    data[VALUE_DATA_INDEX] = SW_RESET;
    if (HAL_I2C_Master_Transmit
                (
                        hi2c,
                        AP3216_WRITE_ADD,
                        data,
                        I2C_WRITE_DATA_SIZE,
                        I2C_TIMEOUT
                )
        == HAL_ERROR)
    {
        Error_Handler();
    }

    // ALS and PS+IR functions active
    data[ADDRESS_DATA_INDEX] = SYS_REG_SYS_CONFIG;
    data[VALUE_DATA_INDEX] = ALS_PS_IR_ACTIVE;
    if (HAL_I2C_Master_Transmit
                (
                        hi2c,
                        AP3216_WRITE_ADD,
                        data,
                        I2C_WRITE_DATA_SIZE,
                        I2C_TIMEOUT
                )
        == HAL_ERROR)
    {
        Error_Handler();
    }
}
