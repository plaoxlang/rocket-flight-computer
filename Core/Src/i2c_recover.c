#include <i2c_recover.h>

#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN  GPIO_PIN_8
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN  GPIO_PIN_9

void I2C_BusClear(I2C_HandleTypeDef *hi2c1)
{
    HAL_I2C_DeInit(hi2c1);

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = I2C_SCL_PIN; HAL_GPIO_Init(I2C_SCL_PORT, &g);
    g.Pin = I2C_SDA_PIN; HAL_GPIO_Init(I2C_SDA_PORT, &g);

    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    // Up to 9 clock pulses so a slave stuck mid-byte releases SDA
    int i;
    for (i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // Generate a STOP condition
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
}