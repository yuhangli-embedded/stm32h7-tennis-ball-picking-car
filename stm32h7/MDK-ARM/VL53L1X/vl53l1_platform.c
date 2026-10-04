#include "vl53l1_platform.h"
#include "main.h"         // for extern hi2c1 and HAL_Delay
#include "stm32h7xx_hal.h"

// 外部声明 CubeMX 生成的 I2C 句柄
extern I2C_HandleTypeDef hi2c1;

// Helper：用 HAL_I2C_Mem_xxx 完成读写
static HAL_StatusTypeDef I2C_Write(uint16_t dev, uint16_t reg, uint8_t *p, uint32_t len) {
    return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_16BIT, p, len, 1000);
}
static HAL_StatusTypeDef I2C_Read (uint16_t dev, uint16_t reg, uint8_t *p, uint32_t len) {
    return HAL_I2C_Mem_Read (&hi2c1, dev, reg, I2C_MEMADD_SIZE_16BIT, p, len, 1000);
}

// 平台接口实现
int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    return (I2C_Write(dev, index, pdata, count) == HAL_OK) ? 0 : -1;
}
int8_t VL53L1_ReadMulti (uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    return (I2C_Read (dev, index, pdata, count) == HAL_OK) ? 0 : -1;
}
int8_t VL53L1_WrByte   (uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}
int8_t VL53L1_WrWord   (uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buf[2] = { (uint8_t)(data>>8), (uint8_t)(data & 0xFF) };
    return VL53L1_WriteMulti(dev, index, buf, 2);
}
int8_t VL53L1_WrDWord  (uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buf[4] = {
        (uint8_t)(data>>24), (uint8_t)(data>>16),
        (uint8_t)(data>>8),  (uint8_t)(data   &0xFF)
    };
    return VL53L1_WriteMulti(dev, index, buf, 4);
}
int8_t VL53L1_RdByte   (uint16_t dev, uint16_t index, uint8_t *pdata) {
    return VL53L1_ReadMulti(dev, index, pdata, 1);
}
int8_t VL53L1_RdWord   (uint16_t dev, uint16_t index, uint16_t *pdata) {
    uint8_t buf[2];
    if (VL53L1_ReadMulti(dev, index, buf, 2)) return -1;
    *pdata = (uint16_t)(buf[0]<<8 | buf[1]);
    return 0;
}
int8_t VL53L1_RdDWord  (uint16_t dev, uint16_t index, uint32_t *pdata) {
    uint8_t buf[4];
    if (VL53L1_ReadMulti(dev, index, buf, 4)) return -1;
    *pdata = ((uint32_t)buf[0]<<24) | ((uint32_t)buf[1]<<16)
           | ((uint32_t)buf[2]<<8)  |  (uint32_t)buf[3];
    return 0;
}
int8_t VL53L1_WaitMs   (uint16_t dev, int32_t wait_ms) {
    HAL_Delay(wait_ms);
    return 0;
}
