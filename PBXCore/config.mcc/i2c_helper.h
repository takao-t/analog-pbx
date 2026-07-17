// i2c_helper.h
#ifndef I2C_HELPER_H
#define I2C_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

bool MCP23017_WriteData(uint16_t target_addr, uint8_t start_reg, uint8_t *data, size_t data_len);
bool MCP23017_WriteReg(uint16_t target_addr, uint8_t reg, uint8_t data);
bool MCP23017_ReadData(uint16_t target_addr, uint8_t start_reg, uint8_t *data, size_t data_len);

#endif // I2C_HELPER_H