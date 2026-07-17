// I2Cヘルパー関数

#include "mcc_generated_files\system\pins.h"
#include "mcc_generated_files/system/system.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// タイムアウト設定 (例: 10us × 1000回 = 10ms)
// 通信速度100kHzの場合、1バイト転送に約100usかかる
#define I2C_TIMEOUT_COUNT 1000

// --- 書き込み関数（マルチバイト対応＆タイムアウト付き） ---
bool MCP23017_WriteData(uint16_t target_addr, uint8_t start_reg, uint8_t *data, size_t data_len) {
    // MCP23017の全レジスタ数は22個なので、十分なサイズのバッファを用意
    uint8_t tx_buffer[32]; 
    
    // バッファオーバーラン防止
    if (data_len > 30) return false; 

    // 先頭にレジスタアドレス、続けて書き込むデータを配置
    tx_buffer[0] = start_reg;
    memcpy(&tx_buffer[1], data, data_len);

    // MCCのAPI呼び出し (送信バイト数は data_len + 1)
    if (!I2C1_Write(target_addr, tx_buffer, data_len + 1)) {
        return false;
    }

    // タイムアウト付きのBusy待機
    uint16_t timeout = I2C_TIMEOUT_COUNT;
    while(I2C1_IsBusy()) {
        __delay_us(10);
        if (--timeout == 0) return false; // タイムアウト発生
    }
    
    // タイムアウト付きのSTOPコンディション完了待機
    timeout = I2C_TIMEOUT_COUNT;
    while(SSP1CON2bits.PEN) {
        __delay_us(10);
        if (--timeout == 0) return false; // タイムアウト発生
    }
    __delay_us(50); // 安全マージン

    // エラーが起きていないか最終確認
    return (I2C1_ErrorGet() == I2C_ERROR_NONE);
}

// --- 単一バイト書き込み用ヘルパー関数（既存コードとの互換用） ---
bool MCP23017_WriteReg(uint16_t target_addr, uint8_t reg, uint8_t data) {
    return MCP23017_WriteData(target_addr, reg, &data, 1);
}

// --- 読み込み関数（タイムアウト付き） ---
bool MCP23017_ReadData(uint16_t target_addr, uint8_t start_reg, uint8_t *data, size_t data_len) {
    uint8_t reg_addr = start_reg;

    if (!I2C1_WriteRead(target_addr, &reg_addr, 1, data, data_len)) {
        return false;
    }

    // タイムアウト付きのBusy待機
    uint16_t timeout = I2C_TIMEOUT_COUNT;
    while(I2C1_IsBusy()) {
        __delay_us(10);
        if (--timeout == 0) return false; // タイムアウト発生
    }
    
    // タイムアウト付きのSTOPコンディション完了待機
    timeout = I2C_TIMEOUT_COUNT;
    while(SSP1CON2bits.PEN) {
        __delay_us(10);
        if (--timeout == 0) return false; // タイムアウト発生
    }
    __delay_us(50); // 安全マージン

    return (I2C1_ErrorGet() == I2C_ERROR_NONE);
}
