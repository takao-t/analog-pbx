/*
PBXCore HAL
PIC 16F18326 2回線用
ハードウエア(PICのピン配置など)依存部分はここで定義する
MCCでピン名を設定しhal_pbx.hとhal_pbx.cで設定すること
*/

// 以下の構成は2ライン用。L0_,L1でピン名を定義する

#include "hal_pbx.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/system/system.h"

// 起動時の初期化
//  すべてのトーンとRINGをOFF
//  ソフトウェアUARTのTXをHighに
void HAL_PBX_Init(void) {
    for (uint8_t i = 0; i < 2; i++) {
        HAL_SetTone(i, TONE_OFF);
        HAL_SetRing(i, false);
    }
    SW_TX_SetHigh();
}

// 指定した回線のトーンを設定
void HAL_SetTone(uint8_t line, ToneType tone) {
    bool tg1 = true;  // Default: H
    bool tg2 = true;  // Default: H

    switch (tone) {
        case TONE_OFF:      tg1 = true;  tg2 = true;  break;
        case TONE_DIAL:     tg1 = false; tg2 = false; break; // L, L
        case TONE_RINGBACK: tg1 = true;  tg2 = false; break; // H, L
        case TONE_BUSY:     tg1 = false; tg2 = true;  break; // L, H
    }

    // 回線ごとのピン操作
    switch (line) {
        case 0:
            if (tg1) { L1_TG1_SetHigh(); } else { L1_TG1_SetLow(); }
            if (tg2) { L1_TG2_SetHigh(); } else { L1_TG2_SetLow(); }
            break;
        case 1:
            if (tg1) { L2_TG1_SetHigh(); } else { L2_TG1_SetLow(); }
            if (tg2) { L2_TG2_SetHigh(); } else { L2_TG2_SetLow(); }
            break;
    }
}

// 鳴動制御 (enable: true=ベルを鳴らす, false=停止)
void HAL_SetRing(uint8_t line, bool enable) {
    // ユニット側はL=鳴動, H=停止
    switch (line) {
        case 0:
            if (enable) { L1_RING_SetLow(); } else { L1_RING_SetHigh(); }
            break;
        case 1:
            if (enable) { L2_RING_SetLow(); } else { L2_RING_SetHigh(); }
            break;
    }
}

// フック状態の取得 (true=オフフック[H], false=オンフック[L])
bool HAL_GetHook(uint8_t line) {
    switch (line) {
        case 0: return L1_HOOK_GetValue() == 1;
        case 1: return L2_HOOK_GetValue() == 1;
        default: return false; // エラーフェールセーフ
    }
}

// ソフトウェアシリアル(bitbanging)送信
void HAL_SoftwareUART_WriteByte(uint8_t data)
{
    // ソフトウェアUARTの精度確保のため割り込みを禁止
    INTERRUPT_GlobalInterruptDisable();

    // スタートビット (Low)
    SW_TX_SetLow();
    __delay_us(BIT_DELAY_US);

    // データビット (8bit, LSB First)
    for (int i = 0; i < 8; i++) {
        if (data & 0x01) {
            SW_TX_SetHigh();
        } else {
            SW_TX_SetLow();
        }
        __delay_us(BIT_DELAY_US);
        data >>= 1;
    }

    // ストップビット (High)
    SW_TX_SetHigh();
    __delay_us(BIT_DELAY_US);
    
    // 割り込みを許可
    INTERRUPT_GlobalInterruptEnable();

    // ストップビットの安全マージン
    __delay_us(BIT_DELAY_US); 
}

// 14ピンではエクステンダが使用できないので常に2を返す
uint8_t HAL_GetMaxLines(void){
    return 2;
}