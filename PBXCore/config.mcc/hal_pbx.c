/*
PBXCore HAL
PIC 16F18857 4回線用
ハードウエア(PICのピン配置など)依存部分はここで定義する
MCCでピン名を設定しhal_pbx.hとhal_pbx.cで設定すること
ピン命名基準
  Lx_TG1  : SLICユニットのTG1
  Lx_TG2  : SLICユニットのTG2
  Lx_HOOK : SLICユニットのフック出力
  Lx_RING : SLICユニットのリング制御
  SW_TX   : スイッチボード制御用ソフトウェアシリアル出力
  
I/Oエクスパンダ(MCP23017)もこのHALで対応する
*/

// 以下の構成はPIC直結は4ライン用。L1_,L2...でピン名を定義する
// I/Oエクスパンダ使用時のピン配置は以下の通り
//  7  6   5   4  3  2   1   0
// HO RI TG2 TG1 HO RI TG2 TG1
// GPIOAがL4,5、GPIOBがL5,6 となる

#include "hal_pbx.h"
#include "i2c_helper.h"
#include "mcc_generated_files\system\pins.h"
#include "mcc_generated_files/system/system.h"

// --- MCP23017用定義 ---
#define MCP23017_ADDR  0x20
#define MCP_REG_IOCON  0x0a
#define MCP_REG_IODIRA 0x00
#define MCP_REG_GPPUA  0x0C
#define MCP_REG_GPIOA  0x12
#define MCP_REG_OLATA  0x14

#define EXT_LINE_START 4 // 拡張回線の開始インデックス (0〜3はPIC直結)

// 最大接続回線数。I2Cをチェックして回線数を決定する
// デフォルトは4(直結数)
static uint8_t active_lines =4 ;

// MCP23017 出力シャドウレジスタ [0]:OLATA, [1]:OLATB
// 初期状態は全てHigh(OFF)にしておく
static uint8_t mcp_olat[2] = {0xFF, 0xFF}; 

// MCP23017 入力キャッシュレジスタ [0]:GPIOA, [1]:GPIOB
static uint8_t mcp_gpio_cache[2] = {0xFF, 0xFF};

// 起動時の初期化
//  すべてのトーンとRINGをOFF
//  ソフトウェアUARTのTXをHighに
void HAL_PBX_Init(void) {

    // PIC直結分のトーン初期化とRingオフに
    for (uint8_t i = 0; i < EXT_LINE_START; i++) {
        HAL_SetTone(i, TONE_OFF);
        HAL_SetRing(i, false);
    }

    // I2Cバス安定まで念のため待つ
    __delay_ms(500);
    I2C1_Initialize();
    __delay_ms(100);
    SW_TX_SetHigh();

    // 1. BANK=0 を強制するトリックかつI2Cの接続チェック
    // 万が一 BANK=1 になっていた場合、0x05 は IOCON なので BANK=0 に復帰する。
    // BANK=0 だった場合、0x05 は IPOLA (入力極性) なので 0x00 を書いても無害。
    if(MCP23017_WriteReg(MCP23017_ADDR, 0x05, 0x00) == true){
    
        // この時点で確実に BANK=0 になっているので、改めて IOCON(0x0A) を初期化
        MCP23017_WriteReg(MCP23017_ADDR, MCP_REG_IOCON, 0x00);

        // 2. 出力ピンの初期化 (先に行う)
        // 出力モードに切り替えた瞬間の誤作動(グリッチ)を防ぐため、
        // 方向(IODIR)を変える前に、あらかじめ出力ラッチに安全な値(全てHigh=OFF)をセットする
        mcp_olat[0] = 0xFF; // OLATA
        mcp_olat[1] = 0xFF; // OLATB
        MCP23017_WriteData(MCP23017_ADDR, MCP_REG_OLATA, mcp_olat, 2);

        // 3. 入力ピンの内部プルアップ有効化 (Bit 3, 7 = HOOK)
        uint8_t gppu_data[2] = {0x88, 0x88};
        MCP23017_WriteData(MCP23017_ADDR, MCP_REG_GPPUA, gppu_data, 2);

        // 4. 入出力方向の設定 (Bit 3, 7 は入力、それ以外は出力)
        // この設定を書き込んだ瞬間に、ピンが実際の出力として有効になる
        uint8_t iodir_data[2] = {0x88, 0x88};
        MCP23017_WriteData(MCP23017_ADDR, MCP_REG_IODIRA, iodir_data, 2);

        active_lines = 8;
    }
    else {
        active_lines = 4;
    }
    
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
    if(line < EXT_LINE_START){ //直結回線時
        switch (line) {
            case 0:
                if (tg1) { L1_TG1_SetHigh(); } else { L1_TG1_SetLow(); }
                if (tg2) { L1_TG2_SetHigh(); } else { L1_TG2_SetLow(); }
                break;
            case 1:
                if (tg1) { L2_TG1_SetHigh(); } else { L2_TG1_SetLow(); }
                if (tg2) { L2_TG2_SetHigh(); } else { L2_TG2_SetLow(); }
                break;
            case 2:
                if (tg1) { L3_TG1_SetHigh(); } else { L3_TG1_SetLow(); }
                if (tg2) { L3_TG2_SetHigh(); } else { L3_TG2_SetLow(); }
                break;
            case 3:
                if (tg1) { L4_TG1_SetHigh(); } else { L4_TG1_SetLow(); }
                if (tg2) { L4_TG2_SetHigh(); } else { L4_TG2_SetLow(); }
                break;
        }
    }
    else if(active_lines > 4) {
        // 拡張回線の場合
        uint8_t ext_idx  = line - EXT_LINE_START;
        uint8_t port_idx = (ext_idx < 2) ? 0 : 1;
        uint8_t shift    = (ext_idx % 2 == 0) ? 0 : 4;

        // 対象回線のTG1(Bit0), TG2(Bit1)をクリア
        mcp_olat[port_idx] &= ~(0x03 << shift);

        // 値に応じてビットをセット
        if (tg1) mcp_olat[port_idx] |= (0x01 << shift);
        if (tg2) mcp_olat[port_idx] |= (0x02 << shift);

        // シャドウレジスタの値を書き込み
        uint8_t reg = (port_idx == 0) ? MCP_REG_OLATA : (MCP_REG_OLATA + 1);
        MCP23017_WriteReg(MCP23017_ADDR, reg, mcp_olat[port_idx]);
    }
}

// 鳴動制御 (enable: true=ベルを鳴らす, false=停止)
void HAL_SetRing(uint8_t line, bool enable) {
    // ユニット側はL=鳴動, H=停止
    if(line < EXT_LINE_START){ //直結回線時処理
        switch (line) {
            case 0:
                if (enable) { L1_RING_SetLow(); } else { L1_RING_SetHigh(); }
                break;
            case 1:
                if (enable) { L2_RING_SetLow(); } else { L2_RING_SetHigh(); }
                break;
            case 2:
                if (enable) { L3_RING_SetLow(); } else { L3_RING_SetHigh(); }
                break;
            case 3:
                if (enable) { L4_RING_SetLow(); } else { L4_RING_SetHigh(); }
                break;
        }
    }
    else if(active_lines > 4) {
        // 拡張回線の場合
        uint8_t ext_idx  = line - EXT_LINE_START;
        uint8_t port_idx = (ext_idx < 2) ? 0 : 1;
        uint8_t shift    = (ext_idx % 2 == 0) ? 0 : 4;

        // 対象回線のRINGビット(Bit2)をクリア
        mcp_olat[port_idx] &= ~(0x04 << shift);
        // RINGオフ(H)の場合はビットを立てる（オンの場合はLなのでそのまま）
        if (!enable) {
            mcp_olat[port_idx] |= (0x04 << shift);
        }

        // シャドウレジスタの値を書き込み
        // ※対象ポート(AまたはB)のみ1バイト書き込めばOK
        uint8_t reg = (port_idx == 0) ? MCP_REG_OLATA : (MCP_REG_OLATA + 1);
        MCP23017_WriteReg(MCP23017_ADDR, reg, mcp_olat[port_idx]);
    }

}

// フック状態の取得（キャッシュトリック）
bool HAL_GetHook(uint8_t line) {
    // PIC直結回線の場合
    if (line < EXT_LINE_START) {
        switch (line) {
            case 0: return L1_HOOK_GetValue() == 1;
            case 1: return L2_HOOK_GetValue() == 1;
            case 2: return L3_HOOK_GetValue() == 1;
            case 3: return L4_HOOK_GetValue() == 1;
        }
    } 
    // 拡張回線(MCP23017)の場合
    else if(active_lines > 4) {
        uint8_t ext_idx = line - EXT_LINE_START; // 0, 1, 2, 3

        // 【トリック】拡張回線の最初 (line == 4) が呼ばれた時だけI2C通信を行う
        if (line == EXT_LINE_START) {
            MCP23017_ReadData(MCP23017_ADDR, MCP_REG_GPIOA, mcp_gpio_cache, 2);
        }

        // 以降は取得済みのキャッシュからビットを判定して返すだけ
        // ext_idx = 0: PortA Bit3 (0x08)
        // ext_idx = 1: PortA Bit7 (0x80)
        // ext_idx = 2: PortB Bit3 (0x08)
        // ext_idx = 3: PortB Bit7 (0x80)
        uint8_t port_idx = (ext_idx < 2) ? 0 : 1; 
        uint8_t mask     = (ext_idx % 2 == 0) ? 0x08 : 0x80;

        return (mcp_gpio_cache[port_idx] & mask) != 0;
    }
    return false;
}

// 最大回線数を取得する関数
uint8_t HAL_GetMaxLines(void) {
    return active_lines;
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