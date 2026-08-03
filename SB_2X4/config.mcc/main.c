/*
アナログスイッチモジュール2x4
コマンドリレーによる自動拡張機能付き
PIC 16F18323
*/

#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files\system\system.h"
#include <pic.h>
#include <pic16f18323.h>
#include <string.h>


// ==========================================
//  設定・定数定義
//  注:PICのクロックは32MHz
// ==========================================

// 拡張ユニット間通信のボーレート (Bit-banging用)
#define SW_BAUD_RATE 9600
#define BIT_DELAY_US (1000000 / SW_BAUD_RATE)

// コマンドバッファサイズ (C0102 + CR + NULL = 7文字)
#define CMD_BUF_SIZE 10

// グローバル変数
char rxBuffer[CMD_BUF_SIZE];
uint8_t rxIndex = 0;
volatile bool sys_initialized = false;

// ==========================================
//  関数プロトタイプ
// ==========================================
void SoftwareUART_WriteByte(uint8_t data, bool isVertical);
void SoftwareUART_WriteString(const char* str, bool isVertical);
void ProcessCommand(char* cmd);
void TurnOffAllLocal(void);
void Initial_blink(void);

// ==========================================
//  メイン関数
// ==========================================
int main(void)
{
    // システム初期化 (MCC生成コード)
    SYSTEM_Initialize();

    // 割り込み許可(EUSARTが割り込みを使用する)
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    TMR2_PeriodMatchCallbackRegister(Initial_blink);

    // アイドル状態のTXラインをHighにしておく
    TX_V_SetHigh();
    TX_H_SetHigh();
 
    while (1)
    {
        // ウオッチドッグ・クリア
        // WDTはMCCで約1秒(1:32768)に設定
        CLRWDT();

        // EUSART(Hardware RX)にデータが来ているか確認
        // 注:ESUARTはMCCで割り込み、バッファ有に設定
        if (EUSART_IsRxReady())
        {
            uint8_t rxData = EUSART_Read();

            // 改行コード(CR: 0x0D)が来たらコマンド解析へ
            if (rxData == '\r' || rxData == '\n') {
                if (rxIndex > 0) {
                    rxBuffer[rxIndex] = '\0'; // 文字列終端
                    ProcessCommand(rxBuffer);
                    rxIndex = 0; // バッファリセット
                }
            }
            // 通常文字ならバッファにためる
            else {
                if (rxIndex < CMD_BUF_SIZE - 1) {
                    rxBuffer[rxIndex++] = (char)rxData;
                }
            }
        }
    }
}

// ==========================================
//  コマンド解析・ルーティング処理 (中核ロジック)
// ==========================================
void ProcessCommand(char* cmd)
{
    // 1. 全リセットコマンド (RFFFF)
    if (strcmp(cmd, "RFFFF") == 0) {
        // Global Reset
        // 初期イニシャライズされていない場合
        if(sys_initialized == false){
            TMR2IE = 0;
            TMR2_Deinitialize();
            SW_CTRL_1_SetLow();
            SW_CTRL_2_SetLow();
            SW_CTRL_3_SetLow();
            SW_CTRL_4_SetLow();
            SW_CTRL_5_SetLow();
            SW_CTRL_6_SetLow();
            SW_CTRL_7_SetLow();
            SW_CTRL_8_SetLow();
            sys_initialized = true;
        }
        TurnOffAllLocal();
        SoftwareUART_WriteString("RFFFF\r", true);  // 縦へ転送
        SoftwareUART_WriteString("RFFFF\r", false); // 横へ転送
        return;
    }


    // 書式チェック (5文字 Cxxyy)
    if (strlen(cmd) != 5) {
        //Command Error(must be 5 characters)
        return;
    }

    char type = cmd[0]; // 'C' or 'R'

    // コマンドチェック(C or R)
    if( type != 'C' && type != 'R'){
        //Command Error(must be 'C' or 'R')
        return;
    }
    
    // 座標のパース (文字コードから数値へ変換)
    // cmd[1,2] -> Y (縦), cmd[3,4] -> X (横)
    uint8_t y = (cmd[1] - '0') * 10 + (cmd[2] - '0');
    uint8_t x = (cmd[3] - '0') * 10 + (cmd[4] - '0');

    // 座標値チェック(1< 座標 <99)
    if( y < 1 || y > 99){
        // Range Error
        return;
    }
    if( x < 1 || x > 99){
        // Range Error
        return;
    }

    // --- ルーティングロジック ---

    // ケースA: 縦(Y)が自分の範囲(2)より大きい -> 下のユニットへ
    if (y > 2) {
        y -= 2; // 座標を2減らす
        // コマンド書き換え
        cmd[1] = (y / 10) + '0';
        cmd[2] = (y % 10) + '0';
        // Forward to V port
        // CRを付加して送信
        SoftwareUART_WriteString(cmd, true); // true = 縦方向
        SoftwareUART_WriteString("\r", true); // true = 縦方向
     }
    // ケースB: 縦は自分だが、横(X)が自分の範囲(4)より大きい -> 右のユニットへ
    else if (x > 4) {
        x -= 4; // 座標を2減らす
        // コマンド書き換え
        cmd[3] = (x / 10) + '0';
        cmd[4] = (x % 10) + '0';
        // Forward to H port
        SoftwareUART_WriteString(cmd, false); // false = 横方向
        SoftwareUART_WriteString("\r", false);// false = 方向向
    }
    // ケースC: 自分の担当範囲 (x<=4, y<=2)
    else {
        bool state = (type == 'C'); // 'C'ならON, それ以外('R')ならOFF
        //Local Switch
        // スイッチ制御
        if(state) {
            if (y == 1 && x == 1) SW_CTRL_1_SetHigh();
            else if (y == 1 && x == 2) SW_CTRL_2_SetHigh();
            else if (y == 1 && x == 3) SW_CTRL_5_SetHigh();
            else if (y == 1 && x == 4) SW_CTRL_6_SetHigh();
            else if (y == 2 && x == 1) SW_CTRL_3_SetHigh();
            else if (y == 2 && x == 2) SW_CTRL_4_SetHigh();
            else if (y == 2 && x == 3) SW_CTRL_7_SetHigh();
            else if (y == 2 && x == 4) SW_CTRL_8_SetHigh();
        }
        else {
            if (y == 1 && x == 1) SW_CTRL_1_SetLow();
            else if (y == 1 && x == 2) SW_CTRL_2_SetLow();
            else if (y == 1 && x == 3) SW_CTRL_5_SetLow();
            else if (y == 1 && x == 4) SW_CTRL_6_SetLow();
            else if (y == 2 && x == 1) SW_CTRL_3_SetLow();
            else if (y == 2 && x == 2) SW_CTRL_4_SetLow();
            else if (y == 2 && x == 3) SW_CTRL_7_SetLow();
            else if (y == 2 && x == 4) SW_CTRL_8_SetLow();
        }
    }
}

// ==========================================
//  ソフトウェアUART送信 (Bit-banging)
//  isVertical: trueなら縦(RC3), falseなら横(RC4)
// ==========================================
void SoftwareUART_WriteByte(uint8_t data, bool isVertical)
{
    INTERRUPT_GlobalInterruptDisable();

    // スタートビット (Low)
    if(isVertical) TX_V_SetLow(); else TX_H_SetLow();
    __delay_us(BIT_DELAY_US);

    // データビット (8bit, LSB First)
    for (int i = 0; i < 8; i++) {
        if (data & 0x01) {
            if(isVertical) TX_V_SetHigh(); else TX_H_SetHigh();
        } else {
            if(isVertical) TX_V_SetLow(); else TX_H_SetLow();
        }
        __delay_us(BIT_DELAY_US);
        data >>= 1;
    }

    // ストップビット (High)
    if(isVertical) TX_V_SetHigh(); else TX_H_SetHigh();
    __delay_us(BIT_DELAY_US);
    
    INTERRUPT_GlobalInterruptEnable();
    
    // ストップビットの安全マージン
    __delay_us(BIT_DELAY_US); 
}

void SoftwareUART_WriteString(const char* str, bool isVertical)
{
    while (*str) {
        SoftwareUART_WriteByte(*str, isVertical);
        str++;
    }
}

// ==========================================
//  ローカルスイッチ全オフ
// ==========================================
void TurnOffAllLocal(void)
{
    SW_CTRL_1_SetLow();
    SW_CTRL_2_SetLow();
    SW_CTRL_3_SetLow();
    SW_CTRL_4_SetLow();
    SW_CTRL_5_SetLow();
    SW_CTRL_6_SetLow();
    SW_CTRL_7_SetLow();
    SW_CTRL_8_SetLow();
}

void Initial_blink(void)
{
    SW_CTRL_1_Toggle();  
    SW_CTRL_2_Toggle();
    SW_CTRL_3_Toggle();
    SW_CTRL_4_Toggle();
    SW_CTRL_5_Toggle();
    SW_CTRL_6_Toggle();
    SW_CTRL_7_Toggle();
    SW_CTRL_8_Toggle();
}