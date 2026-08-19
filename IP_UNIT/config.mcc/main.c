/*
アナログPBXシステム用 IP接続ユニット(PBX側) PIC 16F18326
Raspberry Pi等のSBCとUART(ESUART)で接続して使用する
SBC側ではBaresipを動かしSIP呼制御等はそちらで行う
オーディオI/Fの入出力はスイッチボードに接続する

このユニットのインタフェースはSLICユニットの接続に準拠するが
IPユニット接続に対応したPBXコアが必要となる

MCCで以下を名前で設定すること
T1_IN
T2_IN
HO_OUT
RI_IN
LED1
LED2
LED3
またMCCでTimer2を1msTickとして使用するように設定すること
WDTは500mSより長い時間に設定のこと(オプション)

Baresipでダイヤルすべき番号はRI信号線のパルスで伝達される
(これによりインタフェースはSLICと同じでも番号が伝達できる)

LEDはState情報の表示用UnitStateで定義された値を表示する。LED1側がLSB

音声パスはPBXのスイッチボードを通すので、SBCのAUDIO IN/OUTをスイッチボード
へ接続すること。要するにSLICのAI/AOと同じように扱う
*/

#include "mcc_generated_files/system/system.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 状態定義
typedef enum {
    STATE_IDLE,
    STATE_RX_PULSES,       // PBXからRIピン経由でパルス受信中
    STATE_WAIT_IP_ANS,     // SBCへDIALを送り、ANS(応答)を待っている状態
    STATE_TX_PULSE_SETUP,  // SBCからRINGを受け、PBXへ発信(オフフック)した直後
    STATE_TX_PULSES,       // PBXへHOピン経由でパルス送信中
    STATE_TALKING          // 通話中
} UnitState;


UnitState current_state = STATE_IDLE;

// タイマー・カウンタ変数 (TMR2の1ms Tickで操作)
volatile uint16_t state_timer = 0;
volatile uint16_t pulse_timeout = 0;
volatile uint16_t pbx_busy_timer = 0;
// ハートビート用変数
volatile uint16_t sbc_link_timer = 0;
uint16_t led_blink_timer = 500;  // LEDは1tick(1ms)x500で点滅
bool is_link_up = false;

// PBX(RI_IN)からのパルス受信関連
uint8_t rx_pulse_count = 0;
uint8_t rx_digit_count = 0;
char rx_number[16];
bool last_ri_state = true; 

// PBX(HO_OUT)へのパルス送信関連
char tx_number[16];
uint8_t tx_digit_idx = 0;
uint8_t tx_pulse_count = 0;
uint8_t tx_pulse_phase = 0; // 0:Digit間ポーズ, 1:Pulse Low(Break), 2:Pulse High(Make)

// UARTコマンド受信バッファ
#define UART_BUF_SIZE 32
char uart_rx_buf[UART_BUF_SIZE];
uint8_t uart_rx_idx = 0;

// プロトタイプ宣言
void ProcessStateMachine(void);
void ProcessUARTCommand(char *cmd);
void StatusLEDControl(uint8_t num);

//LEDによるステータス表示
void StatusLEDControl(uint8_t num){
    // ハートビートでリンクが確認できた場合だけLEDを更新
    if(is_link_up){
        LED1_SetLow();
        LED2_SetLow();
        LED3_SetLow();
    
        switch(num){
            case 0x00:
                break;
            case 0x01:
                LED1_SetHigh();
                break;
            case 0x02:
                LED2_SetHigh();
                break;
            case 0x03:
                LED1_SetHigh();
                LED2_SetHigh();
                break;
            case 0x04:
                LED3_SetHigh();
                break;
            case 0x05:
                LED1_SetHigh();
                LED3_SetHigh();
                break;
            case 0x06:
                LED2_SetHigh();
                LED3_SetHigh();
                break;
            case 0x07:
                LED1_SetHigh();
                LED2_SetHigh();
                LED3_SetHigh();
                break;
            default:
                LED1_SetLow();
                LED2_SetLow();
                LED3_SetLow();
        }
    }
}

// 1ms Tick割り込みハンドラ (TMR2から呼び出し)
void SystemTick_1ms(void) {
    if (state_timer > 0) state_timer--;
    if (pulse_timeout > 0) pulse_timeout--;
    
    // ハートビート・タイムアウト処理
    if (sbc_link_timer > 0) {
        sbc_link_timer--;
        if (sbc_link_timer == 0) {
            is_link_up = false; // 15秒間PINGが来なければリンクダウン
        }
    }

    // リンクダウン中のLED点滅
    if(!is_link_up){
        led_blink_timer--;
        if(led_blink_timer == 0){
            LED3_Toggle();
            led_blink_timer = 500;
        }
    }

    // PBXがビジートーン(T1=L, T2=H)を出しているかの検知
    // PBX側の HAL_SetTone(ch, TONE_BUSY) は TG1=L, TG2=H になる
    if (T1_IN_GetValue() == 0 && T2_IN_GetValue() == 1) {
        if (pbx_busy_timer < 65000) pbx_busy_timer++;
    } else {
        pbx_busy_timer = 0;
    }
}

int main(void)
{
    SYSTEM_Initialize();
    
    printf("\r\nIP Unit Started.\r\n");
    HO_OUT_SetLow(); // オンフック(切断)状態で開始

    // ゴーストダイヤル防止. 初期起動時は少し待つ(10秒)
    // WDTと干渉しないように注意(WDT must > 500ms:後のフック処理に注意)
    printf("Waiting...\r\n");
    for(uint16_t i = 0; i < 100; i++){
        LED1_Toggle();
        LED2_Toggle();
        LED3_Toggle();
        __delay_ms(100);
        CLRWDT();
    }

    // 電源立ち上がりタイミングによりHO_OUTがPBX側でHighにみえた場合の対処用
    // PBX側のHOOK信号監視によるUnavail復帰用
    // HO_OUTはLowがオンフック、Highがオフフック
    // 注: PBX起動時間よりも「後」に実行されること
    // WDT must > 500ms
    StatusLEDControl(0x07);
    CLRWDT();
    HO_OUT_SetHigh();
    __delay_ms(500);
    CLRWDT();
    HO_OUT_SetLow();
    __delay_ms(500);
    HO_OUT_SetHigh();
    CLRWDT();
    __delay_ms(500);
    HO_OUT_SetLow();

    // Timer2のコールバック設定
    TMR2_OverflowCallbackRegister(SystemTick_1ms);
    
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    printf("Ready.\r\n");
    StatusLEDControl(0x00);

    //エッジ検出の初期化
    last_ri_state = RI_IN_GetValue();

    while(1)
    {
        CLRWDT();

        // 1. UART受信処理 (SBCからのコマンド受け取り)
        if (EUSART_IsRxReady()) {
            char c = EUSART_Read();
            if (c == '\r' || c == '\n') {
                if (uart_rx_idx > 0) {
                    uart_rx_buf[uart_rx_idx] = '\0';
                    ProcessUARTCommand(uart_rx_buf);
                    uart_rx_idx = 0;
                }
            } else {
                if (uart_rx_idx < UART_BUF_SIZE - 1) {
                    uart_rx_buf[uart_rx_idx++] = c;
                }
            }
        }
        
        // 2. ステートマシン処理
        ProcessStateMachine();

    }
}

void ProcessStateMachine(void) {
    // 現在のピン状態を取得 (RI_INはアクティブLow前提)
    bool current_ri = T1_IN_GetValue(); // ※PBXからのRI信号(要結線確認)
    current_ri = RI_IN_GetValue();      // 正しくはRIピン

    // PBXからの強制切断（BUSY）検知 (100ms以上継続で確定)
    if (current_state != STATE_IDLE && pbx_busy_timer > 100) {
        printf("DROP (Reason: PBX BUSY detected)\r\n"); // SBCに切断を通知
        HO_OUT_SetLow();    // オンフック
        current_state = STATE_IDLE;
        return;
    }

    StatusLEDControl(current_state);

    switch (current_state) {
        case STATE_IDLE:
            // PBXからRIの立ち下がり(パルス開始)を検知したら受信モードへ
            if (last_ri_state == true && current_ri == false) {
                rx_pulse_count = 1;
                rx_digit_count = 0;
                memset(rx_number, 0, sizeof(rx_number));
                pulse_timeout = 100; // 桁確定タイマー
                current_state = STATE_RX_PULSES;
            }
            break;

        case STATE_RX_PULSES:
            // パルスの立ち下がりカウント
            if (last_ri_state == true && current_ri == false) {
                rx_pulse_count++;
                pulse_timeout = 100; // パルスが来るたびにリセット
            }
            
            // パルスが途絶えたら1桁確定
            if (current_ri == true && pulse_timeout == 0 && rx_pulse_count > 0) {
                uint8_t digit = (rx_pulse_count == 10) ? 0 : rx_pulse_count;
                rx_number[rx_digit_count++] = digit + '0';
                rx_pulse_count = 0;
                
                // 次の桁を待つロングタイマー (PBX側は300ms間隔で送ってくるので500ms待つ)
                state_timer = 500; 
            }
            
            // 桁間ポーズがタイムアウトしたら全桁受信完了とみなしてSBCへ送信
            if (current_ri == true && pulse_timeout == 0 && rx_pulse_count == 0) {
                if (state_timer == 0 && rx_digit_count > 0) {
                    if (is_link_up) { // ハートビートでリンクupの状態ならDIAL送出
                        printf("DIAL %s\r\n", rx_number);
                        current_state = STATE_WAIT_IP_ANS;
                    } else {
                        // リンクダウン時は一瞬応答してすぐ切る(BUSYにする)
                        // printf("Link DOWN! Rejecting call from PBX.\r\n");
                        HO_OUT_SetHigh();
                        __delay_ms(100);
                        HO_OUT_SetLow();
                        current_state = STATE_IDLE;
                    }
                }
            }
            break;

        case STATE_WAIT_IP_ANS:
            // SBCからの "ANS" コマンド待ち。
            // ProcessUARTCommand内で処理するため、ここではタイムアウトなどのみ監視
            if (!is_link_up) { // 待っている間にダウンした場合の処理
                // printf("Link DOWN during WAIT_ANS! Aborting.\r\n");
                HO_OUT_SetHigh();
                __delay_ms(100);
                HO_OUT_SetLow();
                current_state = STATE_IDLE;
            }
            break;

        case STATE_TX_PULSE_SETUP:
            // SBCからRINGを受け、HO_OUTをHにしてPBXのダイヤルトーンを待つ
            if (state_timer == 0) {
                tx_digit_idx = 0;
                tx_pulse_phase = 0;
                state_timer = 100; // 最初だけ少しディレイ
                current_state = STATE_TX_PULSES;
            }
            break;

        case STATE_TX_PULSES:
            if (state_timer == 0) {
                if (tx_pulse_phase == 0) { // 次の桁の準備
                    if (tx_number[tx_digit_idx] != '\0') {
                        uint8_t d = tx_number[tx_digit_idx] - '0';
                        tx_pulse_count = (d == 0) ? 10 : d;
                        tx_pulse_phase = 1;
                    } else {
                        // 全桁送信完了 -> PBXが呼び出しを開始するのでTALKINGへ
                        HO_OUT_SetHigh(); // 確実にオフフック維持
                        current_state = STATE_TALKING;
                    }
                }
                
                if (tx_pulse_phase == 1) { // Pulse LOW (Break: 33ms)
                    HO_OUT_SetLow();
                    state_timer = 33;
                    tx_pulse_phase = 2;
                } 
                else if (tx_pulse_phase == 2) { // Pulse HIGH (Make: 67ms)
                    HO_OUT_SetHigh();
                    tx_pulse_count--;
                    if (tx_pulse_count > 0) {
                        state_timer = 67;
                        tx_pulse_phase = 1;
                    } else {
                        state_timer = 800; // 桁間ポーズ(PBXの600ms以上にする)
                        tx_pulse_phase = 0;
                        tx_digit_idx++;
                    }
                }
            }
            break;

        case STATE_TALKING:
            // 通話中は何もしない。PBXからのBUSY検知、またはSBCからのDROPコマンドを待つ。
            break;
    }
    
    last_ri_state = current_ri;
}

// SBC(Python)からのコマンド処理
void ProcessUARTCommand(char *cmd) {
    // PING-PONGハートビートの応答
    if (strcmp(cmd, "PING") == 0) {
        sbc_link_timer = 15000; // タイマーを15秒にリセット
        is_link_up = true;
        printf("PONG\r\n");     // PONGを即座に返す
        return;
    }

    if (strncmp(cmd, "RING ", 5) == 0) {
        if (current_state == STATE_IDLE) {
            strncpy(tx_number, cmd + 5, sizeof(tx_number) - 1);
            HO_OUT_SetHigh(); // オフフック
            state_timer = 600; // PBXがダイヤルトーンを出すまでの猶予時間
            current_state = STATE_TX_PULSE_SETUP;
        }
    } 
    else if (strcmp(cmd, "ANS") == 0) {
        if (current_state == STATE_WAIT_IP_ANS) {
            HO_OUT_SetHigh(); // オフフックして通話を確立
            current_state = STATE_TALKING;
        }
    } 
    else if (strcmp(cmd, "DROP") == 0) {
        HO_OUT_SetLow(); // オンフック(切断)
        current_state = STATE_IDLE;
    }
}