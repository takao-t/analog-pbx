/*
 * File:   main.c
 * Device: PIC16F18326
 * Description: SLIC Controller with DTMF decoder.
 */

#include "mcc_generated_files/adc/adc.h"
#include "mcc_generated_files/clc/clc1.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/system/system.h"
#include <pic16f18326.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>


// MCCによる各種設定(参考)
// クロックはHFINTOSC 32MHz
// 各種タイマー
//  TMR6 : 1.5ms デバウンス(CLC)用
//  TMR5 : 1ms ダイヤルパルス生成用
//  TMR3 : 25mSメインループ処理用ペースメーカー
//  TMR2 : 125us ADC用
// トーン源
//  NCO1 : 400Hz トーンジェネレータ
// デバウンス処理
//  CLC1,3 : デバウンス処理(HOOK_OUTはCLC1の出力)
//  電話機からのフック信号はプログラムでは処理せず直接CLCから出力
// Watchdog有効 : 実行サイクル25mSを阻害しないように設定のこと

// DTMF解析処理用
//  FVR=2.048V
//  FVR->DACで1/2 FVRを出力しADCの入力バイアスに使う
//  (SLICのVoutで観測されるDTMF信号が+-1V未満のため)
//  DAC Out : RA0
//  ADC In : RA1
// *ICSPは作動時には外す

// ================
// 各種ピン処理定義
// ================
#define TG1_IN      TG1_IN_GetValue()
#define TG2_IN      TG2_IN_GetValue()
#define RING_CMD    RING_CMD_GetValue()   
#define REV_CMD     REV_CMD_GetValue()
#define TONE_PIN_OUTPUT()   TONE_PIN_SetDigitalOutput()
#define TONE_PIN_HIZ()      TONE_PIN_SetDigitalInput()
#define HOOK_OUT_CHECK()    CLC1_OutputStatusGet()

// ケイデンス設定(単位ミリ秒:メイン処理の1フレーム=25mS)
// ACTIVE: 鳴動時間
// DEACTIVE: 鳴動+休止時間
// 日本: 1秒鳴動2秒休止
#define CADENCE_ACTIVE 40
#define CADENCE_DEACTIVE 120

// DTMF検出閾値
#define DTMF_THRESHOLD 100000 
#define DEBOUNCE_COUNT 2

// UARTバッファサイズ
#define CMDBUFSIZE  16

// ==========================================
// EEPROM メモリアドレスマップとデフォルト値
// ==========================================
typedef enum {
    EE_ADDR_MAGIC = 0,   // 0x00: 初期化済み判定用マジックナンバー
    EE_ADDR_PPS,         // 0x01: PPS設定 (10 or 20)
    EE_ADDR_SER_MODE,    // 0x02: シリアルモード (0:OFF, 1:ON)
    EE_ADDR_TALK_TIME,   // 0x03: 通話中判定時間 (秒単位)
    // 今後新しい設定が増えたらここに追加していく
    EE_MAX_ADDR          // 常に最後のアドレス+1になる
} EEPROM_MAP_t;

#define EEPROM_MAGIC_VAL 0xAA // 初期化済み値
#define DEFAULT_PPS 10 // PPSのデフォルト値

//__eeprom uint8_t pps_data[3] = {PPS_MAGIC, DEFAULT_PPS, 0};

// =========================
// Goertzel用グローバル変数
// =========================
// 8つの周波数の過去2回分のサンプルを保持
volatile int16_t s1[8] = {0}; 
volatile int16_t s2[8] = {0};
volatile int16_t s1_res[8] = {0}; 
volatile int16_t s2_res[8] = {0};
volatile uint8_t sample_count = 0;
volatile bool dtmf_ready = false; // メインループへの通知フラグ

// ==========================================
// ダイヤルパルス(DP)制御用 定義と変数
// ==========================================
typedef enum {
    DP_IDLE,       // 待機中
    DP_BREAK,      // ブレイク(開放)出力中
    DP_MAKE,       // メイク(閉結)出力中
    DP_PAUSE       // 桁間のポーズ(休止)中
} DP_STATE_t;

// 状態管理変数
DP_STATE_t dp_state = DP_IDLE;
uint8_t dp_pulses_remaining = 0;
uint16_t dp_frame_timer = 0;
// DP送出中フラグ
volatile bool is_dp_active = false;

// パルス設定用のグローバル変数 (単位: ms)
uint16_t dp_time_break;
uint16_t dp_time_make;
uint16_t dp_time_pause = 600; // ポーズは固定でもOK

// DP受信(デコード)用変数
volatile uint8_t rx_pulse_count = 0; // 受信したパルス数
volatile uint8_t rx_dp_timeout = 0;  // タイムアウトカウンタ(25ms単位)

// フック状態検出(シリアル送出用)
#define HOOK_DETECT_TIME 5 //5フレーム(5x25ms=125ms)
static bool logical_hook_state = false; // 現在確定しているフック状態
static uint8_t hook_change_timer = 0;   // 状態変化の継続タイマー

// CLCの極性反転によるフック制御(DP)マクロ
// フック信号はCLCだけで処理しているため出力段の反転でDP信号にする
// CLCの出力段を反転させることでパルスを生成する
#define DP_SET_BREAK()  (LC1POL = 1) // ループ開放 (パルス発生)
#define DP_SET_MAKE()   (LC1POL = 0) // ループ閉結 (通常時)

// UART用変数
// グローバル変数
char rxBuffer[CMDBUFSIZE];
uint8_t rxIndex = 0;
// シリアルコマンドでの変数
volatile uint8_t ser_tone_mode = 3;
volatile bool ser_ring_cmd = false;
volatile bool ser_rev_cmd = false;
volatile bool enable_serial_transmit = false;
volatile bool tg1_overwrite = false;
volatile bool dp_out_enable = true;

// 通話状態判定用変数
volatile uint16_t talk_threshold = 1200; //デフォルトは30秒

// ===========================================================
// ADC割り込みハンドラ (125µsごとにハードウェアトリガで実行される)
// ===========================================================
void Custom_ADC_Interrupt_Handler(void) {

    // 1. ADC値の取得とDCバイアスの除去
    int16_t x_in = (int16_t)ADRES - 512;
    
    // 2. 16bitオーバーフロー防止のためのスケーリング(1/4)
    x_in = x_in >> 2;

    int16_t s_new;
    int16_t t1, t2, t3, t4;

    // ==========================================
    // 3. 各周波数のGoertzel計算 (ループ展開＆シフト演算)
    // s_new = x_in + (coeff * s1) - s2
    // ==========================================

    // [0] 697 Hz: coeff = 1.7034 -> 1 + 1/2 + 1/8 + 1/16 + 1/64
    t1 = s1[0] >> 1;     // 1/2
    t2 = t1 >> 2;        // 1/8
    t3 = t2 >> 1;        // 1/16
    t4 = t3 >> 2;        // 1/64
    s_new = x_in + s1[0] + t1 + t2 + t3 + t4 - s2[0];
    s2[0] = s1[0]; s1[0] = s_new;

    // [1] 770 Hz: coeff = 1.6358 -> 1 + 1/2 + 1/8 + 1/128 + 1/256
    t1 = s1[1] >> 1;     // 1/2
    t2 = t1 >> 2;        // 1/8
    t3 = t2 >> 4;        // 1/128
    t4 = t3 >> 1;        // 1/256
    s_new = x_in + s1[1] + t1 + t2 + t3 + t4 - s2[1];
    s2[1] = s1[1]; s1[1] = s_new;

    // [2] 852 Hz: coeff = 1.5624 -> 1 + 1/2 + 1/16
    t1 = s1[2] >> 1;     // 1/2
    t2 = t1 >> 3;        // 1/16
    s_new = x_in + s1[2] + t1 + t2 - s2[2];
    s2[2] = s1[2]; s1[2] = s_new;

    // [3] 941 Hz: coeff = 1.4828 -> 1 + 1/2 - 1/64
    t1 = s1[3] >> 1;     // 1/2
    t2 = t1 >> 5;        // 1/64
    s_new = x_in + s1[3] + t1 - t2 - s2[3];
    s2[3] = s1[3]; s1[3] = s_new;

    // [4] 1209 Hz: coeff = 1.1632 -> 1 + 1/8 + 1/32 + 1/128
    t1 = s1[4] >> 3;     // 1/8
    t2 = t1 >> 2;        // 1/32
    t3 = t2 >> 2;        // 1/128
    s_new = x_in + s1[4] + t1 + t2 + t3 - s2[4];
    s2[4] = s1[4]; s1[4] = s_new;

    // [5] 1336 Hz: coeff = 1.0084 -> 1 + 1/128
    t1 = s1[5] >> 7;     // 1/128
    s_new = x_in + s1[5] + t1 - s2[5];
    s2[5] = s1[5]; s1[5] = s_new;

    // [6] 1477 Hz: coeff = 0.7904 -> 1/2 + 1/4 + 1/32 + 1/128
    t1 = s1[6] >> 1;     // 1/2
    t2 = t1 >> 1;        // 1/4
    t3 = t2 >> 3;        // 1/32
    t4 = t3 >> 2;        // 1/128
    s_new = x_in + t1 + t2 + t3 + t4 - s2[6];
    s2[6] = s1[6]; s1[6] = s_new;

    // [7] 1633 Hz: coeff = 0.5602 -> 1/2 + 1/16 - 1/256
    t1 = s1[7] >> 1;     // 1/2
    t2 = t1 >> 3;        // 1/16
    t3 = t2 >> 4;        // 1/256
    s_new = x_in + t1 + t2 - t3 - s2[7];
    s2[7] = s1[7]; s1[7] = s_new;

    // ==========================================
    // 4. サンプル数のカウント
    // ==========================================
    sample_count++;
    if (sample_count >= 205) {
        // 結果をメインループ用バッファに退避し、次回の計算のために初期化
        for(uint8_t i = 0; i < 8; i++){
            s1_res[i] = s1[i];
            s2_res[i] = s2[i];
            s1[i] = 0;
            s2[i] = 0;
        }
        sample_count = 0;
        dtmf_ready = true; // メインループへ通知
    }
}

// ========================
// ダイヤルパルス生成関連処理
// ========================

// PPS切り替え関数 (システム起動時や設定変更時に呼ぶ)
void Set_DialPulse_PPS(uint8_t pps) {
    if (pps == 20) {
        // 20 PPS (周期50ms) の設定例: Break 33ms, Make 17ms (約66%:33%)
        dp_time_break = 33;
        dp_time_make  = 17;
    } else {
        // 10 PPS (周期100ms) の設定例: Break 67ms, Make 33ms
        dp_time_break = 67;
        dp_time_make  = 33;
    }
}

// ==========================================
// パルス送出開始関数 (DTMFデコード確定時に呼ぶ)
// ==========================================
void Start_DialPulse(char digit) {

    // ダイアルパルス出力無効なら何もしない
    if(dp_out_enable == false) return;

    // 数字チェック
    if (digit >= '1' && digit <= '9') {
        dp_pulses_remaining = digit - '0';
    } else if (digit == '0') {
        dp_pulses_remaining = 10;
    } else {
        return; 
    }

    // 割り込み競合を防ぐため、is_dp_activeがfalseであることを確認
    if (is_dp_active) return; 

    // 変数の初期化
    dp_state = DP_BREAK;
    dp_frame_timer = dp_time_break; // 変数からロード
    DP_SET_BREAK();

    // 準備が全て整ってから最後にフラグを立てる！
    is_dp_active = true;
}

// =====================================================
// パルス送出ステートマシン(タイマーにより1ms間隔で呼ばれる)
// =====================================================
void Process_DialPulse_StateMachine(void) {
    if (!is_dp_active) return;

    switch (dp_state) {
        case DP_BREAK:
            dp_frame_timer--;
            if (dp_frame_timer == 0) {
                dp_state = DP_MAKE;
                dp_frame_timer = dp_time_make; // 変数からロード
                DP_SET_MAKE();
            }
            break;

        case DP_MAKE:
            dp_frame_timer--;
            if (dp_frame_timer == 0) {
                dp_pulses_remaining--;
                if (dp_pulses_remaining > 0) {
                    dp_state = DP_BREAK;
                    dp_frame_timer = dp_time_break; // 変数からロード
                    DP_SET_BREAK();
                } else {
                    dp_state = DP_PAUSE;
                    dp_frame_timer = dp_time_pause; // 変数からロード
                }
            }
            break;

        case DP_PAUSE:
            dp_frame_timer--;
            if (dp_frame_timer == 0) {
                is_dp_active = false;
                //パルス送出完了確認音
                TONE_PIN_OUTPUT();
                __delay_ms(50);
                TONE_PIN_HIZ();
                __delay_ms(20);
                TONE_PIN_OUTPUT();
                __delay_ms(50);
                TONE_PIN_HIZ();
                dp_state = DP_IDLE;
            }
            break;
            
        case DP_IDLE:
        default:
            break;
    }
}

// 文字列送出ルーチン
// 注: EUSARTは割り込み有でバッファ16byteなので長い文字列は送らないこと
void Transmit_String(const char* str){
    if(enable_serial_transmit == false) return;
    uint8_t i = 0;
    while(*str){
        EUSART_Write(*str);
        str++;
    }
    EUSART_Write(0x0d);
    EUSART_Write(0x0a);
}

// ==========================================
// EEPROM アクセス関数群
// ==========================================

// 値が変わっている場合のみ書き込む（EEPROMの寿命対策）
void Config_Save(EEPROM_MAP_t addr, uint8_t value) {
    if (EEPROM_READ(addr) != value) {
        EEPROM_WRITE(addr, value);
    }
}

// 起動時の設定読み込みと未初期化時のデフォルト値書き込み
void Config_Load(void) {
    // 初回起動（マジックナンバーが違う）の場合は初期化
    if (EEPROM_READ(EE_ADDR_MAGIC) != EEPROM_MAGIC_VAL) {
        EEPROM_WRITE(EE_ADDR_MAGIC, EEPROM_MAGIC_VAL);
        Config_Save(EE_ADDR_PPS, DEFAULT_PPS);
        Config_Save(EE_ADDR_SER_MODE, 0);
        Config_Save(EE_ADDR_TALK_TIME, 30);
    }

    // 1. PPS設定の反映
    uint8_t pps = EEPROM_READ(EE_ADDR_PPS);
    if (pps == 10 || pps == 20) {
        Set_DialPulse_PPS(pps);
    } else {
        Set_DialPulse_PPS(DEFAULT_PPS);
    }

    // 2. シリアルモード設定の反映
    enable_serial_transmit = (EEPROM_READ(EE_ADDR_SER_MODE) == 1);
    tg1_overwrite = enable_serial_transmit; // 既存コードのロジックを維持

    // 3. 通話中判定時間の設定（秒からフレーム数へ変換）
    talk_threshold = EEPROM_READ(EE_ADDR_TALK_TIME) * 40; 
}

// EEPROMの強制初期化とデフォルト読み込み
void Config_Reset(void) {
    // 0. EEPROMを強制上書き
    EEPROM_WRITE(EE_ADDR_MAGIC, EEPROM_MAGIC_VAL);
    Config_Save(EE_ADDR_PPS, DEFAULT_PPS);
    Config_Save(EE_ADDR_SER_MODE, 0);
    Config_Save(EE_ADDR_TALK_TIME, 30);

    // 1. PPS設定の反映
    uint8_t pps = EEPROM_READ(EE_ADDR_PPS);
    if (pps == 10 || pps == 20) {
        Set_DialPulse_PPS(pps);
    } else {
        Set_DialPulse_PPS(DEFAULT_PPS);
    }

    // 2. シリアルモード設定の反映
    enable_serial_transmit = (EEPROM_READ(EE_ADDR_SER_MODE) == 1);
    tg1_overwrite = enable_serial_transmit;

    // 3. 通話中判定時間の設定（秒からフレーム数へ変換）
    talk_threshold = EEPROM_READ(EE_ADDR_TALK_TIME) * 40; 
}


// ======================================
//  シリアルコマンド解析・変数設定・実行処理
// ======================================
void ProcessCommand(char* cmd)
{
    uint8_t ok_flag = 0;

    if(strcmp(cmd, "RST") == 0){ // コマンド系リセット
        ser_tone_mode = 3;
        ser_ring_cmd = false;
        ser_rev_cmd = false;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "-TO") == 0){ // トーンジェネレータ停止
        ser_tone_mode = 3;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+CT") == 0){ // コーリングトーン生成
        ser_tone_mode = 0;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+BT") == 0){ // ビジートーン生成
        ser_tone_mode = 1;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+RB") == 0){ // リングバックトーン生成
        ser_tone_mode = 2;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+RN") == 0){ // 鳴動モード(電話機鳴動)
        ser_ring_cmd = true;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "-RN") == 0){ // 鳴動モード停止
        ser_ring_cmd = false;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+RV") == 0){ // 回線リバース
        ser_rev_cmd = true;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "-RV") == 0){ // 回線ノーマル
        ser_rev_cmd = false;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+DP") == 0){ // ダイヤルパルス出力有効
        dp_out_enable = true;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "-DP") == 0){ // ダイヤルパルス出力無効
        dp_out_enable = false;
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+PP1") == 0){ // DTMF-DP変換を10PPS
        Set_DialPulse_PPS(10);
        Config_Save(EE_ADDR_PPS, 10);
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+PP2") == 0){ // DTMF-DP変換を20PPS
        Set_DialPulse_PPS(20);
        Config_Save(EE_ADDR_PPS, 20);
        ok_flag = 1;
    }
    else if(strcmp(cmd, "+SER") == 0){ // シリアルモード(アンサバック有効)
        //送信バッファをリセット
        EUSART_TransmitDisable();
        EUSART_TransmitEnable();
        enable_serial_transmit = true;
        ok_flag = 1;
        tg1_overwrite = true;
        Config_Save(EE_ADDR_SER_MODE, 1);
    }
    else if(strcmp(cmd, "-SER") == 0){ // シリアルモード停止(アンサバック無効)
        enable_serial_transmit = false;
        ok_flag = 1;
        tg1_overwrite = false;
        Config_Save(EE_ADDR_SER_MODE, 0);
    }
    else if(strcmp(cmd, "?HO") == 0){ //フック状態問い合わせ
        if(logical_hook_state){
            Transmit_String("HOOK:OFF");
            ok_flag = 1;
        }
        else{
             Transmit_String("HOOK:ON");
             ok_flag = 1;
        }
    }
    else if(strncmp(cmd, "+TM", 3) == 0){ // 通話中判定時間の設定 (+TM10 など)
        uint8_t sec = (uint8_t) atoi(&cmd[3]);      // 3文字目以降を数値化
        if(sec > 0){
            talk_threshold = sec * 40;    // 1秒 = 40フレーム(25ms)
            Config_Save(EE_ADDR_TALK_TIME, sec);
            ok_flag = 1;
        }
    }
    else if(strncmp(cmd, "?ST1", 4) == 0){ // 設定情報出力
        char tmp_stat_str[16];
        char tmp_dp_str[4];
        uint8_t tmp_pps;
        //PPSはPPS自体を変数に持っていないのでブレーク時間で判断
        if(dp_time_break == 67){
            tmp_pps = 10;
        }
        else if(dp_time_break == 33){
            tmp_pps = 20;
        }
        else {
            tmp_pps = 0; // Something Wrong status
        }

        // ダイヤルパルス出力有無
        if(dp_out_enable == true){
            tmp_dp_str[0] = 'D';
            tmp_dp_str[1] = 'P';
            tmp_dp_str[2] = 0x00;
        }
        else{
            tmp_dp_str[0] = '-';
            tmp_dp_str[1] = '-';
            tmp_dp_str[2] = 0x00;
        }
        sprintf(tmp_stat_str, "ST1:%d,%s,%d", tmp_pps, tmp_dp_str, talk_threshold /40);
        Transmit_String(tmp_stat_str);
        ok_flag = 1;
        
    }
    else if(strncmp(cmd, "?ST2", 4) == 0){ // 動作情報出力
        char tmp_stat_str[16];
        char tmp_ring_str[4];

        tmp_ring_str[1] = 0x00;
        if(ser_ring_cmd == true) tmp_ring_str[0] = 'R';
        else tmp_ring_str[0] = '-';

        sprintf(tmp_stat_str, "ST2:%d,%s", ser_tone_mode, tmp_ring_str);
        Transmit_String(tmp_stat_str);
        ok_flag = 1;       
    }
    else if(strcmp(cmd, "++FRST") == 0){ //フルリセット EEPROMも初期化する 
        Config_Reset();
        ok_flag = 1;
    }

    if(ok_flag == 1){
        Transmit_String("OK");
    }
    else{
        Transmit_String("ERR");
    }
}

// CLC割り込みでダイヤルパルス数をカウントさせる
void Dial_Pulse_Count(void)
{
    // パルスをカウントアップ
    rx_pulse_count++;
    
    // タイムアウトカウンタをリセット (25ms x 20 = 500ms)
    // 500ms経過しても次のパルスが来なければ確定とする
    rx_dp_timeout = 20;
}

// ====================
// メインアプリケーション
// ====================
int main(void)
{
    SYSTEM_Initialize();

    // ADCでの割り込みハンドラを登録
    ADC_ConversionDoneCallbackRegister(Custom_ADC_Interrupt_Handler);
    // デフォルトPPS設定
    Set_DialPulse_PPS(DEFAULT_PPS);
    // パルス生成用処理を1ms(TMR)で実行させる
    TMR5_OverflowCallbackRegister(Process_DialPulse_StateMachine);
    // パルス数カウントのためのCLC1の割り込みハンドラを登録
    CLC1_CLCI_SetInterruptHandler(Dial_Pulse_Count);

    // ADC変換と各種タイマー、ピン割り込み処理のために割り込みを許可
    INTERRUPT_GlobalInterruptEnable(); 
    INTERRUPT_PeripheralInterruptEnable(); 

    // main内変数
    bool tone_mod_phase = false;
    uint16_t busy_timer = 0;
    bool busy_phase = false;
    uint16_t cadence_cycle = 0;    
    uint8_t ring_period = 0;
    uint8_t last_ring_cmd = 1;
    uint8_t last_tg2 = 1;

    // デバウンスと押された文字判定用
    static char last_detected_char = 0;
    static uint8_t char_match_count = 0;
    static bool key_is_pressed = false;
    static uint8_t release_count = 0;
    // REVコマンド(pin)のカウント
    uint8_t rev_cmd_count = 3;
    // PPS初期値読み出し用
    uint8_t initial_pps = 10;

    // 通話中検出(時間処理)用変数
    uint16_t offhook_timer = 0;
    bool is_talking = false;
   
    // 起動時設定をEEPROMから取得
    Config_Load();

    // ================================================
    // DAC初期化:ADCバイアス用DC出力
    // ADCのVrefはFVRでつくる(2.048V)
    // DACの+refはFVRにする
    // ADCはDCバイアスが必要になるので1/2FVRをDACでつくる
    // ================================================
    DAC1_SetOutput(16);

    // =====================================
    // メインループ(25msフレーム)
    // 25msはTMR3で生成
    // =====================================
    while (1) {
        // 処理は25msフレーム内に入れる
        // フレーム内の処理で時間を意識する数値は25mS × nで計算すること
        if (TMR3_OverflowStatusGet()) {
            TMR3_OverflowStatusClear();
            TMR3_Reload();

            // ウォッチドッグクリア
            // WDTの設定は25msの実行を阻害しないこと
            CLRWDT();

            // ==============================================
            // 受信ダイヤルパルス(DP)のデコードとタイムアウト処理
            // シリアルモード時にシリアルでダイヤルした番号を送出
            // ==============================================
            if(enable_serial_transmit){ // シリアルモードでなければ送信の必要はない
                if (rx_dp_timeout > 0) {
                    rx_dp_timeout--; // 25msごとに減算
                
                    if (rx_dp_timeout == 0) {
                        // タイムアウト発生！(= パルスの連続が途切れた)
                        if (rx_pulse_count > 0) {
                        
                            // フック状態をチェックして切り分け
                            // HOOK_OUT_CHECK() が true(=オフフック/Make) ならダイヤル完了
                            // false(=オンフック/Break) のままなら、単に受話器を置いた(電話を切った)だけ
                            if (HOOK_OUT_CHECK() == true) { 
                            
                                char decoded_digit;
                                if (rx_pulse_count >= 10) {
                                    decoded_digit = '0'; // 10パルスは '0'
                                } else {
                                    decoded_digit = '0' + rx_pulse_count; // 1〜9
                                }
                            
                                // 取得した番号をシリアル出力
                                char d_str[8] = "R_DP:X";
                                d_str[5] = decoded_digit;
                                Transmit_String(d_str);
                            }
                        
                            // 処理が終わったら（あるいは電話を切っただけなら）カウントをリセット
                            rx_pulse_count = 0;
                        }
                    }
                }
            } //シリアルでのDP送出処理ここまで

            // ==========================================
            // フック状態(受話器の上げ/下げ)の確定とシリアル通知
            // (指定秒間の継続を確認してエッジを検出)
            // ==========================================
            // 現在の物理的なフック状態を取得 (true:オフフック, false:オンフック)
            bool current_physical_hook = HOOK_OUT_CHECK();

            // 確定している状態と、現在の物理状態が違っている場合
            if (current_physical_hook != logical_hook_state) {
                hook_change_timer++;
                
                // HOOK＿DETECT_TIMEフレーム分継続したか？
                if (hook_change_timer >= HOOK_DETECT_TIME) {
                    // 指定時間継続したので状態遷移を確定
                    logical_hook_state = current_physical_hook;
                    hook_change_timer = 0; // 次回の変化に備えてリセット
                    
                    // シリアルへ通知 (1回だけ送出される)
                    if (logical_hook_state == true) {
                        Transmit_String("HOOK:OFF"); // 受話器を上げた (オフフック)
                        // オフフックしたらAD変換と割り込みを有効に
                        ADC_Enable();
                        ADC_ConversionDoneInterruptEnable();
                        // 通話中タイマリセット
                        offhook_timer = 0;
                        is_talking = false;
                    } else {
                        Transmit_String("HOOK:ON");  // 受話器を置いた (オンフック)
                        // オンフック時にAD変換させない(ノイズによる誤動作防止)
                        ADC_ConversionDoneInterruptDisable();
                        ADC_Disable();
                        // 通話中タイマリセット
                        offhook_timer = 0;
                        is_talking = false;
                    }
                }
            } else {
                // 指定時間経つ前に元の状態に戻った (ダイヤルパルスやノイズ)
                // タイマーをリセットしてカウントをやり直す
                hook_change_timer = 0;
            }

            // オフフック継続時間のカウントと通話状態への移行
            if(logical_hook_state == true){
                if(!is_talking){
                    offhook_timer++;
                    if(offhook_timer >= talk_threshold){
                        is_talking = true;
                        Transmit_String("TALK:ON");
                    }
                }
            }


            // DP出力中はDTMFデコードしない
            if(is_dp_active == false){
                // DTMFデコード処理
                if(dtmf_ready){
                    dtmf_ready = false; // フラグを下ろす

                    uint32_t power[8];
                
                    // 1. 各周波数のパワーを計算
                    for (int i = 0; i < 8; i++) {
                        // s1_res, s2_resは最大数千になるため、2乗すると32bitが必要
                        int32_t sq1 = (int32_t)s1_res[i] * s1_res[i];
                        int32_t sq2 = (int32_t)s2_res[i] * s2_res[i];
                        int32_t M   = (int32_t)s1_res[i] * s2_res[i];
                        int32_t coeff_M = 0;

                        // coeff * s1 * s2 の部分をシフト展開
                        switch(i) {
                            case 0: coeff_M = M + (M>>1) + (M>>3) + (M>>4) + (M>>6); break; // 697Hz
                            case 1: coeff_M = M + (M>>1) + (M>>3) + (M>>7) + (M>>8); break; // 770Hz
                            case 2: coeff_M = M + (M>>1) + (M>>4);                   break; // 852Hz
                            case 3: coeff_M = M + (M>>1) - (M>>6);                   break; // 941Hz
                            case 4: coeff_M = M + (M>>3) + (M>>5) + (M>>7);          break; // 1209Hz
                            case 5: coeff_M = M + (M>>7);                            break; // 1336Hz
                            case 6: coeff_M = (M>>1) + (M>>2) + (M>>5) + (M>>7);     break; // 1477Hz
                            case 7: coeff_M = (M>>1) + (M>>4) - (M>>8);              break; // 1633Hz
                        }

                        power[i] = (uint32_t)(sq1 + sq2 - coeff_M);
                    }

                    // 2. 低群(Row)と高群(Col)の最大ピークを探す
                    uint8_t row_idx = 0;
                    uint8_t col_idx = 4;
                    uint32_t max_row_power = 0;
                    uint32_t max_col_power = 0;

                    for (uint8_t i = 0; i < 4; i++) {
                        if (power[i] > max_row_power) { max_row_power = power[i]; row_idx = i; }
                    }
                    for (uint8_t i = 4; i < 8; i++) {
                        if (power[i] > max_col_power) { max_col_power = power[i]; col_idx = i; }
                    }

                    // 3. 閾値判定・ツイスト判定
                    bool twist_ok = true;
                    if (max_row_power > (max_col_power << 3)) twist_ok = false;
                    else if (max_col_power > (max_row_power << 3)) twist_ok = false;

                    // --- 今回のフレームでの検出結果を入れる変数 ---
                    char current_char = 0; // 0は「未検出(無音)」を意味する

                    if ((max_row_power > DTMF_THRESHOLD) && (max_col_power > DTMF_THRESHOLD) && twist_ok) {
                        const char dtmf_char[4][4] = {
                            {'1', '2', '3', 'A'}, {'4', '5', '6', 'B'},
                            {'7', '8', '9', 'C'}, {'*', '0', '#', 'D'}
                        };
                        current_char = dtmf_char[row_idx][col_idx - 4];
                    }

                    // ==========================================
                    // 4. デバウンスと1回入力判定 (ステートマシン)
                    // ==========================================

                    if (current_char != 0) {
                        // 何かDTMFが聞こえている
                        if (current_char == last_detected_char) {
                            release_count = 0; //安定受信中はリリースカウントをリセット
                            // 上限でカウントストップ
                            if( char_match_count < DEBOUNCE_COUNT){
                                char_match_count++;
                            }
                            // DEBOUNCE_COUNT数フレーム(*25ms)が連続で同じキーなら確定
                            if (char_match_count >= DEBOUNCE_COUNT) {
                                // まだ「押しっぱなし」扱いでなければ1回だけ処理を実行
                                if (!key_is_pressed) {
                                    key_is_pressed = true; // 押しっぱなし状態に移行
                                
                                    // ここは1回だけ実行される

                                    // DTMFデコーダのみの出力
                                    // ダイヤルした番号はパルス側で得られるのでここは
                                    //  1. ダイヤルパルス出力禁止時
                                    //  2. 通話中
                                    // の場合にシリアルで受信したDTMFを出力
                                    if(dp_out_enable == false || is_talking == true){
                                        char d_str[8] = "DTMF:";
                                        d_str[5] = current_char;
                                        Transmit_String(d_str);
                                    }

                                    // 通話中であればパルス出力させない
                                    if(is_talking == false){
                                        // DP出力を開始させる
                                        Start_DialPulse(current_char);
                                    }
                                }
                            }
                        } else {
                            // 前回のフレームと違うキーが来た (押し間違いやノイズ)
                            release_count++;
                            if(release_count >= DEBOUNCE_COUNT){
                                last_detected_char = current_char;
                                char_match_count = 1; // カウントリセット
                                key_is_pressed = false;
                                release_count = 0;
                            }
                        }
                    } else {
                        // 何も聞こえない (無音、または閾値以下)
                        // 指が離れたとみなして全てリセット
                        release_count ++; //無音フレームをカウントアップ
                        if(release_count >= DEBOUNCE_COUNT){
                            last_detected_char = 0;
                            char_match_count = 0;
                            key_is_pressed = false;
                            release_count = 0;
                        }
                    }
                }
            }
            // DTMF->DPのメインでの変換処理ここまで

            // シリアルコマンド受信処理
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
                    if (rxIndex < CMDBUFSIZE - 1) {
                        rxBuffer[rxIndex++] = (char)rxData;
                    }
                    // バッファ溢れ防止
                    if(rxIndex >= CMDBUFSIZE - 1 ) rxIndex = 0;
                }
            } //UART受信処理ここまで

            // ケイデンス制御
            // ring_period = 1で鳴動時間
            // RING_CMD状態が1->0に変化した場合にはケイデンスサイクルをリセット
            // (PBX側からDistinctive ring対応できるように)
            if( last_ring_cmd == 1 && RING_CMD == 0){
                cadence_cycle = 0;
            }
            last_ring_cmd = RING_CMD;
            // RBT音ケイデンスも同様にスタート位置を決める
            if(last_tg2 == 1 && TG2_IN == 0){
                cadence_cycle = 0;
            }
            last_tg2 = TG2_IN;

            // ケイデンスサイクル制御
            cadence_cycle++;
            if( cadence_cycle <= CADENCE_ACTIVE) ring_period = 1;
            else if( cadence_cycle <= CADENCE_DEACTIVE) ring_period = 0;
            else cadence_cycle = 0;
            
            // ビジー音は断続音にする
            // 500mS毎の断続(25x20)
            busy_timer++;
            if (busy_timer >= 20) {
                busy_timer = 0;
                busy_phase = !busy_phase;
            }

            // ==========
            // SLIC制御部
            // ==========
            if ((RING_CMD == 0 || ser_ring_cmd) && (logical_hook_state == false)) {
                if(ring_period == 1){ //鳴動サイクル内なら鳴らす
                    RM_OUT_SetHigh();
                    //  25mSフレームなので反転することで20Hzになる 
                    if (FR_OUT_GetValue() == 0) FR_OUT_SetHigh(); else FR_OUT_SetLow();
                } else { //鳴動サイクル外なので鳴らさない
                    RM_OUT_SetLow();
                    FR_OUT_SetHigh();
                }
            }
            else { // 鳴動サイクル中はリバース制御しても意味はないのでリバースさせない
                //回線リバース制御:pinをUARTの受信と兼用させるため3サイクル(75ms)は作動させない
                if(!ser_rev_cmd){ //シリアル側で制御されている場合はピンは無視
                    if(REV_CMD == 1 )
                    {
                        RM_OUT_SetLow();
                        FR_OUT_SetLow();
                        rev_cmd_count = 3;
                    }
                    else {
                        if(rev_cmd_count == 0){
                            RM_OUT_SetLow();
                            FR_OUT_SetHigh();
                            rev_cmd_count = 0;
                        }
                        else{
                            rev_cmd_count--;                       
                        }
                    }
                }
                // シリアルでの回線リバース制御
                if(ser_rev_cmd){
                    RM_OUT_SetLow();
                    FR_OUT_SetHigh();
                }
            }
            // --- SLIC制御ここまで ---

            // --- トーン制御 
            // 400HzはNCOで生成
            bool tg1 = TG1_IN;
            bool tg2 = TG2_IN;
            uint8_t tone_mode = 3; 

            if(tg1_overwrite) tg1 = 1;

            // TG入力ピンにより発生させるトーンを制御
            // HH=3でトーン停止
            if      (tg1 == 0 && tg2 == 0) tone_mode = 0; 
            else if (tg1 == 0 && tg2 == 1) tone_mode = 1; 
            else if (tg1 == 1 && tg2 == 0) tone_mode = 2; 
            else                           tone_mode = 3; 

            // ピン制御されていない(HH=Open)場合にはシリアルのモードを採用
            if(tone_mode == 3){
                tone_mode = ser_tone_mode;
            }

            if (tone_mode == 3) {
                // HH=Hi-Z モード
                TONE_PIN_HIZ();    // 出力ピンを Hi-Zに
            } else {
                bool output_enable = false;
                switch (tone_mode) {
                    case 0: output_enable = true; break; //LL=Calling Tone(400Hz)
                    case 1: output_enable = busy_phase; break; //LH=Busy Tone(400Hz断続)
                    case 2: //HL=Ring Back Tone : RBT時は変調+ケイデンス制御する 
                        if(ring_period == 1){
                            // 25mS(20Hzで変調:出力ON/OFFの繰り返し)
                            if(tone_mod_phase == true) output_enable = true;
                            else output_enable = false;
                            tone_mod_phase = !tone_mod_phase;
                        } else { //2秒休止時間
                            output_enable = false;
                        }
                        break; 
                }

                if (output_enable) {
                    TONE_PIN_OUTPUT();    // 出力有効区間
                } else {
                    TONE_PIN_HIZ();       // 出力禁止区間
                }
            }
            // --- トーン制御ここまで --
        
        } // 25msフレーム処理ここまで 
    } // whileここまで
} // end of main