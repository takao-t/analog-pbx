#include <Arduino.h>
#include <SoftwareSerial.h>

// ==========================================
// ピンアサイン設定 (Arduino UNO)
// ==========================================
// Line 1 (回線1: 内線11)
#define L1_T1 2
#define L1_T2 3
#define L1_RI 4
#define L1_HO 5

// Line 2 (回線2: 内線12)
#define L2_T1 6
#define L2_T2 7
#define L2_RI 8
#define L2_HO 9

// Switch board 制御用 (SoftwareSerial)
#define SW_TX 10
#define SW_RX 11 // 受信はしないため未接続でOK

SoftwareSerial swSerial(SW_RX, SW_TX);

// ==========================================
// 状態定義の構造体
// ==========================================
enum LineState {
  ST_IDLE,        // 待機中 (オンフック)
  ST_DIAL_TONE,   // 発信音出力中
  ST_DIALING,     // ダイヤルパルス受信中
  ST_RINGING_OUT, // 呼出中 (発信側：リングバックトーン出力)
  ST_RINGING_IN,  // 着信中 (着信側：ベル鳴動)
  ST_CONNECTED,   // 通話中
  ST_BUSY         // 話中音出力中
};

// 各回線状態を保持する構造体
struct Line {
  int id; // 2回線なので0か1
  uint8_t pin_t1, pin_t2, pin_ri, pin_ho;
  
  LineState state; //現在の状態
  bool current_ho; 
  
  // デバウンス用変数
  bool raw_ho;
  unsigned long last_ho_change_time;
  
  // ダイヤル処理用
  int pulse_count;         // 現在の桁のパルス数
  unsigned long last_pulse_time; // 最後のパルス変化時間
  int dialed_number;       // 入力された内線番号(11, 12等)
  int digit_count;         // 入力された桁数
};

// 2回線分の管理構造体
Line lines[2] = {
  {0, L1_T1, L1_T2, L1_RI, L1_HO, ST_IDLE, false, false, 0, 0, 0, 0, 0},
  {1, L2_T1, L2_T2, L2_RI, L2_HO, ST_IDLE, false, false, 0, 0, 0, 0, 0}
};

// ==========================================
// 制御ヘルパー関数
// ==========================================

// 電話機へのトーン制御
void setTone(Line* line, bool t1, bool t2) {
  digitalWrite(line->pin_t1, t1 ? HIGH : LOW);
  digitalWrite(line->pin_t2, t2 ? HIGH : LOW);
}

// 電話機鳴動制御
void setRinging(Line* line, bool ring) {
  digitalWrite(line->pin_ri, ring ? LOW : HIGH);
}

// 動作ログ出力
void logMessage(Line* line, const char* msg) {
  Serial.print("[Line "); Serial.print(line->id + 1); Serial.print("] ");
  Serial.println(msg);
}

// 切断時処理
void handleDisconnect(Line* line, Line* other) {
  setTone(line, true, true);
  setRinging(line, false);
  
  if (line->state == ST_RINGING_OUT && other->state == ST_RINGING_IN) {
    other->state = ST_IDLE;
    setRinging(other, false);
    logMessage(other, "Ringing Cancelled");
  }
  else if (line->state == ST_CONNECTED && other->state == ST_CONNECTED) {
    other->state = ST_BUSY;
    setTone(other, false, true); 
    logMessage(other, "Changed to BUSY");
    
    swSerial.print("R0102\r"); 
    delay(10);
    swSerial.print("R0201\r");
    delay(10);
    Serial.println(">> SwitchBoard: R0102, R0201 (Disconnected)");
  }
  
  line->state = ST_IDLE;
  logMessage(line, "Changed to IDLE");
}

// ダイヤル完了時のルーティング処理
void routeCall(Line* caller) {
  Serial.print("[Line "); Serial.print(caller->id + 1); Serial.print("] Calling Extension: ");
  Serial.println(caller->dialed_number);
  
  int target_id = -1;
  
  // 内線番号の判定
  // 内線番号設定はないので決め打ち
  if (caller->dialed_number == 11) {
    target_id = 0; // Line 1
  } else if (caller->dialed_number == 12) {
    target_id = 1; // Line 2
  }
  
  // 不正な番号、または自分自身に発信した場合 -> BUSY
  if (target_id == -1 || target_id == caller->id) {
    caller->state = ST_BUSY;
    setTone(caller, false, true); // ビジートーン
    logMessage(caller, "Invalid Number or Self - BUSY");
  } 
  else {
    // 相手回線の状態を確認
    Line* target = &lines[target_id];
    if (target->state == ST_IDLE) {
      // 相手が空いている -> 呼出開始
      caller->state = ST_RINGING_OUT;
      setTone(caller, true, false); // リングバックトーン
      
      target->state = ST_RINGING_IN;
      setRinging(target, true);    // ベル鳴動開始
      logMessage(caller, "Ringing target...");
    } else {
      // 相手が使用中 -> BUSY
      caller->state = ST_BUSY;
      setTone(caller, false, true); // ビジートーン
      logMessage(caller, "Target is BUSY");
    }
  }
}

// ------------------------------------------
// フック状態変化エッジ処理
// ------------------------------------------
void handleHOEdge(Line* line, bool isOffHook) {
  Line* other = &lines[1 - line->id]; 
  
  if (!isOffHook) {
    // --- オンフック (H -> L) ---
    if (line->state == ST_DIAL_TONE) {
      // ダイヤル開始
      line->state = ST_DIALING;
      line->pulse_count = 1;
      line->dialed_number = 0;
      line->digit_count = 0;
      line->last_pulse_time = millis();
      setTone(line, true, true); // 発信音停止
      logMessage(line, "Dialing Started");
    } 
    else if (line->state == ST_DIALING) {
      line->pulse_count++;
      line->last_pulse_time = millis();
    } 
    else {
      handleDisconnect(line, other);
    }
  } else {
    // --- オフフック (L -> H) ---
    if (line->state == ST_IDLE) {
      line->state = ST_DIAL_TONE;
      setTone(line, false, false);
      logMessage(line, "Off-Hook (Dial Tone)");
    }
    else if (line->state == ST_DIALING) {
      line->last_pulse_time = millis();
    }
    else if (line->state == ST_RINGING_IN) {
      setRinging(line, false);
      line->state = ST_CONNECTED;
      setTone(line, true, true);
      logMessage(line, "Answered (Connected)");
      
      if (other->state == ST_RINGING_OUT) {
        other->state = ST_CONNECTED;
        setTone(other, true, true);
        
        swSerial.print("C0102\r");
        delay(10);
        swSerial.print("C0201\r");
        delay(10);
        Serial.println(">> SwitchBoard: C0102, C0201 (Connected)");
      }
    }
  }
}

// ------------------------------------------
// タイムアウト・桁確定監視処理
// ------------------------------------------
void updateLineTimeouts(Line* line) {
  if (line->state != ST_DIALING) return;

  Line* other = &lines[1 - line->id];
  unsigned long now = millis();
  
  if (!line->current_ho) {
    // L状態が300ms継続 -> オンフック（切断）とみなす
    if (now - line->last_pulse_time > 300) {
      logMessage(line, "Dialing Aborted (On-Hook Timeout)");
      handleDisconnect(line, other);
    }
  } else {
    // H状態（オフフック）の継続監視
    
    // パルス計測中で600ms経過 -> 1桁分の入力確定
    if (line->pulse_count > 0 && (now - line->last_pulse_time > 600)) {
      // 10パルスは数字の0とする処理
      int digit = (line->pulse_count == 10) ? 0 : (line->pulse_count % 10);
      line->dialed_number = (line->dialed_number * 10) + digit;
      line->digit_count++;
      
      Serial.print("[Line "); Serial.print(line->id + 1); Serial.print("] Digit received: ");
      Serial.println(digit);
      
      // 次の桁へ向けてリセット
      line->pulse_count = 0; 
      line->last_pulse_time = now; 
      
      // 2桁入力されたらルーティング実行
      if (line->digit_count >= 2) {
        routeCall(line);
      }
    }
    // 次の桁がダイヤルされず4秒経過 -> タイムアウトでルーティング実行（「1」等だけ回された場合の処理）
    else if (line->pulse_count == 0 && line->digit_count > 0 && (now - line->last_pulse_time > 4000)) {
      logMessage(line, "Inter-digit timeout");
      routeCall(line);
    }
  }
}

// フック出力の監視
void pollHO(Line* line) {
  bool reading = (digitalRead(line->pin_ho) == HIGH); 
  
  if (reading != line->raw_ho) {
    line->raw_ho = reading;
    line->last_ho_change_time = millis();
  }
  
  // デバウンス処理(15ms) : 注:SLICユニット側でデバウンスしているが安全のため
  // 無くてもたぶん大丈夫
  if ((millis() - line->last_ho_change_time) > 15) {
    if (reading != line->current_ho) {
      line->current_ho = reading;
      handleHOEdge(line, line->current_ho);
    }
  }
}

// Arduino初期化
void setup() {
  // デバッグコンソールは115.2k
  // 動作ログもこちらへ出力
  Serial.begin(115200);
  // Switchboardへのソフトウェアシリアルは9600
  swSerial.begin(9600);
  
  // 各SLICユニット接続ピンの初期化
  for (int i = 0; i < 2; i++) {
    pinMode(lines[i].pin_t1, OUTPUT);
    pinMode(lines[i].pin_t2, OUTPUT);
    pinMode(lines[i].pin_ri, OUTPUT);
    pinMode(lines[i].pin_ho, INPUT); 
    
    // 初期状態:トーン停止とRING停止の明示処理
    setTone(&lines[i], true, true); 
    setRinging(&lines[i], false);   
  }
  
  // Switchboard初期化(リセットコマンド送出)
  Serial.println("Initializing SwitchBoard...");
  delay(500);
  swSerial.print("RFFFF\r");
  delay(100);
  swSerial.print("RFFFF\r");
  delay(100);
  
  Serial.println("PBX System Started.");
}

// メインループ
// 回線毎にフック状態を監視し
// 各回線のステートマシンを処理する
void loop() {
  for (int i = 0; i < 2; i++) {
    pollHO(&lines[i]);
    updateLineTimeouts(&lines[i]);
  }
}