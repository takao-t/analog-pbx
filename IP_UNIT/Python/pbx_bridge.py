import asyncio
import serial_asyncio
import json
import logging
import re

# ==========================================
# PIC PBX IPユニット-BareSIPブリッジプログラム
# 
# 設定項目
# 注意: Raspberry PiのGPIO上UARTならば/dev/ttyS0だがUSBシリアルを
#       使って接続する場合はデバイス名を変更のこと
#       ttyS0を使用する場合にはRaspberry Piのコンソールとして使用
#       されるgettyを無効化すること
# BareSIP側では以下の設定を行う必要がある
#    module_app              ctrl_tcp.so
#    ctrl_tcp_listen         127.0.0.1:4444 # ctrl_tcp - TCP interface JSON
#    Raspberry Piの場合、オーディオ入力等を持たないので適切なオーディオインタフェースを
#    接続しBareSIPで適切に設定しておくこと。このプログラム自体は呼制御しか関与しない
# ==========================================
SERIAL_PORT = '/dev/ttyS0'
SERIAL_BAUD = 9600
BARESIP_HOST = '127.0.0.1'
BARESIP_PORT = 4444

# ダイヤル発信時のデフォルトSIPドメイン (AsteriskなどのIP)
SIP_DOMAIN = '192.168.254.235'  # 自環境に合わせて要変更

# 内線番号なし、またはパースできなかった場合のデフォルト着信先(フォールバック)
# PBXの内線番号を設定する
FALLBACK_EXTENSION = '11' 

# ロギング設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

# ==========================================
# グローバル変数
# ==========================================
baresip_writer = None
serial_writer = None
last_pong_time = 0

# ==========================================
# PING-PONG ハートビート送信処理
# ==========================================
async def heartbeat_task():
    global serial_writer
    while True:
        await asyncio.sleep(5)
        if serial_writer:
            try:
                serial_writer.write(b"PING\n")
                # logger.info("-> PIC: PING")
            except Exception as e:
                logger.error(f"Failed to send PING: {e}")

# ==========================================
# Baresip (TCP/Netstring) 処理
# ==========================================
async def read_netstring(reader):
    """BaresipからのNetstringデータ ([length]:[data],) を読み取る"""
    len_bytes = bytearray()
    while True:
        char = await reader.readexactly(1)
        if char == b':':
            break
        if not char.isdigit():
            raise ValueError(f"Invalid netstring length character: {char}")
        len_bytes.extend(char)
        if len(len_bytes) > 10:
            raise ValueError("Netstring length too long")
            
    if not len_bytes:
        raise ValueError("Netstring length missing")
        
    data_len = int(len_bytes.decode('ascii'))
    
    # 指定されたバイト数だけJSONデータを読み取る
    data = await reader.readexactly(data_len)
    
    # 末尾のカンマを読み取る
    comma = await reader.readexactly(1)
    if comma != b',':
        raise ValueError("Netstring missing trailing comma")
        
    return data

def baresip_send_cmd(cmd_dict):
    """BaresipへNetstring形式でJSONコマンドを送信する"""
    global baresip_writer
    if baresip_writer:
        try:
            data = json.dumps(cmd_dict).encode('utf-8')
            # Netstringフォーマット: length:data,
            netstring = f"{len(data)}:".encode('ascii') + data + b","
            baresip_writer.write(netstring)
            logger.info(f"-> Baresip: {cmd_dict}")
        except Exception as e:
            logger.error(f"Failed to send to Baresip: {e}")

async def baresip_tcp_client():
    global baresip_writer
    
    while True:
        try:
            logger.info(f"Connecting to Baresip at {BARESIP_HOST}:{BARESIP_PORT}...")
            reader, baresip_writer = await asyncio.open_connection(BARESIP_HOST, BARESIP_PORT)
            logger.info("Connected to Baresip.")
            
            while True:
                try:
                    # Netstringパーサーを経由してJSON文字列を取得
                    data = await read_netstring(reader)
                    event_str = data.decode('utf-8')
                    event = json.loads(event_str)
                    await handle_baresip_event(event)
                except asyncio.IncompleteReadError:
                    logger.warning("Baresip connection closed.")
                    break
                except json.JSONDecodeError:
                    pass
                except Exception as e:
                    logger.error(f"Error parsing Baresip event: {e}")
                    
        except ConnectionRefusedError:
            logger.error("Baresip connection refused. Retrying in 5 seconds...")
            await asyncio.sleep(5)
        except Exception as e:
            logger.error(f"Baresip connection error: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)

async def handle_baresip_event(event):
    """Baresipから受信したJSONイベントを処理してPICへ転送"""
    global serial_writer
    
    # 必須キーの確認 (応答メッセージなどは無視)
    if 'class' not in event or 'type' not in event:
        return
        
    if event['class'] != 'call':
        return

    evt_type = event['type']
    
    # 1. 着信 (INVITE) を受けた場合
    if evt_type == 'CALL_INCOMING':
        uri = event.get('localuri', '') 
        # sip:の直後に数字(1桁以上)があり、その後に@が続くパターンを検索
        match = re.search(r'sip:(\d+)@', uri)
        
        # パターンにマッチしなかった場合はFALLBACK_EXTENSIONを使用する
        if match:
            ext_num = match.group(1)
            logger.info(f"<- Baresip INCOMING to parsed EXT: {ext_num} (from URI: {uri})")
        else:
            ext_num = FALLBACK_EXTENSION
            logger.info(f"<- Baresip INCOMING to FALLBACK EXT: {ext_num} (URI '{uri}' did not contain a numeric extension)")

        if serial_writer:
            serial_writer.write(f"RING {ext_num}\n".encode('ascii'))
            logger.info(f"-> PIC: RING {ext_num}")
            

    # 2. 発信または着信の通話が確立（相手が応答）した場合
    elif evt_type == 'CALL_ESTABLISHED':
        logger.info("<- Baresip ESTABLISHED")
        if serial_writer:
            serial_writer.write(b"ANS\n")
            logger.info("-> PIC: ANS")

    # 3. 通話が切断（ハングアップ/拒否）された場合
    elif evt_type == 'CALL_CLOSED':
        logger.info("<- Baresip CLOSED")
        if serial_writer:
            serial_writer.write(b"DROP\n")
            logger.info("-> PIC: DROP")


# ==========================================
# PIC (UART) 処理
# ==========================================
class SerialProtocol(asyncio.Protocol):
    def connection_made(self, transport):
        global serial_writer
        self.transport = transport
        serial_writer = transport
        logger.info(f"Connected to PIC on {SERIAL_PORT} at {SERIAL_BAUD}bps.")
        self.buffer = b""

    def data_received(self, data):
        self.buffer += data
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            line_str = line.decode('ascii', errors='ignore').strip()
            if line_str:
                asyncio.create_task(self.handle_serial_cmd(line_str))

    def connection_lost(self, exc):
        global serial_writer
        serial_writer = None
        logger.error(f"Serial connection lost: {exc}")

    async def handle_serial_cmd(self, cmd):
        """PICから受信したテキストを処理してBaresipへ転送"""

        # ログ溜まり防止のためPONGだけ別処理
        if cmd == "PONG":
            global last_pong_time
            last_pong_time = asyncio.get_event_loop().time()
            # logger.info(f"<- PIC: {cmd}")
            return    

        # 通常処理
        logger.info(f"<- PIC: {cmd}")
        
	# PIC側から応答が来たら着信させる 
        if cmd == "ANS":
            baresip_send_cmd({"command": "accept"})
            return

        # 1. 発信要求 (DIAL 123)
        if cmd.startswith("DIAL "):
            number = cmd[5:].strip()
            # Baresipへ JSONフォーマットで発信コマンドを送信
            baresip_send_cmd({
                "command": "dial", 
                "params": f"sip:{number}@{SIP_DOMAIN}"
            })
            
        # 2. 切断要求 (DROP または DROP (Reason: ...))
        elif cmd.startswith("DROP"):
            # Baresipへ JSONフォーマットで切断コマンドを送信
            baresip_send_cmd({"command": "hangup"})


async def main():
    # シリアル通信のタスクを開始
    loop = asyncio.get_running_loop()
    serial_coro = serial_asyncio.create_serial_connection(
        loop, SerialProtocol, SERIAL_PORT, baudrate=SERIAL_BAUD
    )
    
    # TCPクライアント(Baresip連携)と並行実行
    await asyncio.gather(
        serial_coro,
        baresip_tcp_client(),
        heartbeat_task()
    )

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Bridge stopped.")
