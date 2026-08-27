# これは何？
Arduino UNOでPBXのコア部分を実装した例です。

2回線(1:1)のアナログPBXに仕立てることができます。

## 必要なコンポーネント
- SLICユニット×2
- Switchboard 2x2を1枚か2x4を1枚
- 当然、Arduino UNO(ATmega 328P)

## 接続
ソース中でピン番号を指定していますので好きなピンに変えてもかまいません。基本は以下の通りです。
|SLICユニット|Arduino UNO|備考|
|-----|-----|-----|
|T1|2|Line 1|
|T2|3|LIne 1|
|RI|4|Line 1|
|HO|5|Line 1|
|T1|6|Line 2|
|T2|7|Line 2|
|RI|8|Line 2|
|HO|9|Line 2|

|SwitchBoard|Arduino UNO|備考|
|-----|-----|-----|
|RX|9|SoftwareSerialのTX|
||10|SoftwareSerialのRXだが繋がない|

## 使い方
接続方法等は各ユニットの説明をよく読んでください。

内線番号は11(Line 1)と12(Line 2)の決め打ちです。変えたい場合にはソースを修正してください。ダイヤルすれば相手に繋がって会話できます。DTMF->DP変換はSLICユニットが行いますので、DTMF(トーン)式の電話機でも使うことができます。

まあ、要するにArduino UNOでも交換機が作れますよというデモみたいなもんです。

開発にはVScode+Platform IOを使用していますが、通常のArduino環境でも問題なく使えると思います。
