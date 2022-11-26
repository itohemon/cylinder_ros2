# ハードウェアについて
## 使用ハードウェア
- Raspberry Pi 4B 8GB
- DCブラシモータ([SPG30E-60K](https://www.cytron.io/c-motor-and-motor-driver/c-dc-motor/c-dc-geared-motor/p-12v-75rpm-3kgfcm-brushed-dc-geared-motor-with-encoder))  2個
- 自作モーター制御・LED制御基板
  - [モータドライバ TB6612FNG](https://www.robotshop.com/jp/ja/pololu-dual-dc-motor-driver-1a-4-5v-3-5v-tb6612fng.html)
  - Raspberry Pi Pico
- [8x8 LED & LEDドライバ(MAX7219)基板](https://www.amazon.co.jp/KKHMF-MAX7219-8X8-LED-%E3%83%A2%E3%82%B8%E3%83%A5%E3%83%BC%E3%83%AB-DIY-%E3%82%AD%E3%83%83%E3%83%88-Arduino%E3%81%A8%E4%BA%92%E6%8F%9B/dp/B07JH4H8RT/ref=sr_1_11)  2個
- IMU([BNO055](https://www.robot-electronics.co.uk/bno055.html))
- LiPoバッテリ
- LIDAR([LD06](https://www.ldrobot.com/product/en/98)）	
- 自作LiPoカットオフ・電圧変換基板
  - [DC/DCコンバータ](https://akizukidenshi.com/catalog/g/gK-15108/)
- 自作電源スイッチ基板
  - [4.5～40V DC 16A　Big MOSFETスライドスイッチ](https://www.robotshop.com/jp/ja/45-40v-dc-16a-big-mosfet-slide-switch.html)
  - タクトスイッチ
  - [電圧計](https://www.robotshop.com/jp/ja/028-led-digital-dc-voltmeter-spa.html)

## Raspberry Pi 4B
### GPIO
|ピン番号|ピン名|接続先部品|接続先ピン名|
|---|---|---|---|
|1|3.3V Power|BNO055|VCC|
|2|5V Power|LD06|P5V|
|4|5V Power|ファン|+|
|6|GND|ファン|-|
|29||LD06|Tx|
|30|GND|LD06|GND|
|32|GPIO 12(TXD5)|BNO055|Rx|
|33|GPIO 13(RXD5)|BNO055|Tx|
|34|GND|BNO055|Gnd|

`/boot/firmware/usercfg.txt`でUART5を有効化する

## Raspberry Pi Pico
|ピン番号|ピン名|接続先部品|接続先ピン名|
|---|---|---|---|
|1|GP0|TB6612FNGv2|PWMA|
|2|GP1|TB6612FNGv2|AI2|
|3|GND|-|GND|
|4|GP2|TB6612FNGv2|AI1|
|5|GP3|TB6612FNGv2|STBYn|
|6|GP4|TB6612FNGv2|BI1|
|7|GP5|TB6612FNGv2|BI2|
|8|GND|-|GND|
|9|GP6|TB6612FNGv2|PWMB|
|11|GP8|モータ|ACHB|
|12|GP9|モータ|ACHA|
|13|GND|-|GND|
|14|GP10|モータ|BCHB|
|15|GP11|モータ|BCHA|
|18|GND|-|GND|
|22|GP17|MAX7219|CS|
|23|GND|-|GND|
|24|GP18|MAX7219|CLK|
|25|GP19|MAX7219|DIN|
|28|GND|-|GND|
|30|RUN|SW|1|
|33|GND|-|GND|
|36|3V3|-|3.3V|
|38|GND|-|GND|

## TB6612FNG
|ピン番号|ピン名|接続先部品|接続先ピン名|
|---|---|---|---|
|1|GND|-|GND|
|2|VCC|-|3.3V|
|3|AO1|モータA|1|
|4|A02|モータA|2|
|5|B02|モータB|1|
|6|B01|モータB|2|
|7|VMOT|-|11.1V|
|8|GND|-|GND|
|9|GND|-|GND|
|10|PWMB|Pico|GP6|
|11|BI2|Pico|GP5|
|12|BI1|Pico|GP4|
|13|STBY|Pico|GP3|
|14|AI1|Pico|GP2|
|15|AI2|Pico|GP1|
|16|PWMA|Pico|GP0|
