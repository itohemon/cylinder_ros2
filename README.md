# cylinder_ros2

## 事前設定
### ROS2のセットアップファイル作成
以下を`$HOME/ros2_setup.bash`に記載する
```bash
#
# ROS2
#
source /opt/ros/humble/setup.bash
source /home/itohemon/microros_ws/install/setup.bash
source /home/itohemon/ros2_ws/install/setup.bash
export PICO_SDK_PATH=/home/itohemon/pico-sdk
```

### microROS実行環境セットアップ
[zennの記事](https://zenn.dev/katsuitoh/articles/0d33c3e95ff466#raspberry-pi-4%E4%B8%8A%E3%81%AEubuntu%E3%81%ABmicro-ros%E5%AE%9F%E8%A1%8C%E7%92%B0%E5%A2%83%E3%82%92%E4%BD%9C%E3%82%8B)
を参考にセットアップする

### LD06, BNO055のノードセットアップ
[zennの記事](https://zenn.dev/katsuitoh/articles/af8b36a26ab66e)を参考にセットアップする

### pigpioのインストール
```
mkdir ~/tmp
cd ~/tmp
git clone https://github.com/joan2937/pigpio.git
cd pigpio
mkdir build
cd build
cmake ..
make
sudo make install
```

### pigpiodの自動起動設定
```
sudo vi /lib/systemd/system/pigpiod.service
```

ファイルの中身は以下。
```
[Unit]
Description=Pigpio daemon

[Service]
Type=forking
PIDFile=pigpio.pid
ExecStart=/usr/local/bin/pigpiod -l -n 127.0.0.1 -s 10
ExecStop=/usr/bin/systemctl kill pigpiod

[Install]
WantedBy=multi-user.target
```
起動確認。
```
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
sudo systemctl status pigpiod
```
リブートして```ps``コマンドでpigpiodが立ち上がってるか確認する。
[参照] (https://qiita.com/mashi0727/items/1e25c4eea511968066ca)


## 母艦PCにてRaspberryPi Picoへのファームウェアインストール
```shell-session
cd ~/ros2_ws/src/cylinder_ros2/
mkdir build
cd build
cmake ..
make
```

コンパイル完了したら、RaspberryPiPicoをUbuntuから見えるよう以下実行
1. BOOTSELを押しながらUSB接続
2. BOOTSELを離す

Picoが自動マウントされるのでマウント先に.uf2ファイルをコピーする
```shell-session
cp cylinder_ros2_pico_micro_ros.uf2 /media/$USER/RPI-RP2
```

## 実行方法
### terminal#1
```shell-session
ros2 launch cylinder_ros2 cylinder.launch.py
```
あるいは
```shell-session
ros2 launch cylinder_ros2 cylinder_withIMU.launch.py
```

### SLAM-ToolBox起動
terminal#2
```shell-session
ros2 launch cylinder_ros2 online_async_launch.py
```

### map保存
terminal#3
```shell-session
cd /home/ubuntu/ros2_ws/src/cylinder_ros2/cmd
./map_save.sh <mapname>
```

### Nav2起動
terminal#3
```shell-session
ros2 launch cylinder_ros2 navigation2.launch.py
```
