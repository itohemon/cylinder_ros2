# cylinder_ros2

## 事前設定
### ROS2のセットアップファイル作成
以下を`$HOME/ros2_setup.bash`に記載する
```bash
#
# ROS2
#
source /opt/ros/galactic/setup.bash
source /home/ubuntu/microros_ws/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export PICO_SDK_PATH=/home/ubuntu/pico-sdk
```

### microROS実行環境セットアップ
[zennの記事](https://zenn.dev/katsuitoh/articles/0d33c3e95ff466#raspberry-pi-4%E4%B8%8A%E3%81%AEubuntu%E3%81%ABmicro-ros%E5%AE%9F%E8%A1%8C%E7%92%B0%E5%A2%83%E3%82%92%E4%BD%9C%E3%82%8B)
を参考にセットアップする

### LD06, BNO055のノードセットアップ
[zennの記事](https://zenn.dev/katsuitoh/articles/af8b36a26ab66e)を参考にセットアップする

## RaspberryPi Picoへのファームウェアインストール
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
