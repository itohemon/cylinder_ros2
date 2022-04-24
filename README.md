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
### Nav2起動
terminal#3
```shell-session
ros2 launch cylinder_ros2 navigation2.launch.py
```
