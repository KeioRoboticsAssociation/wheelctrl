# wheelctrl_4ws

## 概要

base_linkの速度指令値を**4輪ステアリング**のモデルに基づいて4つのモーターに送る指令値へ変換するROSパッケージ



## Subscribed Topics

- **/cmd_vel** (type : `geometry_msgs::Twist`)
- **/pose** (type : `geometry_msgs::PoseStamped`)



## Published Topics

- **/control_RF** (type : `Float32MultiArray`)
- **/control_LF** (type : `Float32MultiArray`)
- **/control_LB** (type : `Float32MultiArray`)
- **/control_RB** (type : `Float32MultiArray`)

<img src="https://i.imgur.com/3giWneE.png" style="zoom:50%;" />

(補足)

1. [こちら](https://github.com/moden3/serial_test)のROS nodeに接続して、マイコンに指令値を送ることを想定している

2. `Float32MultiArray`の具体的なコンテンツは以下の通り

```c++
std_msgs::Float32MultiArray floatarray;
floatarray.data.resize(2);
floatarray.data[0] = target_speed[0];// wheel velocity
floatarray.data[1] = target_theta[0];// wheel angle
pub_RF.publish(floatarray);
```



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])
- **lost_time_threshold** : timeout period (default : 500[ms])
- **body_width** : 4WSロボット(正方形)の一辺の長さ (default : 0.440[m])