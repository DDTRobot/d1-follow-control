# d1-remote-control

D1 机器人远程控制包

## 构建与安装

```bash
git clone <repo-url>
cd d1-follow-control
bash build_deb.sh
```

## 手动安装 deb

```bash
sudo dpkg -i d1-follow-control_1.0_all.deb
```

## 后续不用该功能可以卸载，会占用sdk mode

```bash
sudo apt remove d1-follow-control
```

## 使用方法

安装完，打开SDK mode即可进入智能跟随模式，请在空旷测试区域测试。(由于相机视野有限，在机器狗周围仍存在大部分盲区，有一定的碰撞风险，请自行排查并承担其产生的风险)

请注意：勿在玻璃和高反光物体附近测试，D435i 使用主动红外结构光 + 双目立体视觉：左右红外相机捕捉 IR 投影图案，通过视差计算深度。

玻璃和反光物体的问题:

玻璃（透明材质）
IR 光线直接穿透玻璃，相机看到的是玻璃后方的物体
深度值要么是玻璃后方物体的距离，要么因无法匹配而返回 0（无效值）
常见场景：玻璃门、水族箱、透明塑料容器
强反光物体（镜面、金属抛光面）

强反光物体（镜面、金属抛光面）
IR 光被镜面反射到其他方向，双目相机接收不到有效反射
或者接收到来自其他方向的杂散反射，导致深度值偏大或无效
常见场景：镜子、不锈钢表面、光滑瓷砖

控制部分代码请参考:
deb-package\opt\d1-follow-control\d1_follow_d435i.py

参数说明

目标距离，靠近多远停止
target_distance

跟随更快/更慢
kp_distance 调大/调小

转向更快/更慢
kp_angle 调大/调小

最高速度
max_linear_speed / max_angular_speed

到位后还在轻微抖动
适当加大 distance_deadzone

避障安全距离阈值
self.safe_distance

跨过障碍设定时长，理论上等于（机身长度+障碍垂直距离）/ 速度
self.turning_forward_duration

平滑系数，控制的是每帧速度变化的比例，避免速度突变
self.smooth_alpha = 0.6


默认最高速1.5m/s，如需修改为最高3m/s的跟随，可参考以下修改：

self.target_distance = 1.8
self.max_linear_speed = 3.0
self.max_angular_speed = 3.0
self.safe_distance = 1.6
self.turning_forward_duration = 0.8 

提高target_distance可避免高速下由于速度惯性，到位停止不及时而产生碰撞风险
提高safe_distance可延长避障距离，便于高速下及时避障，注意：请在空旷位置测试，理论上路面宽度需要大于safe_distance x 2 ，才能稳定跟随，不然会判定左侧或者右侧存在障碍，频繁进入避障模式