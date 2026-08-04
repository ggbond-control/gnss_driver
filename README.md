# g60_driver

`g60_driver` 是面向 G60 GNSS 接收机的 ROS 2 Jazzy 驱动包。它读取 NMEA 0183 数据并发布 ROS 标准定位消息，也可将定位结果转换为局部轨迹。本包不依赖 `gpsd`；随包提供 udev 规则，但只有管理员显式执行安装命令时才会修改系统。

## 功能

- 串口 NMEA 输入与自动重连：`g60_serial`。
- UDP 监听：`g60_udp`；TCP 客户端：`g60_tcp`；NMEA 话题输入：`g60_nmea_topic`。
- 校验并解析 `GGA`、`RMC`、`VTG`、`GST`、`HDT` 语句。
- 发布标准 ROS 消息：定位 `fix`、速度 `vel`、航向 `heading`、时间参考 `time_reference`。
- 将首个有效定位作为原点，发布东-北-天（ENU）局部轨迹 `gps_path`，并提供重置服务。
- 将 `/fix` 与 `/odometry_horizon` 的实时轨迹拟合到同一个里程计坐标系，供 RViz 对比。

## NMEA 与 ENU 术语说明

GNSS 接收机以文本形式连续输出 NMEA 0183 语句。每行以 `$` 开头，例如 `$GNGGA`；其中 `GN` 表示组合 GNSS 星座，后面的三位字母表示语句类型。

| 缩写 | 含义 | 语句主要内容 | 本驱动用途 |
| --- | --- | --- | --- |
| GGA | 定位解数据 | 经纬度、高程、定位质量、卫星数、HDOP | 默认发布 `fix`。 |
| RMC | 推荐的最小导航数据 | 定位是否有效、经纬度、地面速度、地面航向、日期和时间 | 发布 `vel`；启用 `use_rmc_fix` 时也可发布 `fix`。 |
| VTG | 对地航向和对地速度 | 车辆相对地面的航向与速度 | 发布 `vel`。 |
| HDT | 真航向 | 以真北为零度、顺时针增加的航向角 | 转换为 ROS 四元数后发布 `heading`。 |
| GST | GNSS 伪距误差统计 | 纬度、经度和高程的估计标准差 | 用于改进后续 `fix` 的协方差。 |

`fix` 不是 ROS 再次融合得到的结果，而是接收机已经解算好的 GNSS 定位。驱动只做格式转换，并将 GGA 的定位质量及 HDOP/GST 误差写入 ROS 消息。接收机可能在内部同时使用 GPS、北斗等星座，但这是接收机自身完成的。

ENU 是 East、North、Up（东、北、上）的缩写，是以米为单位的局部直角坐标系。轨迹节点把第一个有效 `fix` 设为原点 `(0, 0, 0)`；之后向东移动为 `x` 增大、向北移动为 `y` 增大、海拔升高为 `z` 增大。`gps_path` 就是这些 ENU 点构成的路径，适合车辆周边的小范围轨迹显示。

## 编译

当前包位于 `~/Workspace/driver_ws/src/g60_driver`。在 zsh 中从工作空间根目录编译并加载：

```zsh
cd ~/Workspace/driver_ws
colcon build --packages-select g60_driver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source install/setup.zsh
```

若系统尚未安装串口与四元数转换依赖：

```zsh
sudo apt install ros-jazzy-tf-transformations python3-serial
```

## 设备准备

G60 连接后，Linux 实际分配的设备名是 `/dev/ttyACM0`，但这个编号可能随重插或重启变化。驱动默认使用稳定名称 `/dev/g60_gnss`，波特率为 9600。

若 `/dev/g60_gnss` 已存在，无须执行任何 udev 操作，直接启动即可。若不存在，执行下面命令一次，系统就会为 QinHeng `1a86:55d4` 的 G60 ACM 串口创建该名称：

```zsh
sudo bash "$(ros2 pkg prefix g60_driver)/share/g60_driver/scripts/install_udev_rule.sh"
```

规则会设置 `dialout` 组的读写权限。若启动时提示无法打开串口，将当前用户加入该组一次，然后重新登录终端或系统：

```zsh
sudo usermod -aG dialout "$USER"
```

`sudo chmod 777 /dev/g60_gnss` 只适合临时排查权限问题，重新插拔设备后会失效，正常使用不需要它。

## 启动

仅启动 GNSS 串口驱动：

```zsh
ros2 launch g60_driver g60_serial.launch.py
```

同时启动定位驱动与局部轨迹节点：

```zsh
ros2 launch g60_driver g60_bringup.launch.py
```

## GPS 与里程计对齐

当系统同时发布 `/fix`（`sensor_msgs/NavSatFix`）与 `/odometry_horizon`（`nav_msgs/Odometry`）时，启动对齐节点：

```zsh
ros2 launch g60_driver gps_odom_alignment.launch.py use_rviz:=true
```

该节点只负责对齐和可视化，不会启动 GNSS 串口驱动。若 `/fix` 由本包提供，请另行启动 `g60_serial.launch.py`；若 `/fix` 来自其他定位系统，可直接使用。

节点会按时间戳将 GPS 和里程计位置配对。GPS 经纬度先转换为以首个有效 GPS 点为原点的二维 ENU 米制坐标，再对最开始发生明显位移的 10 组配对样本做一次二维刚体最小二乘拟合，求得旋转与平移。拟合不改变里程计尺度，所有输出统一标记为 `world` 坐标系；数值坐标轴沿用里程计的 `x/y` 定义。

默认只收集 GPS 和里程计均移动至少 1 米的 10 组样本，时间差必须不超过 0.15 秒。第 10 组采集完成后，变换永久锁定，不再受后续 GPS 漂移影响；之后每一条有效 `/fix` 都直接投影到锁定的 `odom` 坐标系并追加到 GPS 轨迹。车辆应在室外获得有效 GPS 后产生一段实际移动轨迹；原地静止时无法可靠估计两个坐标系之间的方向。

RViz 固定坐标系为 `world`，红线为对齐后的 GPS 轨迹，绿线为 `/odometry_horizon` 轨迹。对应接口：

| 话题或服务 | 类型 | 说明 |
| --- | --- | --- |
| `gps_trajectory_aligned` | `nav_msgs/Path` | 映射到 `world` 坐标系的 GPS 轨迹。 |
| `odometry_trajectory` | `nav_msgs/Path` | 映射到 `world` 坐标系的里程计轨迹。 |
| `gps_pose_aligned` | `geometry_msgs/PoseStamped` | 最新 GPS 对齐位置。 |
| `reset_alignment` | `std_srvs/Trigger` | 清空轨迹、原点和拟合结果。 |

常用参数位于 `config/gps_odom_alignment.yaml`：`max_time_delta` 是允许的 GPS/里程计最大时间差，`calibration_pairs` 是锁定变换所需的样本数，`min_calibration_displacement` 是两组校准样本之间的最小位移，`max_points` 是每条输出轨迹的点数上限，`output_frame` 默认为 `world`。

## 接口

| 话题或服务 | 类型 | 说明 |
| --- | --- | --- |
| `fix` | `sensor_msgs/NavSatFix` | 默认由 GGA 发布定位；`use_rmc_fix:=true` 时由 RMC 发布。ROS 不额外融合数据。 |
| `vel` | `geometry_msgs/TwistStamped` | RMC 或 VTG 给出的速度，`x` 为东向，`y` 为北向。 |
| `heading` | `geometry_msgs/QuaternionStamped` | HDT 真航向，转换为 ROS ENU yaw。 |
| `time_reference` | `sensor_msgs/TimeReference` | 接收机 UTC 时间；存在 RMC 日期时一并使用。 |
| `gps_path` | `nav_msgs/Path` | 相对首个有效 fix 的 ENU 轨迹，坐标系默认为 `gps_path`。 |
| `reset_trajectory` | `std_srvs/Trigger` | 清除轨迹与局部原点。 |

话题名均为相对名称，可通过 ROS 的 namespace 或 remap 机制调整。

## 参数

所有驱动节点共用：`frame_id`（默认 `gps`）、`time_ref_source`、`use_rmc_fix`，以及位置误差参数 `epe_no_fix`、`epe_sps`、`epe_dgps`、`epe_rtk_fixed`、`epe_rtk_float`、`epe_waas`。

轨迹节点参数为：`fix_topic`、`path_topic`、`frame_id`、`max_points`。当前 ENU 换算适用于车辆附近的局部轨迹；全局建图或高精度定位应使用专门的地理坐标/融合组件。

## `resource` 文件

`resource/g60_driver` 是 `ament_python` 要求的空标记文件。`colcon build` 会将其安装进 ament 包索引，使 ROS 2 能解析 `ros2 pkg prefix g60_driver`、查找 launch/config 文件和定位可执行程序，请勿删除。
