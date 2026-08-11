# g60_driver

`g60_driver` 是 G60 GNSS 接收机的 ROS 2 Jazzy 驱动包。它从 G60 串口读取 NMEA 数据并发布 `/fix`，可将实时 GPS 与外部里程计 `/odometry_horizon` 对齐到 `world` 坐标系，生成 `/fix_odom`、RViz 轨迹、OVJSN 轨迹文件和可复用的二维变换 TXT 文件；也可脱离实时 GNSS，使用变换文件将 XYZ 转换为 GPS 消息。

## 工作流程

```text
G60 串口 -> /fix ---------------------> 原始 GPS OVJSN
                    |                         data/gps_trajectory.ovjsn
                    v
           GPS/里程计二维对齐 -> world -> RViz 轨迹
                    ^                |
/odometry_horizon ---+                +-> /fix_odom -> 对齐后 OVJSN
                                                    data/fix_odom_trajectory.ovjsn
                                         +-> data/gps_odom_transform.txt
```

对齐过程收集最开始发生明显移动的 30 组 GPS/里程计配对样本，计算一次二维旋转和平移。达到 30 组后变换锁定，后续 GPS 漂移不会继续改变该关系。

## 功能与接口

- 读取 G60 的 NMEA 串口数据，默认设备为 `/dev/g60_gnss`，波特率为 9600。
- 解析 GGA、RMC、VTG、GST、HDT。发布 `/fix`、`/vel`、`/heading`、`/time_reference`。
- 将 `/fix` 与 `/odometry_horizon` 对齐，输出坐标系统一标记为 `world`。
- 发布 `gps_trajectory_aligned`、`odometry_trajectory`、`gps_pose_aligned` 和 `/fix_odom`。
- 使用 `template.ovjsn` 写出两份轨迹，文件内的 `Object.Name` 分别为 `fix`、`fix_odom`。
- 写入并读取 `gps_odom_transform.txt`，支持 GPS 经纬高（LLA）与 `world` XYZ 的正反解。
- 读取已锁定的变换文件，将 `/odometry_horizon`（`nav_msgs/Odometry`）转换为 `/fix_from_odom`（`sensor_msgs/NavSatFix`），并可写出 `fix_from_odom_trajectory.ovjsn`。

`/fix` 是 G60 接收机解算后的定位结果，驱动只负责校验、解析和转换成 ROS 标准消息，并不再次融合 GPS 数据。

## 编译

包位于 `~/Workspace/driver_ws/src/g60_driver`。在 zsh 中执行：

```zsh
cd ~/Workspace/driver_ws
colcon build --packages-select g60_driver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source install/setup.zsh
```

如缺少依赖：

```zsh
sudo apt install ros-jazzy-tf-transformations python3-serial
```

## 设备准备

G60 使用 QinHeng USB 串口，VID:PID 为 `1a86:55d4`，设备节点是 `/dev/ttyACM*`。安装一次 udev 规则后，设备会稳定地显示为 `/dev/g60_gnss`：

```zsh
sudo bash "$(ros2 pkg prefix g60_driver)/share/g60_driver/scripts/install_udev_rule.sh"
sudo usermod -aG dialout "$USER"
```

执行加入用户组的命令后需要重新登录终端或系统。`sudo chmod 777 /dev/g60_gnss` 只能临时排查权限，重新插拔后会失效，正常使用不需要执行。

## 启动

### 仅发布 `/fix`

用于检查 G60 是否正常输出定位：

```zsh
ros2 launch g60_driver g60_serial.launch.py
```

串口参数在 `config/g60_driver.yaml` 中修改。临时覆盖示例：

```zsh
ros2 run g60_driver g60_serial --ros-args -p port:=/dev/ttyACM0 -p baud:=9600
```

### GPS/里程计对齐与变换输出

默认会启动 G60、GPS/里程计对齐和两份 OVJSN 导出：

```zsh
ros2 launch g60_driver g60_alignment.launch.py
```

如需 RViz：

```zsh
ros2 launch g60_driver g60_alignment.launch.py use_rviz:=true
```

完整 launch 的开关如下。布尔值使用 `true` 或 `false`：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `start_g60_fix` | `true` | 启动 G60 串口节点并发布 `/fix`。设为 `false` 时使用外部系统提供的 `/fix`。 |
| `export_polyline` | `true` | 同时写入原始 GPS 和对齐后 GPS 的 OVJSN 文件。 |
| `use_rviz` | `false` | 启动 RViz。 |

例如，外部系统已经提供 `/fix` 时，不再启动 G60 串口：

```zsh
ros2 launch g60_driver g60_alignment.launch.py start_g60_fix:=false use_rviz:=true
```

只需要对齐而不导出文件：

```zsh
ros2 launch g60_driver g60_alignment.launch.py export_polyline:=false
```

### 使用已有变换将里程计转为 GPS

第三个 launch 不启动串口、不读取实时 GPS，也不重新拟合，只读取已经生成的变换文件，并把输入里程计位置反算为 GPS。默认同时把 `/fix_from_odom` 写成 OVJSN：

```zsh
ros2 launch g60_driver g60_transform.launch.py
```

默认接口为：

```text
/odometry_horizon nav_msgs/msg/Odometry      # world/里程计坐标，单位米
/fix_from_odom    sensor_msgs/msg/NavSatFix  # 转换后的经纬高
```

默认输出文件：

```text
data/fix_from_odom_trajectory.ovjsn
```

如果只需要发布 `/fix_from_odom`，不写 OVJSN：

```zsh
ros2 launch g60_driver g60_transform.launch.py export_polyline:=false
```

参数从 `config/g60_transform.yaml` 读取。默认配置为：

```yaml
g60_transform:
  ros__parameters:
    transform_path: ~/Workspace/driver_ws/src/g60_driver/data/gps_odom_transform.txt
    input_topic: /odometry_horizon
    output_topic: /fix_from_odom

g60_fix_from_odom_to_polyline:
  ros__parameters:
    fix_topic: /fix_from_odom
    output_filename: fix_from_odom_trajectory.ovjsn
    polyline_name: fix_from_odom
```

将 GPS 文本转换为 XYZ 的输入模式暂不启动，后续会在此接口上扩展。

### GPS 目标导航

`g60_transform.launch.py` 还提供 `/set_gps_goal` action，类型为 `inspection_interfaces/action/SetGPSGoal`。它将请求中的经纬高转换为 `world` 坐标，并调用 `/multi_map_navigate_to_pose`（`nav2_msgs/action/NavigateToPose`）。请求中的四元数作为导航目标朝向。

```zsh
ros2 action send_goal /set_gps_goal inspection_interfaces/action/SetGPSGoal \
  "{header: {frame_id: world}, latitude: 30.28892, longitude: 119.98098, altitude: 22.6, orientation: {w: 1.0}}" \
  --feedback
```

收到 action goal 后，节点会等待 Nav2 goal 被接受，再等待 Nav2 返回终态结果，最后返回 `SetGPSGoal.Result`。只有 Nav2 action 状态为 `SUCCEEDED` 且 `error_code=0` 时，`success` 才为 `true`；拒绝、取消、中止或超时都会返回 `false`，具体状态和错误信息放在 `message` 中。Nav2 feedback 中的 `distance_remaining` 会转发为 `SetGPSGoal.Feedback.distance_remaining`。

如果外部取消 `/set_gps_goal`，节点会将取消请求继续转发给当前的 `/multi_map_navigate_to_pose` goal，并把 `/set_gps_goal` 标记为 canceled。

相关参数在 `config/g60_transform.yaml` 中：

- `gps_goal_action`：接收 GPS 目标的 action 名称。
- `navigation_action`：Nav2 action 名称。
- `enable_gps_goal_action`：是否启用该桥接。
- `action_server_wait_sec`：等待 action server 和 goal 接受的超时时间。
- `action_result_timeout_sec`：等待 action 终态的超时时间，`0.0` 表示一直等待。

`NavigateToPose.Goal` 的 `behavior_tree` 当前留空，使用 Nav2 默认行为树；目标中的 `pose` 字段位置是变换后的坐标，`pose.header.frame_id` 直接使用 `SetGPSGoal.Request.header.frame_id`，请求必须填写该字段。

## 对齐配置与输出

GPS/里程计对齐配置集中在 `config/g60_alignment.yaml`：

- `fix_topic`：GPS 输入，默认 `/fix`。
- `odom_topic`：里程计输入，默认 `/odometry_horizon`。
- `output_frame`：所有轨迹输出的坐标系，固定使用 `world`。
- `calibration_pairs`：锁定变换前的有效配对数，默认 30。
- `min_calibration_displacement`：相邻校准样本的 GPS 和里程计最小位移，默认 1 米。
- `max_time_delta`：GPS 与里程计允许的最大时间差，默认 0.15 秒。

里程计转 GPS 的配置集中在 `config/g60_transform.yaml`，包含变换文件路径、输入里程计话题和输出 GPS 话题。

一组样本必须同时满足时间接近、GPS 位移至少 1 米、里程计位移至少 1 米。车辆在室内、静止或没有有效 GPS 时，无法完成可靠对齐。

运行时文件写入源码包的 `data/` 目录；从安装空间执行时写入安装包的 `share/g60_driver/data/`：

```text
data/gps_trajectory.ovjsn       # /fix 的原始 GPS 轨迹
data/fix_odom_trajectory.ovjsn      # /fix_odom 的对齐后轨迹
data/fix_from_odom_trajectory.ovjsn # 第三个启动项输出的 /fix_from_odom 轨迹
data/gps_odom_transform.txt         # GPS ENU 到 world 的已锁定二维变换
```

可调用下列服务重新开始记录：

```zsh
ros2 service call /reset_alignment std_srvs/srv/Trigger '{}'
ros2 service call /reset_polyline std_srvs/srv/Trigger '{}'
ros2 service call /reset_polyline_odom std_srvs/srv/Trigger '{}'
ros2 service call /reset_polyline_fix_from_odom std_srvs/srv/Trigger '{}'
```

## 变换文件读写

`gps_odom_transform.txt` 记录 GPS 原点和二维关系：

```text
world_xy = R * gps_enu_xy + t
gps_enu_xy = transpose(R) * (world_xy - t)
```

其中 `gps_enu_xy` 是以首个有效 GPS 为原点的东、北坐标，单位为米。文件只有 `locked=true` 时才可用于转换。

命令行正反解：

```zsh
ros2 run g60_driver g60_transform_convert --transform data/gps_odom_transform.txt --lla 30.0 120.0 10.0
ros2 run g60_driver g60_transform_convert --transform data/gps_odom_transform.txt --xyz 1.0 2.0 0.0
```

批量将 `data/trajectories.txt` 转换为 XYZ 文件：

```zsh
ros2 run g60_driver g60_transform_convert \
  --transform data/gps_odom_transform.txt \
  --input-file data/trajectories.txt \
  --output-file data/xyz.txt
```

输入文件默认每行是 `longitude,latitude`，输出每行是 `x,y,z`。由于当前 `trajectories.txt` 没有高度列，Z 默认使用变换文件中的 GPS 原点高度；如果输入有第三列高度，则使用该列，也可以手动指定缺省高度：

```zsh
ros2 run g60_driver g60_transform_convert \
  --transform data/gps_odom_transform.txt \
  --input-file data/trajectories.txt \
  --output-file data/xyz.txt \
  --default-altitude 22.6
```

Python 调用：

```python
from g60_driver.transform_io import GpsOdomTransform

transform = GpsOdomTransform.load('data/gps_odom_transform.txt')
world_xyz = transform.gps_lla_to_world(latitude, longitude, altitude)
latitude, longitude, altitude = transform.world_to_gps_lla(x, y, z)
```

## 手动调整 OVJSN 和变换

自动对齐结果需要微调时，使用 `g60_transform_adjust` 同时处理一份 OVJSN 和一份 TXT。输入的 `dx`、`dy` 单位为米，`yaw-deg` 单位为度，三者都在 `world` 坐标系中解释：

```zsh
ros2 run g60_driver g60_transform_adjust \
  --ovjsn-input data/fix_from_odom_trajectory.ovjsn \
  --transform-input data/gps_odom_transform.txt \
  --ovjsn-output data/fix_from_odom_trajectory_adjusted.ovjsn \
  --transform-output data/gps_odom_transform_adjusted.txt \
  --dx 0.5 --dy -0.2 --yaw-deg 1.5
```

程序会把输入 OVJSN 的每个经纬度先转换到旧 TXT 的 `world` 坐标，再应用这组三参数，最后使用新 TXT 反算经纬度。因此输出的 OVJSN 和 TXT 是匹配的一对；原始文件不会被覆盖。

## 术语

- GGA：定位、卫星数、定位质量和 HDOP。
- RMC：定位有效性、地面速度、地面航向、日期和时间。
- VTG：对地速度和对地航向。
- HDT：真北航向。
- ENU：East、North、Up，即东、北、上。这里用于将经纬度转换为附近区域内以米计的局部坐标。
