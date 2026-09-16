# gnss_driver

`gnss_driver` 是 ROS 2 Jazzy 的通用 GNSS 功能包。包名不再绑定某一种接收机，设备差异由适配层和 YAML 参数描述；GPS/RTK 输出、局部坐标、里程计对齐、轨迹文件、TF、RViz 和 Nav2 导航使用同一套处理逻辑。

## 处理链

```text
G60/NMEA、G90/Unicore、D1M/UniRtkPvh
              ↓
       统一 NavSatFix/标准 ROS 话题
              ↓
     对齐、坐标变换、轨迹、TF、Nav2
```

当前已支持：

- G60：串口 NMEA，解析 GGA、RMC、VTG、GST、HDT；支持 GN/GP/GL/BD/IN talker。
- D1M：订阅 `robots_dog_msgs/msg/UniRtkPvh`（默认 `/rtk_pvh`），使用 `bestnav` 转换为 `NavSatFix` 和 `Odometry`。
- G90：串口解析 UM982/G90 的 `#PVTSLNA`、`#BESTNAVA`，发布标准 `/fix` 和 `robots_dog_msgs/msg/UniRtkPvh`（默认 `/rtk_pvh`）。
- GPS 与里程计二维刚体对齐；RTK 高精设备（G90、D1M）利用真实标准差进行 Huber 鲁棒加权拟合，单点设备（G60）使用普通最小二乘拟合。
- LLA↔ENU/world 正反解、`/fix_from_odom`、动态/静态 TF、OVJSN 轨迹导出。
- `inspection_interfaces/action/SetGPSGoal` 到 Nav2 `NavigateToPose` 的桥接（包括取消转发）。

## 目录与架构

- `gnss_driver/nodes`：面向对象分层节点实现
  - `base_gnss_node.py`：**顶层业务基类**，封装通用的 LLA 经纬高校验、ENU 协方差矩阵对角线计算、标准 `/fix` 及 `/rtk_pvh` 发布接口。
  - `base_serial_node.py`：**通信基类**，继承自业务基类，封装串口打开、定时轮询、断线自动重连与行接收逻辑。
  - `g60_node.py`：G60 单天线 NMEA 驱动（`G60DriverNode`）。
  - `g90_node.py`：G90 双天线 RTK Unicore/NMEA 驱动（`G90DriverNode`）。
  - `d1m_bridge_node.py`：D1M RTK 话题转发桥接（`D1MBridgeNode`，无串口依赖）。
  - `ntrip_client_node.py`：Linux 原生 NTRIP 客户端，连接 CORS 并将 RTCM3 注入到 G90 UART2。
- `gnss_driver/adapters`：专用硬件与网络协议解析（`g90_unicore.py`、`ntrip.py`）。
- `gnss_driver/coordinates`、`estimators`：ENU 和 SE(2) 拟合算法。
- `config/devices`：各设备驱动参数；`config/*_alignment.yaml` 与 `config/*_transform.yaml`：对齐与变换参数；`config/ntrip.yaml`：CORS 差分基准站配置；`trajectory.yaml`：轨迹记录配置。
- `launch`：通用 `driver.launch.py`、`alignment.launch.py`、`transform.launch.py`。
- `data`：标定变换与轨迹文件（详见 `data/README.md`）。

## 编译

```zsh
source /opt/ros/jazzy/setup.zsh
# 如使用 SetGPSGoal，再 source inspection_interfaces 所在工作区
cd ~/Workspace/driver_ws
colcon build --packages-select gnss_driver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source install/setup.zsh
```

## 三个主要启动入口

### 1. 设备输入

G60 串口驱动（普通 GPS 单天线，亦可通过 `ros2 run gnss_driver g60_driver` 独立运行）：

```zsh
ros2 launch gnss_driver driver.launch.py device:=g60
```

G90 串口驱动（UM982 双天线 RTK 高精度终端，亦可通过 `ros2 run gnss_driver g90_driver` 独立运行）：

```zsh
# 单独启动 G90 驱动
ros2 launch gnss_driver driver.launch.py device:=g90

# 伴随启动 Linux 原生 NTRIP 客户端（自动连千寻/CORS，通过 /dev/wheeltec_rtk 注入 RTCM3 差分流）：
ros2 launch gnss_driver driver.launch.py device:=g90 ntrip:=true
```

亦可单独运行 NTRIP 差分客户端：

```zsh
ros2 run gnss_driver ntrip_client --ros-args --params-file src/gnss_driver/config/ntrip.yaml
```

D1M 桥接（RTK 话题桥接，亦可通过 `ros2 run gnss_driver d1m_bridge` 独立运行）：

```zsh
ros2 launch gnss_driver driver.launch.py device:=d1m
```

#### 串口别名与权限配置 (udev)

* 主串口（G60/G70/G90 主 Type-C 接口，`1a86:55d4`）：映射为 `/dev/wheeltec_gnss`。
* RTK 差分注入串口（G90 串口 2 CH340 232转USB，`1a86:7523`）：映射为 `/dev/wheeltec_rtk`。

安装别名规则（两端口均自动赋予 `0666` 免提权权限）：

```zsh
# 在包源码根目录或安装目录下直接执行：
sudo sh wheeltec_gnss.sh
sudo usermod -aG dialout "$USER"
```

重新插拔设备或重新登录后生效。可以通过 `ls -l /dev/wheeltec_*` 验证软链接。

#### 串口波特率配置

驱动端的波特率由设备 YAML 中的 `baud` 参数控制，代码不限制为某一个固定值，
例如 G90 默认是 115200，G60 默认是 9600。修改对应文件即可：

```yaml
# config/devices/g90.yaml
gnss_device:
  ros__parameters:
    baud: 230400
```

修改后重新启动节点（无需重新编译）。也可以在直接运行节点时临时覆盖：

```zsh
ros2 run gnss_driver g90_driver --ros-args -p baud:=230400
```

注意：接收机 UART 的输出波特率必须先通过厂家配置工具/串口命令改成相同值，
然后再修改 YAML；只改 ROS 参数会导致乱码或完全收不到数据。修改接收机波特率后，
重新插拔或重启接收机使设置生效。G90 的 `/dev/wheeltec_rtk` 是 NTRIP 注入的
另一物理串口，其速率由 `config/ntrip.yaml` 的 `rtk_baud` 单独控制，与主串口
 `baud` 无关。

波特率修改建议使用下面的一次性配置工具；运行驱动本身不会修改接收机 UART，
只会读取 YAML 中的 `baud` 参数。这样可避免每次重启时重复改写设备配置。

推荐使用一次性配置工具完成修改、保存和校验：

```zsh
ros2 run gnss_driver gnss_g90_configure \
  --port /dev/wheeltec_gnss --current-baud 115200 \
  --baud 460800 --output-port COM1
```

工具会用旧速率发送输出配置和 `CONFIG`，再以新速率重连，发送 `SAVECONFIG` 并执行
`CONFIG` 查询。终端显示“配置成功”且回显目标波特率才算验证通过。完成后把
`config/devices/g90.yaml` 的 `baud` 改为新值（例如 `460800`）。

脚本只配置 `--output-port` 指定的一个接收机端口，并先执行该端口的 `UNLOG`，
不会同时发送无端口前缀和 `COM1` 两套订阅，因此不会主动制造重复报文。

### 2. GPS/里程计对齐

支持通过 `device` 参数（`g60`、`g90`、`d1m`）自动加载对应的参数文件（`config/<device>_alignment.yaml`）：

```zsh
# G60 单点伪距（30个样本，普通最小二乘，输出 g60_gps_odom_transform.txt）
ros2 launch gnss_driver alignment.launch.py device:=g60 start_driver:=false export_polyline:=true

# G90 (UM982) RTK 高精（200个样本，基于真实协方差加权拟合，输出 g90_gps_odom_transform.txt）
ros2 launch gnss_driver alignment.launch.py device:=g90 start_driver:=false export_polyline:=true

# D1M RTK 桥接（200个样本，基于真实协方差加权拟合，输出 d1m_gps_odom_transform.txt）
ros2 launch gnss_driver alignment.launch.py device:=d1m start_driver:=false export_polyline:=true
```

输入 `/fix` 与 `/odometry_horizon`，输出 `/fix_odom`、`world` 坐标系 Path，并写入 `data/<device>_gps_odom_transform.txt`。

重置：

```zsh
ros2 service call /reset_alignment std_srvs/srv/Trigger '{}'
```

### 3. 使用已有变换

同样支持通过 `device` 参数直接加载对应变换配置（`config/<device>_transform.yaml`）：

```zsh
# 加载 G90 变换并开启 RTK 辅助里程计输出
ros2 launch gnss_driver transform.launch.py device:=g90 export_polyline:=true

# 加载 D1M 变换
ros2 launch gnss_driver transform.launch.py device:=d1m export_polyline:=true

# 加载 G60 变换
ros2 launch gnss_driver transform.launch.py device:=g60 export_polyline:=true
```

读取 `transform_path` 指向的 TXT，将 `/odometry_horizon` 反算为 `/fix_from_odom`，发布 `world -> gps` 动态 TF 和供 `rviz_satellite` 使用的 `world -> map` 静态 TF。对具备 RTK 数据源的设备（G90、D1M），同时可发布 `/odometry_from_rtk`。RViz Fixed Frame 设为 `world`，AerialMap 使用 `/fix_from_odom`。

## 坐标与离线工具

变换文件记录 GPS 原点、二维旋转 `R` 和平移 `t`：

```text
world_xy = R * gps_enu_xy + t
gps_enu_xy = transpose(R) * (world_xy - t)
```

```zsh
# --transform 参数可指定各设备的标定文件，如 data/g90_gps_odom_transform.txt、data/d1m_gps_odom_transform.txt 或通用文件
ros2 run gnss_driver gnss_transform_convert --transform data/g90_gps_odom_transform.txt --lla 30.0 120.0 10.0
ros2 run gnss_driver gnss_transform_convert --transform data/g90_gps_odom_transform.txt --xyz 1 2 0
ros2 run gnss_driver gnss_transform_convert --transform data/g90_gps_odom_transform.txt --input-file data/trajectories.txt --output-file data/xyz.txt
```

`gnss_transform_adjust` 可同时微调 TXT 和 OVJSN，原文件不会覆盖。

## 已知限制

- G90 的 GPS 周/周内秒、位置、位置标准差和速度来自 UM982 专有报文；定位质量、定位类型和卫星数优先来自同一串口的 GGA；`$GNHPR` 的航向/俯仰写入 `UniRtkPvh.heading`。原始报文没有提供或当前参考驱动未证明来源的字段保持 `0`、`NaN` 或“未解算”，不伪造 RTK fixed 状态。
- `/fix` 是接收机自身定位结果，不是融合定位；`/fix_odom`、`/fix_from_odom` 是由里程计和变换反算的 GPS。
- ENU 使用局部小范围近似，适合车辆作业区域，不适合跨大区域测量。
