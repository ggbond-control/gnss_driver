# g60_driver

g60_driver 是 G60 GNSS 接收机的 ROS 2 Jazzy 包。项目按三个运行阶段组织：采集原始 GPS、标定 GPS 与里程计变换、使用已有变换进行运行时转换和 GPS 目标导航。

## 功能总览

1. 原始 GNSS 采集
   - launch: launch/g60_serial.launch.py
   - 配置: config/g60_driver.yaml
- 功能: 读取 /dev/g60_gnss，解析 NMEA，发布 /fix、/vel、/heading、/time_reference 和原始 NMEA 调试话题 /nmea_sentence。

2. GPS/里程计标定
   - launch: launch/g60_alignment.launch.py
   - 配置: config/g60_alignment.yaml
   - 输入: /fix 和 /odometry_horizon
   - 功能: 使用 30 组有效运动样本计算 GPS ENU 到 world 的二维刚体变换。
   - 输出: /fix_odom、Path 轨迹、OVJSN 轨迹、data/gps_odom_transform.txt。

3. 使用已有变换
   - launch: launch/g60_transform.launch.py
   - 配置: config/g60_transform.yaml
   - 输入: /odometry_horizon 和 /set_gps_goal action
   - 功能: 将里程计反算为 GPS，或将 GPS 目标转换为 Nav2 导航目标。
   - 输出: /fix_from_odom、data/fix_from_odom_trajectory.ovjsn。

4. 离线工具
   - g60_transform_convert: LLA 与 XYZ 单点/批量转换。
   - g60_transform_adjust: 对 transform TXT 和 OVJSN 成对做 dx、dy、yaw 微调。

当前没有保留 UDP、TCP、NMEA 话题输入和旧局部轨迹节点。

## 编译

如果要使用 /set_gps_goal action，需要先 source inspection_interfaces 所在工作空间。

    source /opt/ros/jazzy/setup.zsh
    source ~/Workspace/task_ws/install/setup.zsh
    cd ~/Workspace/driver_ws
    colcon build --packages-select g60_driver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1
    source install/setup.zsh

常用系统依赖：

    sudo apt install ros-jazzy-tf-transformations python3-serial

## 设备准备

G60 使用 QinHeng USB 串口，VID:PID 为 1a86:55d4，Linux 设备节点为 /dev/ttyACM*。本包默认读取稳定链接 /dev/g60_gnss。

首次使用可安装 udev 规则：

    sudo bash "$(ros2 pkg prefix g60_driver)/share/g60_driver/scripts/install_udev_rule.sh"
    sudo usermod -aG dialout "$USER"

加入 dialout 后需要重新登录。sudo chmod 777 /dev/g60_gnss 只适合临时排查权限，重插设备后会失效。

## 启动 1：发布原始 /fix

启动：

    ros2 launch g60_driver g60_serial.launch.py

功能：

- 读取 /dev/g60_gnss。
- 解析 GGA、RMC、VTG、GST、HDT。
- 发布 /fix、/vel、/heading、/time_reference；可选发布 /nmea_sentence。
- 接受 GPS、组合 GNSS、GLONASS、北斗和惯导 Talker ID；当前未使用的 GSA/GSV 语句会忽略，不记录 warning。

默认配置在 config/g60_driver.yaml：

- port: /dev/g60_gnss
- baud: 9600
- frame_id: gps
- use_rmc_fix: false
- publish_raw_nmea: false（是否发布原始 NMEA 调试话题）
- raw_nmea_topic: /nmea_sentence（仅在 publish_raw_nmea 为 true 时生效）

查看接收机的原始输出时，不要直接读取 `/dev/g60_gnss`，否则会与驱动竞争串口数据。调试时启用原始话题：

在 `config/g60_driver.yaml` 中设置：

    publish_raw_nmea: true

然后使用：

    ros2 topic echo /nmea_sentence

只筛选组合定位和精度语句：

    ros2 topic echo /nmea_sentence | rg '^\$(GN|GP)GGA|^\$(GN|GP)GST'

/fix 是接收机自身解算后的 GNSS 结果；本节点只做格式转换，不做融合定位。

## 启动 2：GPS/里程计标定

启动：

    ros2 launch g60_driver g60_alignment.launch.py

输入：

- /fix: sensor_msgs/msg/NavSatFix
- /odometry_horizon: nav_msgs/msg/Odometry

输出：

- /fix_odom: sensor_msgs/msg/NavSatFix
- gps_trajectory_aligned: nav_msgs/msg/Path
- odometry_trajectory: nav_msgs/msg/Path
- gps_pose_aligned: geometry_msgs/msg/PoseStamped
- data/gps_trajectory.ovjsn
- data/fix_odom_trajectory.ovjsn
- data/gps_odom_transform.txt

标定逻辑：

- GPS 经纬度先转成局部 ENU 米制坐标。
- 与 /odometry_horizon 按时间戳配对。
- 只采集发生明显移动的样本。
- 默认采集 30 组有效样本。
- 求一次二维刚体变换：world_xy = R * gps_enu_xy + t。
- 变换锁定后不再继续拟合。

launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| start_g60_fix | false | 是否同时启动串口 GNSS。默认使用外部已有 /fix。 |
| export_polyline | true | 是否输出 /fix 和 /fix_odom 的 OVJSN。 |
| use_rviz | false | 是否启动 RViz。 |

如果希望本包同时发布 /fix：

    ros2 launch g60_driver g60_alignment.launch.py start_g60_fix:=true use_rviz:=true

如果 /fix 已由外部系统发布：

    ros2 launch g60_driver g60_alignment.launch.py use_rviz:=true

重置标定：

    ros2 service call /reset_alignment std_srvs/srv/Trigger '{}'

关键配置在 config/g60_alignment.yaml：

- fix_topic: /fix
- odom_topic: /odometry_horizon
- fix_odom_topic: /fix_odom
- output_frame: world
- calibration_pairs: 30
- min_calibration_displacement: 1.0
- max_time_delta: 0.15

## 启动 3：使用已有变换

启动：

    ros2 launch g60_driver g60_transform.launch.py

该启动项不读取实时 GPS，也不重新标定，只读取 transform TXT。

### 里程计反算 GPS

输入：

- /odometry_horizon: nav_msgs/msg/Odometry

输出：

- /fix_from_odom: sensor_msgs/msg/NavSatFix
- data/fix_from_odom_trajectory.ovjsn
- 动态 TF：`world -> gps`（准确名称取自 TXT 的 `output_frame` 与 `gps_frame`）。该 TF 的位姿直接来自 `/odometry_horizon`，与 `/fix_from_odom` 的经纬度反算使用同一个世界坐标位置。
- 静态 TF：`world -> map`。该 TF 使用 TXT 内的二维旋转和平移，表示 GPS 的东、北、天（ENU）坐标轴在 `world` 中的方向。Jazzy 版 `rviz_satellite` 固定使用名为 `map` 的 ENU 参考帧。

在 RViz 使用 `rviz_satellite/AerialMap` 时，将 Fixed Frame 设为 `world`，并将 AerialMap 的 Topic 设为 `/fix_from_odom`。Jazzy 版插件没有 ENU 参考帧属性，会自动使用 `map`。不要另外发布静态 `world -> gps` 或 `world -> map` TF；本节点已分别发布所需的动态与静态 TF，使卫星地图能和 `world` 下的点云、轨迹叠加。

默认读取：

- data/gps_odom_transform.txt
- 如果做过手动微调，可在 config/g60_transform.yaml 中改为 data/gps_odom_transform_adjusted.txt。

如果只需要发布 /fix_from_odom，不写 OVJSN：

    ros2 launch g60_driver g60_transform.launch.py export_polyline:=false

### GPS action 导航

g60_transform.launch.py 提供 /set_gps_goal action：

- action 类型: inspection_interfaces/action/SetGPSGoal
- 下游 action: /multi_map_navigate_to_pose
- 下游类型: nav2_msgs/action/NavigateToPose

调用示例：

    ros2 action send_goal /set_gps_goal inspection_interfaces/action/SetGPSGoal "{header: {frame_id: world}, latitude: 30.28892, longitude: 119.98098, altitude: 22.6, orientation: {w: 1.0}, skip_yaw_alignment: true}" --feedback

收到 GPS action goal 后，节点会：

1. 读取 goal 中的 latitude、longitude、altitude。
2. 通过 transform TXT 转换为 x、y、z。
3. 调用 /next_goal_policy 设置本次目标的最终 yaw 策略。
4. 构造 NavigateToPose.Goal。
5. 发送到 /multi_map_navigate_to_pose。
6. 等待 Nav2 goal 接受和最终 result。
7. 将 Nav2 result 转成 SetGPSGoal.Result。

NavigateToPose.Goal 当前填法：

- pose.header.frame_id = SetGPSGoal.Goal.header.frame_id
- pose.header.stamp = 当前 ROS 时间
- pose.pose.position.x/y/z = 经纬高转换后的坐标
- pose.pose.orientation = SetGPSGoal.Goal.orientation 归一化
- behavior_tree = 空字符串，使用 Nav2 默认行为树

skip_yaw_alignment 策略：

- true：调用 /next_goal_policy 设置 align_final_yaw=false，最终抵达位置后不要求对齐请求中的 yaw。
- false：调用 /next_goal_policy 设置 align_final_yaw=true，使用正常最终 yaw 对齐。
- 四元数本身始终会传递给 NavigateToPose；是否执行最终对齐由 multi_map_nav 的策略决定。
- 策略服务请求只设置 align_final_yaw，不设置 obstacle_policy，因此障碍策略恢复为 multi_map_nav 配置中的默认值。

结果与反馈：

- 只有 Nav2 action SUCCEEDED 且 error_code=0 时，SetGPSGoal.Result.success=true。
- Nav2 的 distance_remaining 会转发为 SetGPSGoal.Feedback.distance_remaining。
- 如果策略已设置后才取消 /set_gps_goal，节点仍会先发送对应的 /multi_map_navigate_to_pose goal 以消费一次性策略，再立即转发取消请求。
- /next_goal_policy 不可用、超时或拒绝时，/set_gps_goal 直接 ABORTED，不发送 Nav2 goal。

关键配置在 config/g60_transform.yaml：

- transform_path: ~/Workspace/driver_ws/src/g60_driver/data/gps_odom_transform.txt
- input_topic: /odometry_horizon
- output_topic: /fix_from_odom
- enable_gps_goal_action: true
- gps_goal_action: /set_gps_goal
- navigation_action: /multi_map_navigate_to_pose
- next_goal_policy_service: /next_goal_policy
- action_server_wait_sec: 5.0
- next_goal_policy_wait_sec: 5.0
- action_result_timeout_sec: 0.0

action_result_timeout_sec 为 0.0 表示一直等待 Nav2 最终结果。

## 离线工具 1：坐标正反解

单点 LLA 到 XYZ：

    ros2 run g60_driver g60_transform_convert --transform data/gps_odom_transform.txt --lla 30.0 120.0 10.0

单点 XYZ 到 LLA：

    ros2 run g60_driver g60_transform_convert --transform data/gps_odom_transform.txt --xyz 1.0 2.0 0.0

批量将 data/trajectories.txt 转换为 data/xyz.txt：

    ros2 run g60_driver g60_transform_convert --transform data/gps_odom_transform.txt --input-file data/trajectories.txt --output-file data/xyz.txt

trajectories.txt 每行格式：

- longitude,latitude
- longitude,latitude,altitude

xyz.txt 每行格式：

- x,y,z

如果输入没有高度列，默认使用 transform 文件中的 GPS 原点高度，因此输出局部 z=0。也可以指定默认高度：

    ros2 run g60_driver g60_transform_convert --transform data/gps_odom_transform.txt --input-file data/trajectories.txt --output-file data/xyz.txt --default-altitude 22.6

## 离线工具 2：手动微调变换

自动标定结果有偏差时，可对 transform TXT 和对应 OVJSN 成对微调。

    ros2 run g60_driver g60_transform_adjust --ovjsn-input data/fix_from_odom_trajectory.ovjsn --transform-input data/gps_odom_transform.txt --ovjsn-output data/fix_from_odom_trajectory_adjusted.ovjsn --transform-output data/gps_odom_transform_adjusted.txt --dx 0.5 --dy -0.2 --yaw-deg 1.5

参数含义：

- dx: world X 方向平移修正，单位米
- dy: world Y 方向平移修正，单位米
- yaw-deg: world 平面旋转修正，单位度

输出的 adjusted OVJSN 和 adjusted TXT 是匹配的一对；原始文件不会被覆盖。

## 输出文件

运行输出默认写入源码包 data/：

- data/gps_trajectory.ovjsn
- data/fix_odom_trajectory.ovjsn
- data/fix_from_odom_trajectory.ovjsn
- data/gps_odom_transform.txt
- data/gps_odom_transform_adjusted.txt
- data/xyz.txt

这些文件是运行产物，默认不纳入 git，也不会作为包资源安装；安装时只保留 data/.gitkeep 以创建目录。

## 常见问题

### /fix_from_odom 没有输出

检查：

    ros2 topic info -v /odometry_horizon
    ros2 topic echo /fix_from_odom --once

g60_transform 使用 qos_profile_sensor_data 订阅 /odometry_horizon，可接收常见的 Best Effort 里程计发布端。

### /set_gps_goal 返回 action server 未就绪

说明 /multi_map_navigate_to_pose 尚未启动或名称不一致。检查：

    ros2 action list | grep multi_map_navigate_to_pose

### 标定结果偏移

先确认 30 组样本中车辆有实际运动，且 GPS 与里程计时间戳接近。若仍有小偏差，使用 g60_transform_adjust 生成 adjusted TXT，并让 config/g60_transform.yaml 读取 adjusted 文件。

## 术语

- GGA：定位、卫星数、定位质量和 HDOP。
- RMC：定位有效性、地面速度、地面航向、日期和时间。
- VTG：对地速度和对地航向。
- HDT：真北航向。
- ENU：East、North、Up，即东、北、上；本包用它把经纬度换算为局部米制坐标。
