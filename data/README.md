# data/ 目录说明

本目录存储 GNSS 与里程计的坐标对齐变换矩阵文件以及轨迹导出文件。

## 1. 核心标定变换文件（Baseline Transforms）
- `g60_gps_odom_transform.txt`: G60 单点伪距 GNSS 与里程计对齐标定参数（普通最小二乘拟合）。
- `g90_gps_odom_transform.txt`: G90 (和芯星通 UM982) 高精 RTK 与里程计对齐标定参数（基于真实协方差加权拟合）。
- `d1m_gps_odom_transform.txt`: D1M RTK 与里程计对齐标定参数（基于真实协方差加权拟合）。
- `gps_odom_transform.txt`: 默认/向后兼容标定文件。

## 2. 轨迹记录文件（Trajectory Exports）
- `gps_trajectory.ovjsn`: GPS 原始轨迹导出文件（OvSerial 格式）。
- `fix_odom_trajectory.ovjsn`: 经过里程计对齐计算后的 GNSS 轨迹。
- `fix_from_odom_trajectory.ovjsn`: 里程计通过标定逆变换反算推导出的 GNSS 轨迹。
- `d1m_rtk_trajectory.ovjsn` / `d1m_fix_odom_trajectory.ovjsn`: D1M 对应的轨迹记录。

## 3. 工具与测试数据
- `trajectories.txt` / `xyz.txt`: `gnss_transform_convert` 离线批量坐标系转换工具的示例输入与输出数据。
- `gps_odom_transform_adjusted*.txt`: 经 `gnss_transform_adjust` 手动微调后的历史标定对比文件。
