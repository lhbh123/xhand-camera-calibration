# 数据、坐标系与结果格式

## 采集 session

```text
session/
  session.json
  samples.jsonl
  images/
    0000.png
    0001.png
```

`session.json` 包含 `schema_version=1`、`kind`、`camera_name`、`camera`、`board`，手眼/腕部验证时还有 `robot`。图像为 OpenCV BGR 的无损 PNG；普通图片查看器按 PNG 格式正确显示颜色。

`camera` 的序列号与图像尺寸会在求解/验证时与内参结果比对。采集配置和 RealSense 出厂内参一起存档；出厂畸变模型不直接用于 OpenCV PnP。

`samples.jsonl` 每行一个 JSON。手眼样本示意（缩略）：

```json
{
  "sample_id": "0000",
  "image": "images/0000.png",
  "camera_timestamp_ms": 123456.7,
  "camera_timestamp_domain": "timestamp_domain.hardware_clock",
  "camera_frame_number": 42,
  "host_receive_time_ns": 1000000000,
  "host_receive_monotonic_ns": 2000000000,
  "detected_corners": 45,
  "stationary_checked": true,
  "robot": {
    "host_time_ns": 1000000010,
    "tcp_pose_m_rotvec": [0.4, -0.1, 0.5, 0.1, 0.2, 0.3],
    "joint_positions_rad": [0, 0, 0, 0, 0, 0],
    "tcp_speed": [0, 0, 0, 0, 0, 0],
    "T_B_TCP": [[1,0,0,0.4],[0,1,0,-0.1],[0,0,1,0.5],[0,0,0,1]]
  },
  "robot_before": {}
}
```

上例只说明键名，不是有效的标定样本（矩阵和旋转值未配对）。不要把示例数值送入求解器。

采集顺序是：读取 `robot_before` → 等待稳定检查窗口 → 接收最新图片 → 读取 `robot`。两端的速度和关节变化符合阈值才保存。它无法保证图片与位姿在连续运动时严格同步，因此仅用于停稳采集。

相机硬件时钟、主机墙上时间和主机单调时钟是不同时间域。墙上时间只用于记录，单调时钟用于主机内排序；不直接将相机时间与主机时间相减来计算延迟。

## 坐标变换

`T_A_B` 表示 B 到 A 的变换：`p_A = T_A_B @ p_B`。所有 4×4 变换要求底行为 `[0,0,0,1]`，旋转在 SO(3)，平移单位为米。

UR `[x,y,z,rx,ry,rz]` 中后三个是旋转向量，不能当成 Euler 角。OpenCV `solvePnP` 输出 `T_camera_board`。

手眼使用的关系：

```text
A_i = T_B_TCP_i
X   = T_TCP_Cw
B_i = T_Cw_Q_i
Y   = T_B_Q

A_i @ X @ B_i = Y
```

手眼 `X` 是相机相对于配置 TCP 的安装变换，仍需明确 TCP 与仿真连杆的变换：

```text
T_link_Cw = T_link_tcp @ T_TCP_Cw
```

全局相机：

```text
T_B_Cg = T_B_Q @ inverse(T_Cg_Q)
```

ChArUco 板坐标由 OpenCV Board 定义，图像生成、角点检测和求解使用同一对象定义，不手工重新排列角点。更换字典、板尺寸、正反方向或移动板后，不能沿用同一外参关系。

## 结果文件

- `opencv_intrinsics`：K、D（k1,k2,p1,p2,k3）、图像尺寸、每张图像训练/留出误差、角点覆盖和拒绝列表。
- `handeye`：`T_TCP_Cw`、`T_B_Q`、初始化结果、优化信息、多轴旋转检查、训练一致性。
- `global_extrinsics`：`T_B_Cg`、`T_B_Q`、固定板共同位姿拟合结果。
- `validation`：通过与否、阈值、逐视图误差、拒绝样本及统计。
- `calibrated_camera_rig`：两路 raw/rectified 参数与安装外参，供仿真适配器读取。

外参文件嵌入当时使用的 K/D，验证和导出会检查是否一致，防止后续替换内参后继续使用旧外参。`*_source` 绝对路径用于溯源，不作为结果迁移后的加载依赖；可将相关结果 JSON 一起复制到其他电脑。

手眼与全局结果共享的板尺寸/ID、基座机器人配置和 `T_B_Q` 必须相同。程序无法通过一个 ID 证明实体板没有被移动，必须在现场保持它固定。

## 图像去畸变

`rectify` 使用 `cv2.getOptimalNewCameraMatrix`，`alpha` 默认 0。输出保留完整宽高，不裁剪 ROI。导出中的 rectified 参数使用相同计算。

如果后续主动裁剪 `[x,y,w,h]`，新主点变为 `(cx-x, cy-y)`；如果再缩放，焦距和主点按对应宽高比例更新。应始终记录这一预处理，不能仅改变训练图片而保持旧 K。

## 验证指标的含义

- `translation_m` / `rotation_deg`：当前验证样本反算的板位姿与标定板基准位姿的差。
- `predicted_reprojection_rms_px`：不重拟合相机安装外参，直接预测已知板角点，与检测角点比较的像素 RMS。
- PnP 自己的重投影误差用于输入质量筛选；它不是最终安装外参误差。

验证阶段有拒绝样本会判失败；低误差不等于已验证绝对毫米精度。机器人运动学、TCP 配置、板尺寸误差、平面姿态歧义、相机模型和支架形变都可能影响真实精度。
