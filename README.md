# XHand 双相机标定

面向 **Intel RealSense D405 腕部相机 + D435 全局相机 + Universal Robots UR5e** 的独立相机标定工具。

现场建议使用 Ubuntu 22.04。Python 支持 3.10–3.12。工具只读取 UR5e 状态，不发送运动指令；机械臂通过示教器停稳后采样。不依赖 ROS、Isaac Sim 或 CUDA；只有仿真接入示例需要 Isaac Sim 5.0。

本仓库提供可执行的图像采集、求解和验证流程，已通过合成图像测试。**尚未在你的 D405、D435 和 UR5e 上做硬件联调，也未获得你的真实标定结果。**

## 流程与产物

```text
按序列号识别 D405 / D435，固定 RGB 输出模式
    ↓
生成并实测 ChArUco 标定板
    ↓
分别采集多姿态图片 → 内参 K、畸变 D、留出图片误差
    ↓
固定标定板；D405 随 UR5e 移动，停稳后记录图片 + TCP 位姿
    ↓
手眼求解 T_TCP_Cw + 标定板基座位姿 T_B_Q
    ↓
D435 看同一块未移动的板 → 全局外参 T_B_Cg
    ↓
分别采集独立验证数据 → 误差报告 + 角点叠加图
    ↓
导出双相机配置，确认仿真连杆与 TCP 关系后接入 Isaac Sim
```

- `Cw`：D405 RGB 光学坐标系；`Cg`：D435 RGB 光学坐标系。
- `B`：安装 D405 的 UR5e 基座；`TCP`：标定期间固定配置的 TCP。
- `Q`：ChArUco 板坐标系。
- **`T_A_B` 将 B 中坐标变换到 A，位置单位为米，使用列向量。**
- OpenCV 光学坐标：X 向右，Y 向下，Z 向前。
- 当前只处理 RGB 相机；不做深度标定、RGB/深度对齐、自动机器人运动或连续运动中的硬件同步。

## 1. 安装到现场电脑

将仓库复制到连接相机和机器人的 Ubuntu 主机。以下从仓库根目录执行：

```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip libusb-1.0-0 usbutils

python3 -m venv .venv
source .venv/bin/activate
unset PIP_CONSTRAINT

python -m pip install --upgrade pip --index-url https://pypi.org/simple
python -m pip install -e '.[hardware]' --index-url https://pypi.org/simple

python -m xhand_calib --help
```

不要装进已经跑通的 Isaac Sim 环境。`.[hardware]` 安装 OpenCV contrib、NumPy、SciPy、`pyrealsense2` 和 `ur_rtde`。同一环境只安装一种 OpenCV 包。

如果只在服务器离线求解，不连接设备：

```bash
python -m pip install -e . --index-url https://pypi.org/simple
```

离线命令不会导入 RealSense / RTDE SDK。安装包需要网络；没有自动下载固件、编译内核驱动或安装 ROS 的操作。

## 2. 识别设备并填写配置

```bash
python -m xhand_calib devices
lsusb -t

cp configs/example.json configs/local.json
```

编辑 `configs/local.json`：

1. 将 `cameras.wrist.serial` 填为 D405 序列号，`cameras.global.serial` 填为 D435 序列号。
2. 根据设备枚举结果设置每台相机实际支持的宽、高、帧率。示例中的 `640×480@30` 只是模板。
3. 填写 UR5e 控制箱 IP，保证现场电脑与它可通信。
4. 在示教器中查看当前激活 TCP，将法兰到 TCP 的偏置填入 `tcp_offset_flange_to_tcp_m_rotvec`。单位米/旋转向量弧度。**不要猜测成零。**
5. 核对后设置 `tcp_configuration_verified: true`；标定期间不改变激活 TCP。
6. 将 `board` 尺寸改为标定板的实测值。

示教器可能以毫米/其他姿态显示方式显示 TCP 偏置，填配置前先转换为本仓库规定的米和旋转向量。程序记录此偏置作为元数据，不能自动证明它与示教器一致。

读取一次机器人状态：

```bash
python -m xhand_calib robot-state --config configs/local.json
```

输出 `tcp_pose_m_rotvec` 和 `T_B_TCP`。UR 的 `[rx,ry,rz]` 是旋转向量，代码使用 `Rotation.from_rotvec` 转换，**没有把它当欧拉角，也没有把 TCP 默认为法兰**。

## 3. 生成和安装标定板

```bash
python -m xhand_calib board \
  --config configs/local.json \
  --out data/board/charuco.png
```

默认 7×10 方格，方格 25 mm，标记 18 mm，字典 `DICT_5X5_100`；棋盘主体应打印为 **175×250 mm**。PNG 不包含用于推断打印比例的保证，请在打印软件中指定实体尺寸、关闭自动缩放，并测量连续多个方格。附着在平整硬板上，避免反光、弯曲。

D405 近距离视野可能需要较小的板。若改变尺寸，先修改配置，再生成。内参阶段可以手持移动板；手眼、全局外参和对应验证阶段，必须保持板的位置不变。

`board.frame_id` 标识同一个板坐标系的固定安装位置，例如 `table_board_v1`。移动板后改 ID 并重做相关外参。**程序能检查 ID，但不能检测你是否在保持相同 ID 时实际移动了板。**

## 4. 采集并求解两台相机内参

分别采集 30–50 张。覆盖画面四周、多个距离、左右/上下倾斜，不要只正对镜头。采集前固定分辨率、裁剪、对焦设置；本工具保存原始 RGB 流转 BGR 后的无损 PNG，不进行缩放、裁剪、去畸变或深度对齐。

窗口中按 **空格保存**，按 **Q / Esc 结束**。`--count` 是保存数量目标，不会自动移动板或机器人。

```bash
python -m xhand_calib capture \
  --config configs/local.json --camera wrist --kind intrinsics \
  --out data/intrinsics_wrist --count 40

python -m xhand_calib capture \
  --config configs/local.json --camera global --kind intrinsics \
  --out data/intrinsics_global --count 40

python -m xhand_calib intrinsics \
  --session data/intrinsics_wrist \
  --out results/wrist_intrinsics.json

python -m xhand_calib intrinsics \
  --session data/intrinsics_global \
  --out results/global_intrinsics.json
```

每个采集 session 都保存相机出厂内参、型号、序列号、固件、流配置和开始时的曝光/增益等可读设置。RealSense 不同畸变模型的系数不能直接假定为 OpenCV 五参数。

求解器使用普通 OpenCV Brown-Conrady 五参数模型，从 ChArUco 图片重新拟合 K/D；每五张检测成功的图片留一张验证，不用它重拟合内参。留出图片只单独估计板位姿。它会输出每张误差、角点覆盖范围及未通过检测的样本，不把出厂参数当成真值。

最低 12 张可检测图像，建议至少 30 张。留出误差约 0.5–1 px 可作为初步检查目标，不能代替尺寸、边缘成像和独立实验验证。当前不支持鱼眼模型选择、出厂内参自动选优或高阶畸变模型搜索。

## 5. 固定标定板，采集 D405 手眼数据

将板固定到 D405 和 D435 都能清楚看到的位置，更新并保持 `board.frame_id`。从这一阶段开始直到外参验证结束，板不能移动。

```bash
python -m xhand_calib capture \
  --config configs/local.json --camera wrist --kind handeye \
  --out data/handeye_wrist --count 30
```

对每个姿态：

1. 用示教器改变机器人位置和朝向，让 D405 能看到足够角点。
2. 停稳，等待支架和图像稳定，再按空格。
3. 工具在获取图片前后读取 TCP 速度和关节位置，并跨过 `settle_seconds` 检查窗口。
4. 若运动超过阈值或角点不足，本次不保存。

采集多个旋转方向，不要只有平移或只绕一根轴旋转。建议 25–40 组，求解最低 10 组并检查旋转激励。

保存的数据包含 `T_B_TCP`、原始 TCP 旋转向量、关节角、前后状态、相机时间戳/时间域和主机接收时间。这是**停稳采样检查，不是硬件同步，也不是运动控制/安全停止功能**。

求解：

```bash
python -m xhand_calib handeye \
  --session data/handeye_wrist \
  --intrinsics results/wrist_intrinsics.json \
  --out results/handeye.json
```

默认 OpenCV PARK 初始化，再联合优化 `T_TCP_Cw`、`T_B_Q` 以减少多视图像素残差。可用 `--method tsai` 等比较，或 `--no-refine` 保留 OpenCV 原始解。PnP 误差超过 `--max-reprojection`（默认 2 px）的视图会明确记录为 rejected。

输出关系：

```text
T_B_Q = T_B_TCP @ T_TCP_Cw @ T_Cw_Q
```

结果中的 `training_consistency` 仅表示训练数据一致性，不能当作独立验证。

## 6. 求解 D435 全局外参

D435 支架与板都保持不动，拍摄同一块板，保存 5–10 张清晰图像：

```bash
python -m xhand_calib capture \
  --config configs/local.json --camera global --kind global \
  --out data/global_extrinsics --count 8

python -m xhand_calib global \
  --session data/global_extrinsics \
  --intrinsics results/global_intrinsics.json \
  --handeye results/handeye.json \
  --out results/global_extrinsics.json
```

工具拟合静止板相对于全局相机的共同位姿，再计算：

```text
T_B_Cg = T_B_Q @ inverse(T_Cg_Q)
```

这里的绝对精度继承手眼和机器人基座/板估计的误差。同一固定姿态重复拍摄不能独立验证尺度和所有相机参数。如果两台相机不能看到同一块固定板，本版本需要额外的已知板/板变换流程，不能把板随意搬动后沿用原结果。

## 7. 独立验证和叠加图

D405 使用未参与求解的 5–10 个新机器人姿态；板保持固定。D435 重新采集一组图片，支架和板不动。

```bash
python -m xhand_calib capture \
  --config configs/local.json --camera wrist --kind validation \
  --out data/validation_wrist --count 10

python -m xhand_calib capture \
  --config configs/local.json --camera global --kind validation \
  --out data/validation_global --count 6

python -m xhand_calib validate \
  --session data/validation_wrist \
  --intrinsics results/wrist_intrinsics.json \
  --result results/handeye.json \
  --out results/validation_wrist.json \
  --overlays results/overlays_wrist

python -m xhand_calib validate \
  --session data/validation_global \
  --intrinsics results/global_intrinsics.json \
  --result results/global_extrinsics.json \
  --out results/validation_global.json \
  --overlays results/overlays_global
```

叠加图：绿色圆为检测角点，红色十字为标定结果预测角点，连线表示误差。

默认检查每个验证视图的 RMS ≤2 px、板位姿差≤5 mm 和≤2°；有 rejected 图片也判失败，不静默去掉坏样本。可以通过 `--max-reprojection`、`--max-translation`（米）、`--max-rotation`（度）调整。**阈值是初步检查值，不是设备精度承诺。**

退出码：0 成功；1 输入/运行错误；2 验证未通过；130 用户中断。

全局验证仍然引用原固定板，属于一致性检查。需要外部测量的其他目标才能更充分验证绝对定位精度。平面 PnP 在接近正对、远距离或角点很小时可能不稳定，采集时增加合理倾角和图像覆盖。

## 8. 去畸变与导出

建议先让真机去畸变图像与仿真匹配。输出保持全分辨率，不裁剪 ROI；`K_new` 和零畸变参数另存为旁边的 JSON。

```bash
python -m xhand_calib rectify \
  --image data/validation_wrist/images/0000.png \
  --intrinsics results/wrist_intrinsics.json \
  --out results/wrist_rectified.png --alpha 0

python -m xhand_calib export \
  --wrist-intrinsics results/wrist_intrinsics.json \
  --global-intrinsics results/global_intrinsics.json \
  --handeye results/handeye.json \
  --global-result results/global_extrinsics.json \
  --out results/camera_rig.json
```

导出包含两台相机的 raw K/D、rectified K/D、分辨率、`T_B_Cg`、`T_TCP_Cw`。不提供连杆变换时仍可导出，但标记 `ready_for_isaac_wrist=false`。

不能假定真机 TCP 等于仿真的 `wrist_3_link`。确认该关系后：

```bash
cp configs/link_to_tcp.example.json configs/link_to_tcp.local.json
# 编辑：填写真实 T_link_tcp 和 reference_link，确认后 verified=true。

python -m xhand_calib export \
  --wrist-intrinsics results/wrist_intrinsics.json \
  --global-intrinsics results/global_intrinsics.json \
  --handeye results/handeye.json \
  --global-result results/global_extrinsics.json \
  --link-to-tcp configs/link_to_tcp.local.json \
  --out results/camera_rig_isaac.json
```

`T_link_tcp` 把 TCP 坐标变换到指定仿真连杆坐标；由模型中连杆/法兰关系与示教器 TCP 偏置组合得到，例子里的单位阵仅为占位。

接入代码和步骤见 [docs/SIM_INTEGRATION.md](docs/SIM_INTEGRATION.md)，数据定义见 [docs/DATA_FORMAT.md](docs/DATA_FORMAT.md)。当前导出格式不是原项目 `eye/target` JSON，不能直接传给原版 `--camera-config`。

## 9. 无硬件演示与测试

```bash
python -m unittest discover -s tests -v

# 一条命令走完图像生成、内参、手眼、全局、验证、去畸变、导出。
python scripts/offline_demo.py --out data/offline_demo

# 生成可人工检查的合成图片和样本；目录必须不存在。
python -m xhand_calib.synthetic --out data/synthetic_demo
```

可以将上面命令中的各 session 替换为合成目录中的 `intrinsics_wrist`、`intrinsics_global`、`handeye`、`global`、`validation_wrist`、`validation_global`，离线走通整个 pipeline。

测试包括角点检测、内参留出验证、手眼/全局求解、错误外参检测、连杆占位检查、去畸变、命令退出码以及配置不一致检查。合成数据来自理想平面投影，不模拟真实镜头、USB 延迟、滚动快门、机械误差或反光；不能作为真机精度证据。

本次交付的实际测试环境、误差结果和未验证部分见 [docs/TESTING.md](docs/TESTING.md)。

所有输出使用新路径，默认拒绝覆盖，不会清理旧数据。新开终端后进入仓库并执行 `source .venv/bin/activate` 即可。现场数据、结果、虚拟环境和 `configs/local.json` 已在 `.gitignore` 排除。

## 常见问题

| 现象 | 检查方向 |
|---|---|
| 找不到相机 | USB 线/端口、设备占用、`lsusb -t`、RealSense udev 权限 |
| 设备可见但启动失败 | 查询支持的流模式；关闭 RealSense Viewer 等占用程序 |
| 非 root 无权限 | 按 librealsense 官方说明安装 udev 规则，不必修改相机固件作为第一步 |
| `cv2.aruco` 不存在 | 在独立环境仅安装 opencv-contrib-python |
| 远程无窗口 | 可以 `--headless --count N --interval 3`，但需人工改变板/机器人姿态，不能重复同一姿态凑数 |
| TCP 配置未验证 | 阅读示教器激活 TCP，填写偏置并明确确认，不把法兰/TCP混用 |
| 手眼旋转激励不足 | 增加绕不同方向的腕部旋转；不是只平移 |
| 内参更改后外参命令拒绝 | 内参已影响 PnP；用新内参重新求解相关外参 |
| 全局结果和腕部不一致 | 板是否移动、基座是否选错、TCP 是否改变、PnP/图像质量 |
| 输出路径已存在 | 使用新的 session/结果文件名；保留旧记录方便比较 |

SDK 参考：[librealsense](https://github.com/IntelRealSense/librealsense)、[UR RTDE](https://sdurobotics.gitlab.io/ur_rtde/)、[OpenCV calib3d](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)。
