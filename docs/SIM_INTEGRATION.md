# 接入现有 tweezers 仿真

目标项目是 `tweezers_xhand_w01/tweezers`，现有相机位于 `src/probe_render.py`。本仓库不自动改写该项目，也不重新训练 PPO。

## 先完成真实标定

完成 README 的独立验证后，准备：

1. `results/camera_rig_isaac.json`，其中 `ready_for_isaac_wrist=true`。
2. 仿真机器人**同一个实际基座坐标系**在世界中的 `T_world_base`。
3. `reference_link`，通常是当前项目可读取的 `wrist_3_link`，但以模型为准。

真机配置 TCP 不自动等于 `wrist_3_link`。如果模型已知 `T_link_flange`，示教器偏置为 `T_flange_tcp`，则：

```text
T_link_tcp = T_link_flange @ T_flange_tcp
```

URDF、控制器 base/base_link 的约定也可能不同。`T_world_base` 应对应真实控制器位姿使用的基座坐标系，不能只看名称相似就设为单位阵。克隆环境下，全局相机的位置也需要包含选定环境的平移。

## 使用适配器

将 `examples/isaac_camera_rig.py` 复制到目标项目的 `src/`。它只依赖仿真环境已有的 NumPy、SciPy、Isaac Sim 相机 API，不需要在仿真环境安装 RealSense、RTDE 或 OpenCV。

必须先启动 `SimulationApp`，再导入/构造适配器。`scene` 初始化并建立机器人状态后：

```python
from isaac_camera_rig import CalibratedCameraRig

# 由你的场景坐标配置计算，不要凭空填写单位阵。
T_world_base = measured_robot_base_transform_in_sim_world

rig = CalibratedCameraRig(
    rig_path="/absolute/path/to/camera_rig_isaac.json",
    scene=scene,
    T_world_base=T_world_base,
    image_mode="rectified",
)

# 仿真循环中：完成 physics/robot state update 后，再渲染。
rig.update(env_id=0)
scene.sim.render()
images = rig.rgb()  # {'global': RGB ndarray, 'wrist': RGB ndarray}
```

这是与现有 `scene` 对象衔接的代码片段，不是可以脱离 SimulationApp 单独运行的脚本。创建后可能需要按原项目方式 render 几帧预热，再取图。

不再对腕部相机执行原来的 `follow_tip`。适配器每帧读取指定连杆的世界位置/四元数：

```text
T_world_wrist_camera = T_world_link @ T_link_Cw
T_world_global_camera = T_world_base @ T_B_Cg
```

默认 `rectified` 与 README 的 `rectify --alpha 0` 相匹配：相同分辨率、新 K、零畸变、不裁剪。若选择 `raw`，适配器使用拟合的 OpenCV K/D。

## 关于 Isaac Sim 内参设置

适配器按照标定值设置焦距/水平和垂直 aperture，并调用 Isaac Sim 5.0 的 `set_opencv_pinhole_properties` 设置 fx/fy/cx/cy 和畸变。OpenCV 光学轴转换为 USD 的 +Y 向上、-Z 观察方向，四元数使用 Isaac 的 wxyz。

该文件按照当前机器安装的 Isaac Sim 5.0 API 编写，但尚未在真实标定结果和渲染场景中验证投影精度。接入后应渲染同尺寸标定板，比较角点投影，检查：

- 中心和四角是否一致；
- 非中心主点是否正确生效；
- 画面是否上下翻转或朝反方向；
- 转动腕部时视图是否随连杆正确旋转；
- 镜头近裁剪面是否切掉近距离物体。

若底层渲染器对 fx/fy 或镜头畸变属性的处理与你的预期不同，以标定板投影验证为准，不通过缩放物体来补偿相机错误。

## 同步与保存

每次物理更新后更新腕部位姿，再执行一次 render，随后取两路图像。两路真实相机和仿真数据的帧率可以分别记录，不应把 30 FPS 视频与控制频率自动视为一一对应。

`rig.rgb()` 返回 RGB；若用 OpenCV 保存：

```python
cv2.imwrite("global.png", cv2.cvtColor(images["global"], cv2.COLOR_RGB2BGR))
```

原来的 ProbeRenderer 主要负责视频诊断。正式数据采集建议另外保存两路 PNG、同一时刻的仿真状态和相机位姿；本仓库提供相机对齐适配器，没有自动修改原 demo 的数据集格式。
