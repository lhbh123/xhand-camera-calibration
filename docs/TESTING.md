# 交付验证记录

日期：2026-10-22。

本次使用本机已有依赖执行验证，没有安装或下载 RealSense、UR SDK，没有连接或移动真实机器人。

## 执行环境

| 项目 | 版本 |
|---|---|
| Python | 3.11.17 |
| NumPy | 1.26.0 |
| SciPy | 1.15.3 |
| OpenCV contrib | 4.11.0 |

项目声明支持 Python 3.10–3.12；本次没有逐一测试其他 Python 版本。

## 已执行

```bash
python -m unittest discover -s tests -v
python scripts/offline_demo.py --out /tmp/xhand-camera-calibration-final-demo
python -m xhand_calib --help
python -m xhand_calib board --config configs/example.json --out /tmp/xhand-camera-calibration-delivery-board.png
python -m compileall -q src examples scripts tests
python -m pip wheel . --no-deps --no-build-isolation --no-cache-dir --wheel-dir ../xhand-camera-calibration-dist
```

源码尚未安装时，上述 Python 命令使用 `PYTHONPATH=src`。构建 wheel 不下载依赖，也不修改原有仿真 Python 环境。

16 项 unittest 通过，覆盖：

- UR 旋转向量、变换求逆、OpenCV 光学轴到 USD 相机轴。
- 单轴退化运动、相机序列号不一致、TCP 参考不一致的拒绝。
- 停稳检查逻辑、未确认 TCP 在连接前拒绝。
- 渲染的 ChArUco 图片角点检测、内参恢复和留出图片检查。
- 手眼与全局外参对已知合成真值的恢复。
- 独立验证报告和叠加图片。
- 人为错误外参的验证失败。
- 未确认的连杆变换占位值拒绝。
- 去畸变、保持图片尺寸、避免覆盖。
- CLI 导出与验证失败退出码。
- 标定板 ID 不一致、内参变更的拒绝。
- 移动结果文件后不依赖原机器绝对路径。

## 离线演示结果及局限

演示生成两组各 32 张内参图片、26 张手眼图片、5 张全局图片、8 张腕部验证图片和 6 张全局验证图片，调用真实的 OpenCV 检测与 CLI 求解，没有替换为预先保存的角点真值。

| 指标 | 此次合成数据结果 |
|---|---|
| 腕部验证预测投影 RMS，逐图最大值 | 约 0.415 px |
| 全局验证预测投影 RMS，逐图最大值 | 约 0.391 px |
| 手眼安装变换与合成真值的平移差 | 约 0.677 mm |
| 手眼安装变换与合成真值的旋转差 | 约 0.271° |
| 全局安装变换与合成真值的平移差 | 约 8.260 mm |
| 全局安装变换与合成真值的旋转差 | 约 0.521° |

这些结果证明命令流程可以运行，并不代表达到某个真机精度等级。合成图片包含栅格采样误差，五参数内参拟合仍产生较大的径向系数提示；全局结果还继承腕部/板基座估计误差。尤其需要注意：**同一块静止板的低像素一致性误差可以与更大的绝对位姿误差同时存在。**

因此仓库明确区分训练一致性、固定板独立图片检查和绝对精度。实际使用时应增加边缘覆盖、合理距离/倾角，检查内参是否过拟合，并通过其他已测量目标验证绝对定位。不要仅以 `passed=true` 宣称毫米级精度。

没有执行的部分：

- D405/D435 的 USB/流模式/固件现场联调。
- Ubuntu 22.04 用户权限和 SDK 安装联调。
- UR5e RTDE 网络连接、实际 TCP 配置和真实停稳采集。
- 真机手眼和全局标定。
- Isaac Sim 相机渲染投影精度检查；适配器仅按安装的 5.0 API 编写并做语法检查。
