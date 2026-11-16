# 机器人智能操作作业1

Franka Panda 七轴机械臂用圆形擦垫沿五行蛇形轨迹擦拭水平工件。控制器以 120 Hz 跟踪平面轨迹，并根据接触力调整压入深度。第 7.3 秒，压力目标由 6 N 升至 10 N；第 12.4 秒恢复至 6 N。实验分别在 DGX Spark 的 Isaac Sim 5.1 和 Windows 的 MuJoCo 3.6.0 上运行。

## 查看实现

- [control.py](control.py) 定义七轴正运动学、雅可比矩阵、阻尼最小二乘逆解、五次插值轨迹和压力目标。
- [arm.py](arm.py) 在 Isaac Sim 中加载 Franka URDF，创建工件和擦垫，读取 PhysX 接触力并执行闭环控制。
- [mj_arm.py](mj_arm.py) 从同一 URDF 构建 MuJoCo 模型，运行力反馈与固定深度对照。
- 运行仿真后，Franka.csv、MuJoCo.csv、MuJoCo_fixed.csv 分别记录两平台的闭环试验及固定深度对照。t 的单位为秒，坐标和法向修正量的单位为米，压力单位为牛顿。raw_N 为原始力，filtered_N 为低通后的反馈力，covered 为累计覆盖点数。
- [edit.py](edit.py) 将原生窗口录像、两段引擎相机录像与各自的测量时序合成为提交视频。

## 复现

在仓库根目录运行。两端均需带 libx264 编码器的 FFmpeg。机器人资产来自 Isaac Sim 5.1 自带的 franka_description。

**Isaac Sim 5.1 / Linux**

```bash
export FFMPEG="$(command -v ffmpeg)"
~/isaacsim/python.sh arm.py
```

生成 Franka.csv、Franka_Isaac.mp4 和 Panda_Workcell.usdc。若需录制真实软件窗口，准备 Xvfb、xauth、xdpyinfo、xwininfo、xprop 和支持 x11grab 的 FFmpeg，然后运行下列命令。安装在其他目录时可通过 ISAAC_PYTHON 指定 python.sh。

```bash
PRIVATE_XVFB="$(command -v Xvfb)" bash launch.sh
```

**MuJoCo 3.6 / Windows PowerShell**

```powershell
python -m pip install mujoco==3.6.0 numpy scipy pillow
$env:FFMPEG = (Get-Command ffmpeg).Source
$env:FRANKA_DESCRIPTION = "C:/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description"
python mj_arm.py
$env:FIXED_DEPTH = "1"
python mj_arm.py
Remove-Item Env:FIXED_DEPTH
```

将 FRANKA_DESCRIPTION 改为实际资产目录。首次运行会从 DAE 网格生成 MuJoCo 使用的 OBJ。两次运行分别输出 MuJoCo.csv 和 MuJoCo_fixed.csv。合成视频时，把 Franka_Isaac.mp4、Isaac_Kit_UI.mp4、Franka_MuJoCo.mp4、Franka.csv、MuJoCo.csv 放在同一目录，将 RUN_DIR 指向该目录并执行 python edit.py。

