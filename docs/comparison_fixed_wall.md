# PCR 八机固定墙对比实验

六、八、十二、十六机的通用入口及三类场景命令见
[多机编队说明](multi_uav_formation.md)。本文保留原八机入口及历史验证记录。

本工作区以 `ecdfbcce61e71a4c4c45385a4c3401c12d7d4e57`
（初始版本：老师提供完整多机仿真源码6-30）为基线，增加八机场景入口。
参考仓库是 `~/Codes/EGO-Planner-criterion-validation` 的 `distributed` 分支，
参考版本为 `401e75cc9dbac8267c83a6af6544cd896f392222`。
`distributed` 是分支名，不是目录名。

## 编译与启动

在 zsh 终端中执行：

```zsh
cd ~/Codes/pcr_orign/flip_privite-main/flip_privite-main
source /opt/ros/noetic/setup.zsh
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
rospack find ego_planner
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch
```

`rospack find ego_planner` 应指向本工作区的 `src/planner/plan_manage`。
两个项目使用了相同的 ROS 包名，切换项目时在新终端中 source 对应工作区，
每次只运行一个实验。bash 用户将 `.zsh` 改为 `.bash`。

默认自动开始飞行并打开 RViz。先查看静止队形、再手动启动：

```zsh
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch auto_start:=false
```

等八架飞机及墙体出现后，在已经 source 本工作区的另一终端执行：

```zsh
rostopic pub -1 /traj_start_trigger geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}'
```

不打开 RViz 的运行方式是附加 `rviz:=false`。场景还支持 `start_x`、`goal_x`、
`max_vel`、`safety_distance` 以及参考入口同名的墙体几何参数。
默认值用于两项目对比；修改后应在参考项目中同步修改对应参数。

## 与参考场景一致的部分

| 项目 | 默认值 |
| --- | --- |
| 编队 | 两排四列，相邻槽位间隔 1 m |
| 槽位 0–3 | x=+0.5，y=−1.5/−0.5/+0.5/+1.5 |
| 槽位 4–7 | x=−0.5，y 同上 |
| 编队中心起点 / 终点 | (−6, 0, 1.5) / (10, 0, 1.5) m |
| 速度 / 加速度上限参数 | 0.5 m/s / 6 m/s² |
| 地图尺寸 / 栅格分辨率 | 24×14×3 m / 0.1 m |
| 传感器范围 / 水平视场 | 5 m / 360° |
| 障碍膨胀 / 障碍间距参数 | 0.2 m / 0.2 m |
| 机间间距参数 | 0.5 m |
| 第一面墙 | 中心 (−2, 0)，厚 0.5 m，宽 4 m |
| 第二面墙 | x=6.5 m，厚 0.5 m，总宽 10 m，中央开口 1.8 m |
| 墙体高度 | z=−0.1 至 3.0 m |
| 无人机外形 | `fake_drone.dae`，黑色，缩放 0.35，`tf45=true` |

以下资源直接复制自参考工作区：

- `map_generator/src/fixed_split_merge_wall_map.cpp`：保持原样，生成相同的
  24,000 个地图点和三个墙体长方体 Marker。
- `odom_visualization/meshes/fake_drone.dae`：保持原样。原显示程序与参考项目的
  差异只有默认模型路径，此入口通过参数指定相同模型。
- `launch/comparison_8uav.rviz`：相机、白色背景、网格、墙体、地图点云、0–7 号
  飞机及轨迹的显示配置来自参考 `default.rviz`。相机距离、焦点、俯仰和偏航
  参数完整保留；仅移除参考算法的专用显示项及 8 号以后的无人机。

RViz 相机数值：Distance=20.134401321411133，Pitch=1.4149998426437378，
Yaw=3.1349985599517822，Focal Point=(−4.7901082038879395,
−0.6352322697639465, 0.8174818158149719)。

## 基线算法和适配范围

八机配置 `config/rectangle_8uav_pcr.yaml` 从原版 `20.yaml` 继承图编队代价、
权重及 graph/ICP/sparse 开关：`use_graph=true`、`use_icp=false`、
`use_sparse=false`、`weight_graph=300`。编队规模和相对槽位改为上述八机矩形。
这些权重对应 PCR 原来的目标函数，不等同于参考项目的 `formation_weight=200`。

没有引入参考项目的 StarConvex 判据、断边/重连、换槽、中央路径指导或专用碰撞保护。
本入口的成功率、绕障表现、飞行时间和编队误差由 PCR 原算法实际运行产生。
不要将相同场景与相同外观理解为两种算法会产生相同轨迹。

运行适配包括：

- 修复源码压缩包中变成纯路径文本的顶层 `CMakeLists.txt`，可直接使用 catkin 编译。
- 初始成员数支持小于 20 的编队，八机入口明确设置为 8。
- 使用原有 `SWARM_PRESET_TARGET=4`，从配置计算各机最终航点，避开手动目标回调中
  写死的演示目标。里程计初始化提前，避免等待预设目标后覆盖收到的位置。
- 增加可选 `fsm/auto_start`；首次轨迹的起始时间变量显式初始化。
- 修复 `SwarmGraph` 的三个布尔函数在成功路径上缺少返回值的问题。
  原代码在 Release 优化下会在重规划时崩溃；只补齐返回值，不改变图代价和梯度公式。
- 拒绝非有限或退化的轨迹时长、系数、优化代价和梯度，利用原 L-BFGS 的
  `INVALID_FUNCVAL` 错误路径返回重规划，避免把 NaN 时长转成非法矩阵尺寸。
  优化结束后从求解器保留的变量重建轨迹，避免误用失败试探留下的 MINCO 缓冲区。
- 删除优化函数在碰撞检测之后的调试用“无条件成功”返回，恢复其后原本已有的
  碰撞结果处理。检测仍是原项目的局部轨迹采样检查，未增加参考项目的专用墙体保护。
- 模型、速度及传感器参数可从 launch 传入；旧入口保留原默认值。
- 八机平面场景使用原生的 `is_opt_formation_z=false` 和 `fixed_formation_z=1.5`，
  将编队局部目标高度固定在 1.5 m；高度可通过 `start_z` 一并修改。
  这是编队目标配置，不是额外的硬轨迹约束。原版三维编队默认值仍保留在其他入口中。
- 本机仿真直接通过 ROS 交换原有轨迹、局部目标和里程计消息，不启动 UDP 中继。
  这是仿真传输适配，不是引入参考项目的分布式编队决策。
- 点云测量带里程计时间戳；无障碍时发布有效空扫描；八机场景使用 360° 感知。
- 修复 ESDF 只在障碍点云包围盒内更新的问题，改为覆盖整个局部更新区域，
  让墙边绕行走廊也具有正确的障碍距离和梯度。保留 ESDF 临时数组的有效元素，
  避免 `clear()` 后继续按下标写入。此项修复解决了默认场景中 2、3、6、7 号机
  首条轨迹平滑后反复碰墙、一直留在起点的问题；没有改变物理墙体和无人机模型。
- 占用地图采用参考的 0.1 m 分辨率；A* 搜索步长独立设置为原版地图使用的
  0.25 m（`search/astar_resolution`），原搜索时间上限仍为 0.2 s。
  未设置该参数的旧入口继续使用自身地图分辨率。
  因此两项目的占用几何一致，前端搜索实现和步长属于各自算法配置。
- 原版写死的 20 s 候选轨迹时长上限改为可配置；八机低速场景使用
  `optimization/max_trajectory_duration=45` s，以容纳 0.5 m/s 下的局部绕障轨迹。
  旧入口默认仍为 20 s。该值是规划候选上限，不是实验总时长。

## 校验和采集

可重复检查 launch 展开后的实际参数，而不只检查 YAML 文本：

```zsh
python3 scripts/check_comparison_scene.py
```

脚本核对地图源码、模型文件、墙参数、八机起点/槽位/终点、运动上限、感知和栅格参数，
以及 RViz 相机和显示配置。参考仓库移动后可使用 `--reference /path/to/repository`。
脚本也要求无 RViz 时恰有 41 个节点（1 个地图节点、每机 5 个节点）。

图计算的回归检查覆盖目标队形零误差、平移不变性和局部/完整梯度更新一致性：

```zsh
catkin_make -j2 -l2 run_tests_frob_test_gtest_test_swarm_graph
catkin_test_results build/test_results/frob_test
```

检查默认自动启动路径（使用独立 master，默认端口 11359，结束时自动关闭测试节点）：

```zsh
source devel/setup.zsh
python3 scripts/test_comparison_startup.py
```

该检查要求八个规划器都发布至少一条时长为正、系数有限的轨迹，避免将
“八个节点已启动”误判为“八架无人机都已开始执行轨迹”。

建议两项目使用相同硬件、墙体参数、ROS 时间设置和记录话题，并记录 Git 版本及
`rosparam dump`。实验开始前启动录包，以包含第一条轨迹和完整里程计：

```zsh
mkdir -p results
rosbag record -O results/pcr_fixed_wall.bag --regex \
  '/drone_[0-7]_visual_slam/odom|/drone_[0-7]_planning/(trajectory|start|finish|data_display)|/map_generator/(global_cloud|wall_geometry)|/traj_start_trigger'
```

参考项目也记录相同话题，使用另一个输出文件名。原有源码中的个人目录日志路径不作为
本入口的数据采集方式；两边均以 rosbag 中的轨迹、里程计和时间戳计算对比指标。
所有到达率和碰撞指标应来自运行记录；单次仿真或离散采样检查不能替代多次实验统计。

## 本机验证记录

完整 catkin 编译、场景一致性脚本和两个图计算回归用例均通过。
ESDF 修复前，默认自动启动和延迟统一触发均可复现 2、3、6、7 号机没有首条执行轨迹；
逐机日志确认 A* 已成功，失败发生在平滑轨迹的碰撞检查阶段。
修复后，默认自动启动回归在 **4.78 s** 内收到八机的有效首条轨迹。

另一次默认自动启动与 RViz 完整实跑中，约 10 Hz 采样里程计，并每 2 s 检查
八个规划器的存活状态。启动后 50.99 s，八机位置均进入各自预定终点 0.3 m 范围，
没有规划器退出、断言或非有限位置。

此轮采样没有发现无人机中心进入墙体，但最小两机中心距为 **0.168 m**，
低于配置的 0.5 m；3 号机的采样位置还曾低于地图下界。机间间距及越界指标未满足，
因此此轮不记为已经验证的无碰撞通过。实际高度范围约为 −0.179–2.436 m；
`fixed_formation_z` 固定的是编队局部目标，并不限制整条轨迹的高度。
后续对比需同时报告实际间距、碰撞和高度变化。

本次修复后的本机产物保存在忽略 Git 的 `results/validation/esdf_fix/`：

- [运行数据](../results/validation/esdf_fix/runtime_summary.json)：终点误差、最小间距、高度范围及节点状态。
- [默认启动时 RViz 截图](../results/validation/esdf_fix/rviz_autostart.png)。
- `before_default_startup.json`、`before_manual_startup.json`、`after_default_startup.json`：修复前后的逐机轨迹计数。
- `startup_regression.json`、`roslaunch.log`、`manifest.json`：回归结果、原始日志及源码 SHA-256。

此前手动启动的单次验证仍保留在 `results/validation/`，对应修复前的源码版本，
不用于证明当前默认自动启动路径已经验证。

## 配准失败降级与地图边界（2026-11-20）

编队配准现在显式返回成功状态。输入必须有限且对应点数一致，RANSAC 使用原有
0.15 阈值、1000 次迭代上限和 0.99 概率，并要求至少 4 个内点。
失败结果不作为单位变换继续使用，也不写入参考缓存。配准输出最终应用
`is_opt_formation_z` / `fixed_formation_z` 的设置，避免 ICP 再次覆盖参考高度。

本轮配准数据不足、配准失败或优化扩展采样时配准失败，都会放弃本轮编队参考。
规划管理器复用本轮 A* 初始化和同一个全局局部目标，重试一次无编队参考的局部优化；
障碍、机间避碰、速度和加速度代价继续保留。下次重规划重新尝试配准。
降级也失败时不发布失败轨迹；旧轨迹临近失效时进入停止处理，停止位置本身也须合法。

地图外占据查询仍返回 `-1`。位置有限性、A* 查询与距离场查询增加边界处理；
距离场在地图外返回负的边界距离及朝向域内的梯度，不进行正常地图插值。
所有待发布轨迹按每段 x/y/z 的端点与内部极值做完整时间范围的边界验收，
包含原障碍检查窗口之外的尾段，并使用与 `isInMap` 一致的 1e-4 边界余量。
验收同时覆盖内部 double 轨迹和实际发布消息的 float32 系数、时长精度。
原有障碍碰撞检查时间窗口保持不变；边界验收并不等于连续时间障碍碰撞证明。

参考高度约束只作用于编队参考点，严格二维飞行仍需另行约束整条轨迹的 z 自由度。
前述 `esdf_fix` 数据对应此前源码，不能作为本次修复的运行结果。

回归入口：

```bash
catkin_make test_planning_validation -j1 -l1
source devel/setup.zsh
rostest traj_opt planning_validation.test
```

测试覆盖成功配准及固定参考高度、合法单位变换、失败缓存清理和无编队降级、
非有限坐标、地图外参考点、完整轨迹尾段及采样点之间的越界。

本次实际验证：完整工程与测试目标编译通过，12 项回归全部通过。
独立 ROS 主节点、关闭 RViz、当前默认固定墙及自动启动配置下，启动检查起约
46.98 s 后八机均满足距自身目标不超过 0.3 m、速度不超过 0.15 m/s 并保持 1 s；
此耗时包含启动和保持确认时间，不作为正式对照实验的飞行耗时。
共检查 264 条实际发布的 float32 轨迹，逐段复核坐标极值均在地图内；
约 100 Hz 的八路里程计也未发现越界或非有限位置，八个规划器全程存活。
记录到 245 条配准降级警告和 3 条越界候选拒绝警告；日志经过节流，条数不是精确事件总数。
采样高度范围为 0.181–2.691 m，仍存在三维运动；这次验证未评估机间/墙体碰撞率。
运行摘要位于 `/tmp/pcr-registration-boundary-runtime-24rrf7no/summary.json`，
回归日志位于 `/tmp/pcr-registration-boundary-before-moqhmrwr/rostest.log`。
