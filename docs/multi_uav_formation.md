# PCR 可复用多机编队

参考 `~/Codes/EGO-Planner-criterion-validation` 当前的
`distributed_fixed_wall.launch`、`distributed_sparse_random.launch` 和
`distributed_cluttered_random.launch`，增加运行时矩形队形配置。
PCR 的规划、配准、编队代价及消息类型继续使用当前工程实现。
本次不迁入参考工程的拓扑断边、重连和换槽算法。

## 编译和环境

```zsh
cd ~/Codes/pcr_orign/flip_privite-main/flip_privite-main
source /opt/ros/noetic/setup.zsh
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
rospack find ego_planner
```

包路径应为本工作区的 `src/planner/plan_manage`。bash 用户改用 `.bash`。
两工程的 ROS 包同名，切换时使用新终端并只 source 要运行的工作区；
每次只运行一个场景。以下每一行都是一个独立实验的启动命令。

## 固定墙

```zsh
# 六机 2×3
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=2 formation_columns:=3
# 八机 2×4
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=2 formation_columns:=4
# 十二机 3×4
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=3 formation_columns:=4
# 十六机 4×4
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=4 formation_columns:=4
```

## 随机障碍物

散布随机方柱场对应参考的 `distributed_cluttered_random.launch`：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=2 formation_columns:=3 random_seed:=17
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=2 formation_columns:=4 random_seed:=17
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=3 formation_columns:=4 random_seed:=17
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=4 formation_columns:=4 random_seed:=17
```

包含大障碍、堆叠块和窄通道的随机场对应参考的 `distributed_sparse_random.launch`：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=2 formation_columns:=3 random_seed:=17
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=2 formation_columns:=4 random_seed:=17
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=3 formation_columns:=4 random_seed:=17
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=4 formation_columns:=4 random_seed:=17
```

| 场景 | 地图 / m | 中心起终点 x / m | 默认障碍配置 |
| --- | --- | --- | --- |
| fixed_wall | 24×14×3 | −6 → 10 | 第一墙宽 6 m；第二墙总宽 10 m、开口 1.8 m |
| cluttered_random | 40×18×3 | −17 → 17 | 60 根随机方柱，边长 0.45–1.1 m，端点净空 5 m |
| sparse_random | 32×14×3 | −14 → 14 | 4 个大障碍、6 个小障碍、3 个堆叠块及 2 段窄通道墙 |

地图源码及默认地图参数与参考的通用入口一致。更换机数不会改变地图生成参数。
固定墙新入口使用 6 m 第一墙；旧八机入口仍保留原先的 10 m 参数。
修改随机种子可改变布局，`small_position_seed` 非负时独立重采样小障碍位置。
地图生成器会在无法放入指定数量的障碍时明确报错。

## 队形与常用参数

行沿 x、列沿 y，按行优先编号，与参考工程一致。第 `row, column` 个槽位为：

```text
id = row * formation_columns + column
dx = ((formation_rows - 1) / 2 - row) * row_spacing
dy = (column - (formation_columns - 1) / 2) * column_spacing
```

初始位置为 `(start_x + dx, dy, altitude)`，预设终点为
`(goal_x + dx, dy, altitude)`。默认间距 1 m、高度 1.5 m，速度/加速度参数为
0.5 m/s、6 m/s²。入口支持至少两行两列、最多十六机；启动前检查有限数值、
整数机数和起终点槽位是否在地图内。

```zsh
# 调整间距、速度、高度并关闭 RViz。
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall \
  formation_rows:=3 formation_columns:=4 row_spacing:=0.8 column_spacing:=1.2 \
  altitude:=1.7 max_vel:=0.6 rviz:=false

# 调整随机方柱场密度。
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random \
  formation_rows:=4 formation_columns:=4 small_obstacle_count:=40 \
  small_size_min:=0.45 small_size_max:=0.9 obstacle_separation:=0.6 random_seed:=23

# 等待手动统一触发。
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall auto_start:=false
```

最后一条命令默认八机。在另一已 source 同一工作区的终端，等所有飞机出现后执行：

```zsh
rostopic pub -1 /traj_start_trigger geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}'
```

地图尺寸、`start_x`、`goal_x`、`safety_distance`、`odom_topic`、墙几何及随机障碍参数
均可通过同名 launch 参数覆盖。未指定的参数从场景 YAML 继承。
`astar_time_limit` 控制每次 A* 搜索的时间上限，通用入口默认为 1.0 秒，
旧入口未设置时仍为 0.2 秒。它属于 PCR 的搜索预算，独立于地图和编队几何；
对照实验应记录这一参数。十六机与 RViz 并行运行时，旧 0.2 秒上限可能使部分
成员反复在首条路径搜索阶段超时，表现为停在起点。
实际使用的参数可在运行中查询或保存：

```zsh
rosparam get /pcr_formation_launcher/resolved
rosparam dump /tmp/pcr_formation_params.yaml
```

## 文件复用

- `launch/pcr_formation.launch`：所有机数和场景共用的入口，只传入用户覆盖值。
- `config/pcr_formation_scenes.yaml`：唯一一份通用和分场景默认参数。
- `scripts/launch_pcr_formation.py`：内存中生成 N 架飞机的启动配置和槽位；
  临时生成 N 机 RViz 配置，结束时清理，无需按机数复制文件。
- `comparison_8uav_member.launch` → `run_in_sim.launch` →
  `advanced_param.xml` / `simulator.xml`：复用既有逐机规划和仿真配置。
  原文件名保留，新增可选参数不改变旧入口默认行为。
- `config/rectangle_8uav_pcr.yaml`：复用当前 PCR 优化参数模板，运行时覆盖规模、
  槽位和高度。其当前开关为 `use_graph=false`、`use_icp=true`、`use_sparse=false`；
  新入口沿用当前值。小规模队形下模板中多余槽位不会被规划器读取。
- `sparse_random_obstacle_map.cpp`：从参考工作区原样复用，一份程序支持两种随机场景。

各机仍使用 0.1 m 占用地图、5 m / 360° 感知以及 PCR 的 0.25 m A* 搜索步长。
固定高度参数约束编队参考点，实际轨迹仍为三维优化。

## 参考工程启动命令

在新终端切换到参考工作区，先确保该工程已编译：

```zsh
cd ~/Codes/EGO-Planner-criterion-validation/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
roslaunch ego_planner distributed_fixed_wall.launch formation_rows:=3 formation_columns:=4
# 结束上一实验后选择随机场景：
roslaunch ego_planner distributed_cluttered_random.launch formation_rows:=3 formation_columns:=4 random_seed:=17
roslaunch ego_planner distributed_sparse_random.launch formation_rows:=3 formation_columns:=4 random_seed:=17
```

参考工程也用 `2/3`、`2/4`、`3/4`、`4/4` 分别选择六、八、十二、十六机。
地图与初始队形一致不代表两种规划器会产生相同轨迹。

## 检查

```zsh
# 无需 ROS master：展开三种场景×四种规模，核对真实规划参数及参考源码/地图/槽位。
python3 scripts/check_pcr_formation.py
# 参考仓库已移动时使用 --reference /path/to/repository；独立校验使用 --skip-reference。

# 独立 master 上验证所有成员均发布有限系数、正时长的首条轨迹，结束自动清理。
python3 scripts/test_comparison_startup.py --scene fixed_wall --rows 3 --columns 4 --timeout 90
python3 scripts/test_comparison_startup.py --scene cluttered_random --rows 4 --columns 4 --timeout 90
# 验证等待统一触发的路径。
python3 scripts/test_comparison_startup.py --scene fixed_wall --rows 2 --columns 3 --manual-start --timeout 90
# 同时验证实际离开初始位置至少 0.5 m；--rviz 可包含显示程序的负载。
python3 scripts/test_comparison_startup.py --scene fixed_wall --rows 4 --columns 4 --min-displacement 0.5 --timeout 90
# 不传 --scene 时，仍检查旧八机入口。
```

通用入口在 ROS 节点中展开子启动配置，`roslaunch --nodes` 只显示外层启动器；
使用上面的配置校验脚本检查全部子节点。无 RViz 时子配置包含 `1 + 5*N` 个节点，
外层另有一个启动器。

首条轨迹检查用于验证编队配置和启动链路；全航程到达率、最小机间距离、
墙体碰撞及越界需另行通过完整仿真和记录评估。

## 本机验证记录（2026-11-24）

完整 catkin 编译、十二种配置展开检查、参考资源/地图参数/槽位一致性检查均通过。
十二种场景和规模的自动启动均收到全部成员的有效首条轨迹：

| 场景 | 六机 / s | 八机 / s | 十二机 / s | 十六机 / s |
| --- | --- | --- | --- | --- |
| fixed_wall | 2.50 | 3.26 | 5.13 | 46.95 |
| sparse_random | 2.41 | 3.00 | 4.39 | 7.37 |
| cluttered_random | 2.37 | 2.81 | 5.17 | 6.81 |

上表对应本轮 A* 时限调整前的 0.2 秒版本。
数值包含启动开销，来自每组单次检查，不作为飞行时间或稳定性统计。
十六机固定墙在 A* 搜索/首条轨迹失败后重试，约 47 秒才收齐首条轨迹；
因此规模较大时检查超时建议设为 90 秒。
旧八机入口启动检查为 2.75 秒；六机手动触发检查为 3.27 秒，确认触发前无执行轨迹、
触发后全部收到有效轨迹。运行摘要、源码 SHA-256 及日志保存在忽略 Git 的
[results/validation/multi_uav_formation/summary.json](../results/validation/multi_uav_formation/summary.json)。
本轮验证范围为配置、启动和首条轨迹，未完成这些新规模的全航程避碰验收。

## 十六机固定墙部分成员留在起点的定位

用户截图对应的 `dc26bfe4-e818-11f1-b7b5-918530bafcca` 运行日志中，9 号机出现
62 次首条轨迹失败，10 号机出现 59 次；两者每次都对应 A* 的 0.2 秒超时。
状态机在 `SEQUENTIAL_START` 重试失败的搜索，因此这两架持续停在起点。
搜索时间按实际经过的时间计量，多机与显示并发负载会影响是否超过该阈值。
本次将写死的上限改为 `search/astar_time_limit`，通用入口通过
`astar_time_limit:=1.0` 设置，旧入口继续使用 0.2 秒默认值。
诊断计数和逐条失败摘录保存在
[astar_startup_fix/observed_diagnosis.json](../results/validation/astar_startup_fix/observed_diagnosis.json)。
补充的 `--min-displacement` 检查同时要求每架飞机发布有效轨迹且实际移动到阈值，
避免只收到悬停轨迹就结束检查。

修复后完整编译和十二种配置检查通过；下面的单次实跑均要求每个成员实际移动至少
0.5 m，并收到有效执行轨迹，检查结束时自动清理：

| 配置 | 检查通过耗时 / s |
| --- | --- |
| 十六机固定墙，开启 RViz | 9.34 |
| 十六机固定墙，关闭 RViz | 11.16 |
| 十六机随机方柱 | 7.14 |
| 十六机随机大障碍及窄通道 | 9.50 |
| 旧八机固定墙，保留 0.2 s 搜索时限 | 6.22 |
| 六机固定墙，手动触发 | 4.47 |

带 RViz 的固定墙运行中，9、10 号机也实际离开了起点；本次该运行未记录到 A*
超时或首条轨迹失败。这些单次耗时用于记录检查，不构成性能提升统计。
本次修改前的一次无 RViz 复跑也在 9.96 秒通过位移检查，说明旧问题随并行负载变化；
调整搜索预算针对的是失败日志中的超时退出。
原始日志、每机位移和源码校验值见
[astar_startup_fix/movement_summary.json](../results/validation/astar_startup_fix/movement_summary.json)。
