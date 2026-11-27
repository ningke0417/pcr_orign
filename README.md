# PCR 多机矩形编队

所有机数和场景共用 `pcr_formation.launch`。默认打开 RViz、自动开始飞行，
最大速度参数为 **0.5 m/s**。以下每条 `roslaunch` 命令对应一个独立实验，
结束上一场景后再启动下一场景。

## 编译与终端环境

在 zsh 终端执行：

```zsh
cd ~/Codes/pcr_orign/flip_privite-main/flip_privite-main
source /opt/ros/noetic/setup.zsh
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
```

后续新终端进入同一目录，重新 source ROS 和 `devel/setup.zsh` 即可。
bash 用户将 `.zsh` 改为 `.bash`。两个工程的 ROS 包同名，运行前可用
`rospack find ego_planner` 确认指向本工程的 `src/planner/plan_manage`。

## 编队规模

| 机数 | formation_rows | formation_columns |
| --- | --- | --- |
| 6 | 2 | 3 |
| 8（默认） | 2 | 4 |
| 12 | 3 | 4 |
| 16 | 4 | 4 |

行沿 x 轴、列沿 y 轴，行列间距默认均为 1 m。

## 固定墙场景

`scene:=fixed_wall`：24×14×3 m 地图，编队中心从 x=−6 m 飞向 x=10 m。
默认第一堵墙长 6 m，第二堵墙总长 10 m、中间开口 1.8 m。

```zsh
# 六机
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=2 formation_columns:=3
# 八机
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=2 formation_columns:=4
# 十二机
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=3 formation_columns:=4
# 十六机
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall formation_rows:=4 formation_columns:=4
```

### 修改第一堵墙的长度

第一堵墙沿 **y 轴**延伸，长度使用 **`wall_size_y`**，单位为米。
例如将第一堵墙从 6 m 改为 8 m，运行十六机：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall \
  formation_rows:=4 formation_columns:=4 wall_size_y:=8.0
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `wall_size_y` | 6.0 m | 第一堵墙沿 y 轴的长度 |
| `wall_size_x` | 0.5 m | 第一堵墙沿 x 轴的厚度 |
| `wall_x` | −2.0 m | 第一堵墙的中心 x 坐标 |
| `wall_y` | 0.0 m | 两堵墙共用的中心 y 坐标 |
| `second_wall_x` | 6.5 m | 第二堵墙的中心 x 坐标 |
| `second_wall_size_x` | 0.5 m | 第二堵墙的厚度 |
| `second_wall_total_size_y` | 10.0 m | 第二堵墙沿 y 轴的总跨度，含中间开口 |
| `second_wall_gap_size_y` | 1.8 m | 第二堵墙中间开口宽度，须小于总跨度 |

例如同时设置第一堵墙长 8 m、第二堵墙开口宽 2.2 m：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=fixed_wall \
  formation_rows:=4 formation_columns:=4 wall_size_y:=8.0 second_wall_gap_size_y:=2.2
```

墙长需要结合 `map_size_y`（默认 14 m）设置，为墙两侧的绕行保留空间。

## 随机方柱场景

`scene:=cluttered_random`：40×18×3 m 地图，编队中心从 x=−17 m 飞向 x=17 m。
默认生成 **60 个小型随机障碍物**，x/y 尺寸范围均为 0.45–1.1 m，
端点附近预留 5 m 净空；默认没有大障碍、堆叠块和窄通道墙。

```zsh
# 六机
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=2 formation_columns:=3 random_seed:=17
# 八机
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=2 formation_columns:=4 random_seed:=17
# 十二机
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=3 formation_columns:=4 random_seed:=17
# 十六机
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random formation_rows:=4 formation_columns:=4 random_seed:=17
```

### 修改随机障碍物的数量、大小和布局

此场景主要用 **`small_obstacle_count`** 修改障碍物数量。
例如运行十六机，将数量改为 40 个、尺寸改为 0.45–0.9 m，使用种子 23：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random \
  formation_rows:=4 formation_columns:=4 small_obstacle_count:=40 \
  small_size_min:=0.45 small_size_max:=0.9 obstacle_separation:=0.6 random_seed:=23
```

## 随机大障碍、堆叠块与窄通道场景

`scene:=sparse_random`：32×14×3 m 地图，编队中心从 x=−14 m 飞向 x=14 m。
默认有 **4 个随机大障碍 + 6 个随机小障碍 + 3 个堆叠块 + 2 段窄通道墙**，
共 15 个障碍长方体。

```zsh
# 六机
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=2 formation_columns:=3 random_seed:=17
# 八机
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=2 formation_columns:=4 random_seed:=17
# 十二机
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=3 formation_columns:=4 random_seed:=17
# 十六机
roslaunch ego_planner pcr_formation.launch scene:=sparse_random formation_rows:=4 formation_columns:=4 random_seed:=17
```

分别使用 `large_obstacle_count`、`small_obstacle_count` 修改随机大、小障碍的数量。
例如保留 4 个大障碍，将小障碍增加到 10 个，运行十二机：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=sparse_random \
  formation_rows:=3 formation_columns:=4 large_obstacle_count:=4 small_obstacle_count:=10 random_seed:=17
```

堆叠块和窄通道墙由 `structured_challenge_enabled` 控制。保持其默认值 `true` 时，
障碍长方体总数为 `large_obstacle_count + small_obstacle_count + 5`；
设为 `false` 时，只保留随机大、小障碍。例如：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=sparse_random \
  formation_rows:=3 formation_columns:=4 structured_challenge_enabled:=false \
  large_obstacle_count:=4 small_obstacle_count:=10 random_seed:=17
```

### 两类随机场景的常用参数

| 参数 | cluttered_random 默认值 | sparse_random 默认值 | 含义 |
| --- | --- | --- | --- |
| `small_obstacle_count` | 60 | 6 | 随机小障碍数量，非负整数 |
| `large_obstacle_count` | 0 | 4 | 随机大障碍数量，非负整数 |
| `small_size_min` / `small_size_max` | 0.45 / 1.1 m | 0.45 / 0.9 m | 小障碍 x/y 尺寸的下限和上限 |
| `random_seed` | 17 | 17 | 地图随机种子；相同参数和种子可复现相同地图 |
| `small_position_seed` | −1 | −1 | −1 沿用原采样序列；非负值独立重采样小障碍位置 |
| `obstacle_separation` | 0.6 m | 0.7 m | 随机障碍采样时的分离间隔参数 |
| `endpoint_clearance` | 5.0 m | 3.0 m | 起终点方向预留的障碍净空 |
| `boundary_clearance` | 0.8 m | 0.8 m | 障碍物与地图 y 边界的预留距离 |
| `small_center_y_limit` | 8.0 m | 2.0 m | 小障碍中心的 y 坐标绝对值上限 |
| `structured_challenge_enabled` | false | true | 是否加入 3 个堆叠块和 2 段窄通道墙 |
| `stacked_cluster_x` | −2.0 m | −2.0 m | 开启堆叠块时，其簇位置的 x 参数 |
| `narrow_gap_x` / `narrow_gap_size_y` | 4.8 / 1.6 m | 4.8 / 1.6 m | 开启窄通道时，墙的 x 坐标和开口宽度 |

增加障碍数量时，需要结合地图尺寸、障碍尺寸和间隔设置；生成器无法放入指定数量时
会报 `Unable to place obstacle`，可减少数量、尺寸或间隔，或扩大场景空间。

## 编队和运行参数

以下参数适用于三个场景，可直接追加到上述启动命令：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_vel` | 0.5 m/s | 每架无人机的最大速度参数 |
| `max_acc` | 6.0 m/s² | 最大加速度参数 |
| `row_spacing` / `column_spacing` | 1.0 / 1.0 m | 编队行、列间距 |
| `altitude` | 1.5 m | 初始、终点及编队参考高度；轨迹仍为三维优化 |
| `safety_distance` | 0.2 m | 地图障碍膨胀及优化障碍间距参数 |
| `astar_time_limit` | 1.0 s | 单次 A* 搜索时间上限 |
| `rviz` | true | 是否打开 RViz，设为 `false` 可关闭 |
| `auto_start` | true | 是否自动开始飞行 |

例如修改速度并关闭 RViz：

```zsh
roslaunch ego_planner pcr_formation.launch scene:=cluttered_random \
  formation_rows:=4 formation_columns:=4 max_vel:=0.8 rviz:=false random_seed:=17
```

若启动时设置 `auto_start:=false`，等全部飞机出现后，在已 source 同一工作区的另一终端触发：

```zsh
rostopic pub -1 /traj_start_trigger geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}'
```

## 持久修改默认参数

默认参数文件为
[pcr_formation_scenes.yaml](src/planner/plan_manage/config/pcr_formation_scenes.yaml)。
参数优先级为：**启动命令中的参数 > `scenes.<场景名>` > `common`**。

例如在现有文件的对应字段中修改第一堵墙长度和随机方柱数量，以下仅展示相关字段：

```yaml
common:
  wall_size_y: 8.0
scenes:
  cluttered_random:
    small_obstacle_count: 40
```

`cluttered_random` 的数量默认在 `scenes.cluttered_random` 中设为 60，
所以要修改该场景的默认数量，应编辑该场景下的 `small_obstacle_count`。
仅改 `common.small_obstacle_count` 会被场景自身的值覆盖。
修改 launch 参数或 YAML 后重新启动场景生效，无需重新编译。

运行中可查询本次实际生效的参数：

```zsh
rosparam get /pcr_formation_launcher/resolved
```

## 更多说明

完整参数、各机数的启动命令、参考工程命令及验证方法见
[多机编队说明](docs/multi_uav_formation.md)。
原八机入口 `rectangle_8uav_fixed_split_merge_wall.launch` 继续保留；
它的历史实验说明见 [comparison_fixed_wall.md](docs/comparison_fixed_wall.md)。
