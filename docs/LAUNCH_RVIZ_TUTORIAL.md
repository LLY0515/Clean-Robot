# display.launch.py 与 urdf_config.rviz 教程

> 本文拆解 [display.launch.py](../src/sam_bot_description/launch/display.launch.py) 和 [urdf_config.rviz](../src/sam_bot_description/rviz/urdf_config.rviz) 两个文件，讲清它们各自的作用、每一段的含义，以及两者如何配合把机器人模型在 RViz 里显示出来。
>
> 前置知识：URDF / Xacro 建模见 [URDF_XACRO_TUTORIAL.md](URDF_XACRO_TUTORIAL.md)。

---

## 目录

1. [整体工作流：谁在干什么](#1-整体工作流谁在干什么)
2. [第一部分：display.launch.py 逐段解析](#2-第一部分displaylaunchpy-逐段解析)
3. [第二部分：urdf_config.rviz 逐段解析](#3-第二部分urdf_configrviz-逐段解析)
4. [第三部分：从零跑起来的完整步骤](#4-第三部分从零跑起来的完整步骤)
5. [常见坑](#5-常见坑)

---

## 1. 整体工作流：谁在干什么

先建立全局认识。一次 `ros2 launch` 会串起下面这条链：

```
display.launch.py（启动配方）
│
├─ robot_state_publisher 节点
│    └─ 读 robot_description（xacro 展开后的 URDF）
│    └─ 结合关节角度 → 发布 tf 树（base_link → 各 link）
│
├─ joint_state_publisher(_gui) 节点
│    └─ 发布 /joint_states（关节角度）
│
└─ rviz2 节点
     └─ 按 urdf_config.rviz 的配置渲染
          ├─ RobotModel 显示 → 从 /robot_description 读模型
          ├─ TF 显示 → 显示各坐标系
          └─ Fixed Frame = base_link → 以底盘为参考系
```

一句话总结：

- **launch 文件**回答“启动哪些节点、每个节点传什么参数”；
- **rviz 配置文件**回答“RViz 打开后长什么样、显示哪些东西、以哪个坐标系为准”。

---

## 2. 第一部分：display.launch.py 逐段解析

完整文件见 [display.launch.py](../src/sam_bot_description/launch/display.launch.py)，下面按块拆解。

### 2.1 导入依赖（第 1-4 行）

```python
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
```

- `launch`：ROS 2 launch 系统的核心，提供 `LaunchDescription`、`DeclareLaunchArgument` 等。
- `Command`：**在启动时执行一条 shell 命令，把它的标准输出作为参数值**（后面用来展开 xacro）。
- `LaunchConfiguration`：引用某个 launch 参数的**惰性**占位符——它的值要到真正启动时才解析。
- `launch_ros`：ROS 相关扩展，提供 `Node`、`FindPackageShare`。

### 2.2 找到包的安装目录（第 7 行）

```python
pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
```

- `FindPackageShare(package='...')` 会定位到该包被 `colcon build` 安装后的 `share/<包名>` 目录。
- 为什么是“安装后”的目录而不是源码目录？因为 [CMakeLists.txt](../CMakeLists.txt) 里有这条安装规则：

  ```cmake
  install(
    DIRECTORY launch rviz description
    DESTINATION share/${PROJECT_NAME}
  )
  ```

  源码里 `launch/`、`rviz/`、`description/` 都嵌套在 `src/` 下面；这条规则把三个目录（各自保留目录名 `launch`、`rviz`、`description`）安装到 `install/sam_bot_description/share/sam_bot_description/` 下。安装后目录结构是：

  ```
  install/.../share/sam_bot_description/
  ├── description/sam_bot_description.urdf
  ├── launch/display.launch.py
  └── rviz/urdf_config.rviz
  ```

  `pkg_share` 指向的就是这个 `share/sam_bot_description/` 根目录。

### 2.3 两个默认文件路径（第 8-9 行）

```python
default_model_path = os.path.join(pkg_share, 'description/sam_bot_description.urdf')
default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
```

- 注意模型路径里带 `description/` 前缀，正好对应上面的安装结构。
- 这两个只是**默认值**，用户可以启动时用参数覆盖（见 2.5）。

### 2.4 四个节点（第 11-34 行）

#### ① robot_state_publisher（第 11-15 行）

```python
robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
)
```

这是**最关键的一段**，拆开看：

- `parameters=[{'robot_description': ...}]`：给节点传一个名为 `robot_description` 的参数。
- `Command(['xacro ', LaunchConfiguration('model')])`：启动时执行 shell 命令
  `xacro <model路径>`，`xacro` 会把 `.urdf`/`.xacro` **展开成纯 URDF 字符串**，这个字符串就是 `robot_description` 的值。

> 为什么 `'xacro '` 后面有个空格？因为 `Command` 把列表元素拼成一条命令字符串，`['xacro ', model]` 拼接后就是 `xacro /path/to/file.urdf`。

`robot_state_publisher` 启动后会读取 `robot_description` 参数、解析出 link/joint 树，然后订阅 `/joint_states`，实时计算并广播各 link 的 tf。

#### ②③ joint_state_publisher（第 16-27 行）

```python
joint_state_publisher_node = launch_ros.actions.Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
)
joint_state_publisher_gui_node = launch_ros.actions.Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
)
```

两个节点二选一（通过 `condition` 互斥）：

- 无 GUI 版：把 URDF 里定义的关节默认值发布到 `/joint_states`；
- GUI 版：弹出一个滑杆窗口，手动拖动关节角度。

`IfCondition` / `UnlessCondition` 根据 launch 参数 `gui` 的真假决定要不要启动这个节点。

#### ④ rviz2（第 28-34 行）

```python
rviz_node = launch_ros.actions.Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')],
)
```

- `arguments=['-d', <rviz配置路径>]`：用 `-d` 让 RViz 启动时加载指定配置文件。
- `output='screen'`：把 RViz 的日志打到终端（方便排查）。

### 2.5 声明 launch 参数（第 37-42 行）

```python
launch.actions.DeclareLaunchArgument(name='gui', default_value='True', description='...')
launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='...')
launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='...')
```

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `gui` | `True` | 用哪个 joint_state_publisher（GUI 滑杆版 or 无界面版） |
| `model` | 模型文件路径 | 覆盖要加载的 URDF/xacro |
| `rvizconfig` | rviz 配置路径 | 覆盖 RViz 启动配置 |

### 2.6 组装并返回（第 36-47 行）

```python
return launch.LaunchDescription([
    ...DeclareLaunchArgument...,
    joint_state_publisher_node,
    joint_state_publisher_gui_node,
    robot_state_publisher_node,
    rviz_node
])
```

`LaunchDescription` 是 launch 系统的“菜谱”：按列表顺序启动所有动作。launch 会自动处理节点间的启动顺序和生命周期。

### 2.7 launch 文件小结

它做的事情一句话：**展开 xacro → 喂给 robot_state_publisher → 起 joint 角度发布器 → 起 RViz**。

---

## 3. 第二部分：urdf_config.rviz 逐段解析

完整文件见 [urdf_config.rviz](../src/sam_bot_description/rviz/urdf_config.rviz)。这是 RViz 保存下来的配置文件，格式类似 YAML。它描述的是**界面和显示内容的“快照”**。

### 3.1 顶层结构

整个文件由几个顶层键组成：

| 键 | 作用 |
|----|------|
| `Panels` | 左侧/底部的停靠面板（如 Displays 面板） |
| `Visualization Manager` | 核心：渲染什么、以哪个坐标系为准 |
| `Tools` | 工具栏里的交互工具 |
| `Views` | 相机视角（如 Orbit 环绕视图） |

### 3.2 Panels（第 1-12 行）

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /RobotModel1/Links1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 557
```

- 定义了左侧那个“Displays”面板，并记录了它展开到哪些节点（RobotModel 的 Links、TF 等）、面板高度等 UI 状态。这些是纯界面记忆，改不改都不影响功能。

### 3.3 Visualization Manager（第 13-63 行）

这是核心。包含 `Displays`（显示项列表）、`Global Options`（全局选项）。

#### Global Options（第 41-44 行）

```yaml
Global Options:
  Background Color: 48; 48; 48      # 深灰背景
  Fixed Frame: base_link            # 渲染参考系
  Frame Rate: 30                    # 刷新率
```

- **`Fixed Frame: base_link` 是最重要的一项**。RViz 把所有 tf 都换算到 `base_link` 坐标系下来渲染。因为 robot_state_publisher 发布的 tf 树根就是 `base_link`，所以它能正常显示。
- 如果这里填了一个不存在的坐标系（比如没配置好的 `map`），RViz 会报 `No transform from [base_link] to [xxx]` 错误，模型就显示不出来。

#### Displays 里的三个显示项（第 15-39 行）

##### ① Grid（网格，第 16-21 行）

```yaml
- Alpha: 0.5
  Cell Size: 1
  Class: rviz_default_plugins/Grid
  Color: 160; 160; 164
  Enabled: true
  Name: Grid
```

- 参考网格，帮助观察尺寸（格子 1m）。`Alpha` 透明度、`Color` 用 RGB `;` 分隔。

##### ② RobotModel（机器人模型，第 22-32 行）

```yaml
- Alpha: 0.6
  Class: rviz_default_plugins/RobotModel
  Description Topic:
    Depth: 5
    Durability Policy: Volatile
    History Policy: Keep Last
    Reliability Policy: Reliable
    Value: /robot_description
  Enabled: true
  Name: RobotModel
  Visual Enabled: true
```

- `Class: rviz_default_plugins/RobotModel`：这是 RViz 显示 URDF 模型的插件。
- `Description Topic` 是一个“话题属性”，告诉插件**从哪个话题读取机器人模型**：
  - `Value: /robot_description`：话题名。
  - `Depth: 5`：队列深度（缓存消息数）。
  - `Durability Policy: Volatile` / `History Policy: Keep Last` / `Reliability Policy: Reliable`：这三个是 QoS（服务质量）配置——Volatile（易失，不 latch）、Keep Last（只保留最新 N 条）、Reliable（可靠传输）。

  > ⚠️ 注意点：`robot_state_publisher` 实际上用 **Transient Local**（latch 模式，新订阅者能拿到最后一条）发布 `/robot_description`，而这里配置的是 **Volatile**。这意味着 RViz 若在消息发布**之后**才订阅，可能错过这条只发一次的消息，导致模型不显示。遇到“RViz 里空无一物但终端无报错”时，先查这里。

- `Visual Enabled: true`：是否显示模型的视觉部分（visual 标签）。

##### ③ TF（坐标系，第 33-39 行）

```yaml
- Class: rviz_default_plugins/TF
  Enabled: true
  Name: TF
  Marker Scale: 0.3
  Show Arrows: true
  Show Axes: true
  Show Names: true
```

- 把 tf 树里的每个坐标系画出来：显示坐标轴（Axes）、轴名称（Names）、箭头（Arrows）。调试机器人的关节和连杆关系非常有用。

#### Tools（第 46-53 行）与 Views（第 58-63 行）

- `Tools`：工具栏工具（移动相机、选中、聚焦、测量等）。
- `Views`：当前使用 Orbit（环绕）相机视角，`Target Frame: <Fixed Frame>` 表示相机围绕 Fixed Frame（即 `base_link`）旋转。

### 3.4 rviz 文件小结

- 它是 RViz 的**持久化配置**，决定“显示哪些东西、以哪个坐标系为参考”。
- 三个关键点：**Fixed Frame = base_link**、**RobotModel 的 Description Topic = /robot_description**、**TF 显示**。
- 可以**不用手写**：在 RViz 里手动添加 Displays、调好参数后 `File → Save Config As` 存成 `.rviz` 文件即可，跟这个文件效果一样。

---

## 4. 第三部分：从零跑起来的完整步骤

### 4.1 构建

```bash
# 进入包目录（包含 CMakeLists.txt / package.xml 的那一层）
cd /home/rostwo/CleanRobot_ws

# 构建（这会执行 CMakeLists.txt 里的 install 规则，把 src/sam_bot_description/launch/rviz 装到 share/ 下）
colcon build

# 让当前终端找到安装后的包
source install/setup.bash
```

### 4.2 启动

```bash
# 默认：GUI 滑杆版 joint_state_publisher + RViz
ros2 launch sam_bot_description display.launch.py

# 不带滑杆（纯静态关节角度）
ros2 launch sam_bot_description display.launch.py gui:=False

# 覆盖模型文件（换成你自己的 xacro）
ros2 launch sam_bot_description display.launch.py model:=/path/to/my_robot.urdf.xacro
```

启动后应该看到：

- RViz 窗口打开，加载了 [urdf_config.rviz](../src/sam_bot_description/rviz/urdf_config.rviz) 的配置；
- 青色底盘 + 两个灰色轮子 + 前方球体；
- 左侧 Displays 面板有 Grid / RobotModel / TF 三项；
- 用滑杆（gui=True 时）拖动 `drivewhl_l_joint` / `drivewhl_r_joint`，轮子会跟着转。

### 4.3 命令行验证（不依赖 RViz）

```bash
# 1. 确认 xacro 能展开（或直接 xacro 你的文件）
xacro src/sam_bot_description/description/sam_bot_description.urdf

# 2. 确认 robot_description 参数已加载
ros2 param get /robot_state_publisher robot_description

# 3. 确认 tf 树正确（base_link 是根）
ros2 run tf2_ros tf2_echo base_link base_footprint

# 4. 确认关节角度在发
ros2 topic echo /joint_states
```

---

## 5. 常见坑

| 现象 | 原因 | 解决 |
|------|------|------|
| RViz 空无一物、终端无报错 | RobotModel 的 Description Topic 用 Volatile，错过了 robot_state_publisher 那条 latch 消息 | 重启 robot_state_publisher，或把 Durability Policy 改成 Transient Local |
| `No transform from [xxx] to [base_link]` | Fixed Frame 填了一个 tf 树里不存在的坐标系 | 把 Fixed Frame 改成 `base_link` |
| 模型全白/无颜色 | URDF 里 material 没定义或没生效 | 检查 URDF 的 `<material>` / `<color>` |
| `xacro: command not found` | 没装 xacro，或没 source 环境 | `sudo apt install ros-<distro>-xacro` |
| launch 报找不到包 | 没 `colcon build` 或没 `source install/setup.bash` | 重新 build + source |
| 改了 URDF 但 RViz 不变 | 改的是源码目录，但 launch 读的是安装后的 `share/` 副本 | 重新 `colcon build` |
| build 报 `file INSTALL cannot find ...` | install 的 `DIRECTORY` 指向了不存在的目录（包根下现在用 `launch`，误写成 `src/launch` 会找不到） | 让 `DIRECTORY` 指向包根下的真实相对路径 |
| 改了 install 规则但旧文件还在 | CMake 安装不会清理 `install/` 里的旧文件，会残留过期副本 | `rm -rf build install log` 后重新 `colcon build` |
| 轮子不转 | continuous 关节，滑杆版能转；但 joint 的 axis 设错方向 | 检查 `<axis>` 是否绕期望的轴 |

> 最后一条延伸：`display.launch.py` 只做**可视化**，轮子在这里是“手动滑杆”转动的。真正让轮子按速度转起来，要靠 Gazebo 的 `diff_drive` 插件 + nav2 下发 `/cmd_vel`，那是 nav2 仿真的下一步。
