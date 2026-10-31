# URDF / Xacro 机器人建模教程

> 以本项目 `sam_bot`（两轮差速 + 前万向轮）为贯穿例子，从零讲清 URDF 语法、Xacro 参数化写法，以及如何在 ROS 2 里加载、在 RViz / Gazebo 里可视化。

---

## 目录

1. [概念与工具链](#1-概念与工具链)
2. [URDF 基础结构](#2-urdf-基础结构)
3. [几何形状与材质](#3-几何形状与材质)
4. [关节类型详解](#4-关节类型详解)
5. [Xacro：让 URDF 可维护](#5-xacro让-urdf-可维护)
6. [完整示例：规范化的 sam_bot](#6-完整示例规范化的-sam_bot)
7. [加载与可视化](#7-加载与可视化)
8. [Gazebo 仿真的补充](#8-gazebo-仿真的补充)
9. [常见坑](#9-常见坑)

---

## 1. 概念与工具链

### URDF 是什么

**URDF（Unified Robot Description Format，统一机器人描述格式）** 是一种 XML 格式，用来描述机器人的：

- **运动学结构**：有哪些连杆（link）、连杆之间怎么连接（joint）、坐标系如何定义；
- **几何外观**：每个连杆长什么样（盒子、圆柱、球、网格模型）；
- **物理属性**：质量、质心、转动惯量（仿真需要）；
- **关节约束**：转动/平移的极限、速度/力矩限制。

它只描述“机器人长什么样、结构如何”，本身**不负责**让它动起来——运动由控制系统（如 nav2、关节控制器）驱动。

### Xacro 是什么，为什么需要它

URDF 是纯 XML，写多了会有两个痛点：

1. **重复代码**：左右两个对称的轮子，几乎一模一样的代码要写两遍；
2. **魔法数字**：轮子半径 0.10 散落在文件各处，想改成 0.12 要全局搜索替换。

**Xacro** 是 URDF 的“宏预处理语言”，在 XML 之上加了变量、宏、数学表达式、文件包含。ROS 加载时会先用 `xacro` 命令把它展开成纯 URDF，再交给下游使用。

| 特性 | 语法 | 解决什么 |
|------|------|----------|
| 变量 | `<xacro:property name="r" value="0.10"/>` | 消除魔法数字 |
| 引用变量 | `${r}` | 一处定义处处生效 |
| 数学表达式 | `${(base_width/2 + gap)}` | 自动计算相对位置 |
| 宏 | `<xacro:macro name="wheel" params="prefix">...</xacro:macro>` | 消除重复代码 |
| 文件包含 | `<xacro:include filename="macros.xacro"/>` | 拆分大文件 |

### 工具链（ROS 2）

```
sam_bot.urdf.xacro ──(xacro 展开)──▶ robot_description 参数
                                        │
                                        ▼
                           robot_state_publisher ──▶ tf 树（各 link 相对位置）
                                        ▲
                                        │ 关节角度
                                        │
                           joint_state_publisher ──▶ /joint_states 话题
                                        │
                                        ▼
                                 RViz / Gazebo 显示
```

- **`robot_state_publisher`**：读取 `robot_description` 参数，结合关节角度发布各 link 的 tf 变换。
- **`joint_state_publisher`**：发布关节角度（无 GUI 版发固定值，GUI 版用滑杆手动调）。
- **RViz**：可视化外观。
- **Gazebo**：物理仿真（需要 collision + inertial）。

---

## 2. URDF 基础结构

### 根元素 `<robot>`

```xml
<?xml version="1.0"?>
<robot name="sam_bot" xmlns:xacro="http://ros.org/wiki/xacro">
  ...
</robot>
```

- `name`：机器人名字。
- `xmlns:xacro`：声明 xacro 命名空间，**使用 xacro 语法时必须加**，否则 `${...}` 不会被展开。

### `<link>`：连杆

link 是机器人的“刚性体”。每个 link 有一个坐标系（link frame），它的子元素都描述“相对这个坐标系”的位置。

```xml
<link name="base_link">
  <visual>      <!-- 外观（RViz 显示） -->
  <collision>   <!-- 碰撞体（Gazebo 物理碰撞） -->
  <inertial>    <!-- 惯量（Gazebo 动力学） -->
</link>
```

三个子元素都遵循相同的内部结构：`<origin>` + `<geometry>`（+ 各自特有的标签）。

### `<joint>`：关节

joint 定义两个 link 之间的相对运动关系。**关键规则：机器人是一棵树**，只能有一个根 link（root），每个 link 只能有一个父 link。

```xml
<joint name="base_joint" type="fixed">
  <parent link="base_link"/>
  <child  link="base_footprint"/>
  <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  <!-- 转动/平动关节还需 <axis> 和 <limit> -->
</joint>
```

- `parent` / `child`：树的方向是 **父 → 子**。
- `origin`：child 坐标系在 parent 坐标系中的位姿（`xyz` 位移 + `rpy` 旋转，单位 m 和弧度）。

### `<origin>` 的含义要分清

`<origin>` 出现在两个地方，含义不同：

| 出现位置 | 相对谁 |
|----------|--------|
| joint 里的 `<origin>` | child link 相对 parent link 的位姿 |
| visual/collision/inertial 里的 `<origin>` | 相对**该 link 自身**坐标系的偏移 |

---

## 3. 几何形状与材质

### 基本几何体

```xml
<box size="0.42 0.31 0.18"/>          <!-- 长 宽 高 -->
<cylinder radius="0.10" length="0.04"/> <!-- 半径 高（默认轴沿 z） -->
<sphere radius="0.05"/>
<mesh filename="package://sam_bot_description/meshes/base.stl"/> <!-- 外部网格 -->
```

> 注意 `<cylinder>` 默认轴沿 **z** 方向。轮子要绕 y 轴转，所以常见写法是把圆柱绕 x 转 90°：`<origin rpy="${pi/2} 0 0"/>`。

### 材质

```xml
<material name="Cyan">
  <color rgba="0 1.0 1.0 1.0"/>   <!-- R G B A，0~1 -->
</material>
```

`material name` 若与已定义的材质同名会复用；也可以给 `visual` 的 `material` 直接写 `<color>`。

---

## 4. 关节类型详解

`type` 决定 link 之间的运动自由度：

| type | 含义 | 需要 `<axis>` | 需要 `<limit>` | 用途 |
|------|------|:---:|:---:|------|
| `fixed` | 完全固定，无运动 | 否 | 否 | 传感器安装、footprint |
| `revolute` | 绕轴转动，**有角度范围** | 是 | 是 | 舵机/手臂关节 |
| `continuous` | 绕轴无限转动 | 是 | 否 | 轮子、转盘 |
| `prismatic` | 沿轴直线滑动 | 是 | 是 | 伸缩机构 |
| `floating` | 6 自由度自由运动 | 否 | 否 | 少用 |
| `planar` | 平面 2D 运动 | 否 | 否 | 少用 |

### 转动关节完整写法

```xml
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child  link="arm_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>                       <!-- 转轴方向 -->
  <limit lower="-1.57" upper="1.57"         <!-- 弧度 -->
         effort="30" velocity="3.14"/>      <!-- 力矩 N·m / 角速度 rad/s -->
</joint>
```

`continuous` 就是没有 `lower/upper` 的 `revolute`，适合轮子。

---

## 5. Xacro：让 URDF 可维护

### 5.1 变量 `property`

```xml
<xacro:property name="wheel_radius" value="0.10"/>
```

之后用 `${wheel_radius}` 引用。表达式里可以直接做数学：

```xml
<origin xyz="${base_width/2 + wheel_ygap} 0 ${-wheel_zoff}"/>
```

xacro 支持 `+ - * /` 和常用函数（如 `${pi}`）。

### 5.2 宏 `macro`

```xml
<xacro:macro name="wheel" params="prefix x_reflect y_reflect">
  <link name="${prefix}_link">
    ...
  </link>
  <joint name="${prefix}_joint" type="continuous">
    ...
    <origin xyz="${x_reflect*wheel_xoff} ${y_reflect*(base_width/2+wheel_ygap)} ..."/>
  </joint>
</xacro:macro>
```

调用：

```xml
<xacro:wheel prefix="drivewhl_l" x_reflect="-1" y_reflect="1"/>
<xacro:wheel prefix="drivewhl_r" x_reflect="-1" y_reflect="-1"/>
```

一次定义，生成左右两个轮子——这正是原 `sam_bot` 里 `wheel` 宏的用法。

### 5.3 文件包含 `include`

大文件拆开，主文件里引入：

```xml
<xacro:include filename="$(find sam_bot_description)/description/macros.xacro"/>
```

（宏必须在调用之前 include。）

---

## 6. 完整示例：规范化的 sam_bot

> 相比原来的 `sam_bot_description.urdf`，这一版做了三件事：
> 1. 拆成 `macros.xacro` + 主文件 `sam_bot.urdf.xacro`；
> 2. 给每个 link 补上 `collision` 和 `inertial`（Gazebo 仿真必需）；
> 3. 用标准的 `.urdf.xacro` 扩展名。

### 6.1 `macros.xacro`：可复用宏

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- 驱动轮：一个 link（连续转动的圆柱）+ 一个 continuous 关节
       x_reflect / y_reflect 用于左右镜像定位 -->
  <xacro:macro name="drive_wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_link">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="gray"/>
      </visual>

      <!-- 碰撞体通常与外观一致或更简化 -->
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>

      <!-- 惯量：实心圆柱（绕 z 轴的公式），质量 wheel_mass -->
      <inertial>
        <mass value="${wheel_mass}"/>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <inertia
          ixx="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12.0}"
          ixy="0.0" ixz="0.0"
          iyy="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12.0}"
          iyz="0.0"
          izz="${wheel_mass * wheel_radius*wheel_radius / 2.0}"/>
      </inertial>
    </link>

    <joint name="${prefix}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_link"/>
      <origin xyz="${x_reflect*wheel_xoff} ${y_reflect*(base_width/2 + wheel_ygap)} ${-wheel_zoff}"
              rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

</robot>
```

### 6.2 `sam_bot.urdf.xacro`：主文件

```xml
<?xml version="1.0"?>
<robot name="sam_bot" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:include filename="$(find sam_bot_description)/description/macros.xacro"/>

  <!-- ===== 常量定义 ===== -->
  <xacro:property name="base_width"  value="0.31"/>
  <xacro:property name="base_length" value="0.42"/>
  <xacro:property name="base_height" value="0.18"/>
  <xacro:property name="base_mass"   value="5.0"/>

  <xacro:property name="wheel_radius" value="0.10"/>
  <xacro:property name="wheel_width"  value="0.04"/>
  <xacro:property name="wheel_ygap"   value="0.025"/>
  <xacro:property name="wheel_zoff"   value="0.05"/>
  <xacro:property name="wheel_xoff"   value="0.12"/>
  <xacro:property name="wheel_mass"   value="0.2"/>

  <xacro:property name="caster_xoff" value="0.14"/>

  <!-- 材质 -->
  <material name="cyan"><color rgba="0 1.0 1.0 1.0"/></material>
  <material name="gray"><color rgba="0.5 0.5 0.5 1.0"/></material>

  <!-- ===== 底盘 ===== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="cyan"/>
    </visual>

    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <!-- 实心长方体的惯量公式：Ixx=m/12*(w²+h²) 等 -->
    <inertial>
      <mass value="${base_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
        ixx="${base_mass * (base_width*base_width + base_height*base_height) / 12.0}"
        ixy="0.0" ixz="0.0"
        iyy="${base_mass * (base_length*base_length + base_height*base_height) / 12.0}"
        iyz="0.0"
        izz="${base_mass * (base_length*base_length + base_width*base_width) / 12.0}"/>
    </inertial>
  </link>

  <!-- ===== 足迹参考系（导航/定位用，贴地） ===== -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0 0 ${-(wheel_radius + wheel_zoff)}" rpy="0 0 0"/>
  </joint>

  <!-- ===== 左右驱动轮（宏实例化） ===== -->
  <xacro:drive_wheel prefix="drivewhl_l" x_reflect="-1" y_reflect="1"/>
  <xacro:drive_wheel prefix="drivewhl_r" x_reflect="-1" y_reflect="-1"/>

  <!-- ===== 前万向轮（简化为贴地球体） ===== -->
  <link name="front_caster">
    <visual>
      <geometry>
        <sphere radius="${wheel_radius + wheel_zoff - base_height/2}"/>
      </geometry>
      <material name="cyan"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="${wheel_radius + wheel_zoff - base_height/2}"/>
      </geometry>
    </collision>
    <!-- 万向轮仅作支撑，质量给个小值即可 -->
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="${caster_xoff} 0 ${-base_height/2}" rpy="0 0 0"/>
  </joint>

</robot>
```

### 惯量公式速查

| 形状 | 轴 | 公式 |
|------|-----|------|
| 实心长方体 (l,w,h) | 沿 l 轴 | `I = m/12·(w² + h²)` |
| 实心圆柱 (r,h)，轴沿 z | x/y 轴 | `I = m/12·(3r² + h²)` |
| 实心圆柱 (r,h) | z 轴 | `I = m/2·r²` |
| 实心球 (r) | 任意 | `I = 2m/5·r²` |

> **惯性矩阵对角元素必须为正**。写 0 或负值会导致 Gazebo 报错。拿不准时，用简化值（如 `0.00001`）也比 0 好。

---

## 7. 加载与可视化

### 7.1 展开 xacro 验证

写完先本地验证能否展开：

```bash
xacro src/description/sam_bot.urdf.xacro
```

能打印出完整 URDF 且无报错，再往下走。

### 7.2 launch 文件

本项目的 [display.launch.py](../src/launch/display.launch.py) 已经做了核心几件事：

```python
parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
```

`Command(['xacro ', model])` 会在启动时把 xacro **展开成纯 URDF 字符串**，赋给 `robot_description` 参数。若改用上面的 `.urdf.xacro` 文件，只需把 `default_model_path` 指向新文件名即可。

四个节点各司其职：

| 节点 | 作用 |
|------|------|
| `robot_state_publisher` | 发布 tf 树 |
| `joint_state_publisher(_gui)` | 发布关节角度（GUI 版带滑杆） |
| `rviz2` | 可视化 |

运行：

```bash
ros2 launch sam_bot_description display.launch.py
# 不想用 GUI 滑杆时：
ros2 launch sam_bot_description display.launch.py gui:=False
```

### 7.3 RViz 配置要点

- Fixed Frame 设为 `base_link`；
- Add → RobotModel，`Description Topic` 用 `/robot_description`；
- Add → TF 查看坐标系。

### 7.4 命令行检查

```bash
# 查看 tf 树
ros2 run tf2_ros tf2_echo base_link drivewhl_l_link

# 直接看展开后的 URDF
ros2 param get /robot_state_publisher robot_description
```

---

## 8. Gazebo 仿真的补充

RViz 只看外观，Gazebo 还要物理。要能进 Gazebo，除了上面补的 `collision` 和 `inertial`，还需要：

1. **每个 link 都有惯量**，且不能为 0（见第 6 节）。
2. **给驱动轮加传动/驱动插件**（让轮子能被控制转动），在 URDF 里加：

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>drivewhl_l_joint</left_joint>
    <right_joint>drivewhl_r_joint</right_joint>
    <wheel_separation>${base_width + 2*wheel_ygap}</wheel_separation>
    <wheel_diameter>${2*wheel_radius}</wheel_diameter>
    <command_topic>cmd_vel</command_topic>
  </plugin>
</gazebo>
```

3. **启动 gazebo 并 spawn 机器人**（`gazebo_ros` 的 `spawn_entity` 节点）。

> 这是 nav2 仿真的前提：nav2 通过 `/cmd_vel` 话题下发速度，diff_drive 插件驱动轮子转动，轮子带动车体前进。

---

## 9. 常见坑

1. **忘记 `xmlns:xacro`** → `${...}` 不展开，直接当字面量，报错或不显示。
2. **`<cylinder>` 轴默认沿 z** → 轮子方向不对，记得 `rpy="${pi/2} 0 0"`。
3. **`rpy` 单位是弧度** → 不是角度，转 90° 要写 `${pi/2}`。
4. **joint 的 `origin` 写错方向** → child 相对 parent 的位姿，容易和 visual 里的 origin 搞混。
5. **robot 不是树** → 每个 link 只能一个父；多父或成环会导致 tf 混乱。
6. **惯量为 0** → Gazebo 直接报错拒载。给个合理值。
7. **单位** → URDF 默认米、千克、秒、弧度。
8. **宏未 include 就调用** → `xacro:include` 要放在使用宏之前。
9. **`$(find pkg)` 找不到包** → 确保已 `colcon build` 且 `source install/setup.bash`，包名与 `package.xml` 一致。

---

## 参考资料

- [URDF XML 规范](http://wiki.ros.org/urdf/XML)
- [Xacro 教程](http://wiki.ros.org/xacro)
- [URDF Tutorials（官方教程）](http://wiki.ros.org/urdf/Tutorials)
