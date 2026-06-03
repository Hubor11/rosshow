# rosshow Code Wiki

## 项目概述

**rosshow** 是一个在终端中可视化 ROS (Robot Operating System) 主题的工具，使用 Unicode Braille 艺术渲染各种传感器消息。该项目支持 ROS1 和 ROS2。

### 核心特性
- 支持多种 ROS 消息类型的可视化
- 使用 Unicode Braille 字符实现高分辨率渲染
- 支持纯 ASCII 模式渲染
- 支持 1-bit、4-bit、24-bit 颜色模式
- 交互式键盘控制（缩放、平移、旋转）

### 版本信息
- 版本: 2.0.1
- 许可证: BSD
- 维护者: dheera@dheera.net

---

## 项目架构

```
rosshow/
├── rosshow.py              # 主入口程序
├── termgraphics.py         # 终端图形渲染引擎
├── plotters.py             # 绘图工具（示波器、角度图）
├── getch.py                # 跨平台键盘输入
├── rospy2/                 # ROS2 兼容层
│   ├── __init__.py         # rospy2 核心实现
│   └── constants.py        # 日志级别常量
├── command/                # ROS2 CLI 扩展
│   └── show.py             # show 命令实现
└── viewers/                # 消息类型查看器
    ├── generic/            # 通用查看器
    │   ├── SinglePlotViewer.py    # 单数据绘图查看器
    │   ├── MultiPlotViewer.py     # 多数据绘图查看器
    │   ├── Space2DViewer.py       # 2D空间查看器（可缩放/平移）
    │   └── GenericImageViewer.py   # 通用图像查看器
    ├── sensor_msgs/         # 传感器消息查看器
    │   ├── LaserScanViewer.py
    │   ├── ImageViewer.py
    │   ├── CompressedImageViewer.py
    │   ├── PointCloud2Viewer.py
    │   ├── ImuViewer.py
    │   ├── NavSatFixViewer.py
    │   └── ros2_pointcloud2.py     # ROS2 PointCloud2 兼容
    └── nav_msgs/            # 导航消息查看器
        ├── OccupancyGridViewer.py
        ├── OdometryViewer.py
        └── PathViewer.py
```

---

## 依赖关系

### 系统依赖
- Python 3
- numpy
- pillow (PIL)
- requests

### ROS 依赖
**ROS1:**
- rospy
- std_msgs
- sensor_msgs
- nav_msgs
- geometry_msgs

**ROS2:**
- rclpy
- builtin_interfaces
- rcl_interfaces
- std_msgs
- sensor_msgs
- nav_msgs
- geometry_msgs

### 安装依赖
```bash
sudo pip install numpy pillow requests
```

---

## 主要模块详解

### 1. rosshow.py (主入口)

**职责:** 程序主入口，处理命令行参数、ROS 订阅和绘图循环。

**关键常量:**

| 常量 | 说明 |
|------|------|
| `VIEWER_MAPPING` | 消息类型到查看器的映射字典 |

**关键函数:**

#### `capture_key_loop(viewer)`
键盘捕获循环，在独立线程中运行。

**参数:**
- `viewer`: Viewer 实例，用于处理按键事件

**处理按键:**
- `Ctrl+C`: 退出程序
- 方向键: 调用 `viewer.keypress()`
- `+`/`-`: 缩放控制

#### `main()`
主函数。

**功能流程:**
1. 解析命令行参数 (`-a`, `-c1`, `-c4`, `-c24`, `--reliable`, `--transient-local`)
2. 初始化 ROS 节点
3. 获取话题类型
4. 创建 `TermGraphics` 画布
5. 根据消息类型动态加载对应的 Viewer 类
6. 订阅话题并启动键盘捕获线程
7. 进入 15fps 绘图循环

**命令行参数:**
```
-a, --ascii        # 使用纯 ASCII 模式
-c1               # 强制单色模式
-c4               # 强制 4-bit 颜色模式 (16 色)
-c24              # 强制 24-bit 颜色模式
--reliable        # ROS2 QoS 可靠性 (默认 best_effort)
--transient-local # ROS2 QoS 持久性 (默认 volatile)
```

---

### 2. termgraphics.py (终端图形引擎)

**职责:** 提供终端图形渲染能力，使用 Unicode Braille 字符实现像素级绘图。

**常量:**

| 常量 | 值 | 说明 |
|------|-----|------|
| `MODE_UNICODE` | 0 | Unicode 模式 |
| `MODE_EASCII` | 2 | 扩展 ASCII 模式 |
| `COLOR_SUPPORT_1` | 0 | 单色支持 |
| `COLOR_SUPPORT_16` | 1 | 16 色支持 |
| `COLOR_SUPPORT_256` | 2 | 256 色支持 |
| `COLOR_SUPPORT_24BIT` | 3 | 24-bit 真彩色 |
| `IMAGE_MONOCHROME` | 0 | 单色图像 |
| `IMAGE_UINT8` | 1 | 8-bit 灰度 |
| `IMAGE_RGB_2X4` | 2 | RGB 2x4 块 |
| `IMAGE_RGB` | 3 | RGB 图像 |

**颜色常量:**
- `COLOR_BLACK`, `COLOR_RED`, `COLOR_GREEN`, `COLOR_YELLOW`
- `COLOR_BLUE`, `COLOR_MAGENTA`, `COLOR_CYAN`, `COLOR_WHITE`

**类: `TermGraphics`**

#### `__init__(mode, color_support)`
初始化终端图形。

**参数:**
- `mode`: 渲染模式 (`MODE_UNICODE` 或 `MODE_EASCII`)
- `color_support`: 颜色支持级别（可选，自动检测）

#### `update_shape()`
获取并更新终端尺寸。返回 `True` 如果尺寸发生变化。

#### `clear()`
清空图形缓冲区。

#### `points(points, colors, clear_block)`
绘制多个点。

**参数:**
- `points`: 点列表，`[(x0,y0), (x1,y1), ...]`
- `colors`: 对应颜色数组（可选）
- `clear_block`: 是否清除块（可选）

#### `point(point, clear_block)`
绘制单个点。

#### `line(point0, point1)`
绘制直线。

#### `rect(point0, point1)`
绘制矩形。

#### `poly(points)`
绘制折线（连接所有点）。

#### `text(text, point)`
在指定位置绘制文本。

#### `image(data, width, height, point, image_type, clear_block)`
绘制二进制图像。

**支持的图像类型:**
- `IMAGE_MONOCHROME`: 单色图像
- `IMAGE_UINT8`: 8-bit 灰度
- `IMAGE_RGB`: RGB 图像
- `IMAGE_RGB_2X4`: 2x4 块 RGB

#### `draw()`
将图形缓冲区渲染到屏幕。执行增量渲染优化。

---

### 3. plotters.py (绘图工具)

**职责:** 提供高级绘图功能，包括示波器和角度指示器。

**类: `AnglePlotter`**

角度绘图器，用于绘制方向/角度指示器。

#### `__init__(g, left, right, top, bottom)`
初始化角度绘图器。

#### `update(angle)`
更新角度值。

#### `plot()`
绘制角度指示器（带动画效果）。

---

**类: `ScopePlotter`**

示波器绘图器，用于绘制时序数据曲线。

#### `__init__(g, left, right, top, bottom, ymin, ymax, n, title)`
初始化示波器。

**参数:**
- `n`: 数据点数量（默认 128）
- `ymin/ymax`: Y 轴范围（可选，自动缩放）
- `title`: 图表标题（可选）

#### `update(value)`
添加新的数据点。

#### `plot()`
绘制示波器曲线。

#### `get_nice_scale_bound(value)`
计算合适的刻度边界（自动选择 1, 2, 5 的幂次）。

---

### 4. getch.py (键盘输入)

**职责:** 跨平台键盘输入捕获。

**类: `Getch`**

#### `__init__()`
自动选择平台实现（Unix 或 Windows）。

#### `__call__()`
读取单个字符，不回显。

#### `reset()`
重置终端设置。

---

### 5. viewers/ (消息查看器)

#### 5.1 generic/SinglePlotViewer.py

**支持消息类型:**
- `std_msgs/Bool`, `std_msgs/Float32`, `std_msgs/Float64`
- `std_msgs/Int8/16/32/64`, `std_msgs/UInt8/16/32/64`
- `sensor_msgs/FluidPressure`, `sensor_msgs/RelativeHumidity`
- `sensor_msgs/Illuminance`, `sensor_msgs/Range`, `sensor_msgs/Temperature`

**类: `SinglePlotViewer`**

#### `__init__(canvas, title, data_field)`
初始化单绘图查看器。

**参数:**
- `data_field`: 要绘制的数据字段名（默认 "data"）

#### `update(msg)`
接收新消息并更新绘图数据。

#### `draw()`
绘制时序图。

---

#### 5.2 generic/MultiPlotViewer.py

**支持消息类型:**
- `geometry_msgs/Twist` (6 个数据字段)

**类: `MultiPlotViewer`**

#### `__init__(canvas, title, data_fields, columns)`
初始化多绘图查看器。

**参数:**
- `data_fields`: 要绘制的数据字段列表
- `columns`: 每行显示的图表数量（默认 3）

---

#### 5.3 generic/Space2DViewer.py

**职责:** 通用的 2D 空间查看器，支持缩放/平移，带有平滑动画。

**类: `Space2DViewer`**

#### `__init__(canvas, msg_decoder, title, offset_x, offset_y, scale)`
初始化 2D 空间查看器。

**参数:**
- `msg_decoder`: 回调函数，将 ROS 消息转换为绘图命令
- `scale`: 初始缩放值
- `offset_x/offset_y`: 初始偏移量

#### `keypress(c)`
处理键盘输入。

**支持的按键:**
- `+`/`=`: 放大
- `-`: 缩小
- `up/down/left/right`: 平移

#### `update(msg)`
接收新消息。

#### `draw()`
渲染 2D 视图，带有 0.5 秒动画过渡效果。

**绘图命令类型:**
- `COMMAND_TYPE_POINTS`: 绘制点集
- `COMMAND_TYPE_LINE`: 绘制线段

---

#### 5.4 generic/GenericImageViewer.py

**职责:** 通用图像查看器，支持 numpy 数组或 PIL.Image。

**类: `GenericImageViewer`**

#### `__init__(canvas, msg_decoder, title)`
初始化图像查看器。

#### `update(msg)`
接收新消息。

#### `draw()`
调整图像大小并渲染到终端。

---

#### 5.5 sensor_msgs/LaserScanViewer.py

**支持消息类型:** `sensor_msgs/LaserScan`

**类: `LaserScanViewer` (继承自 `Space2DViewer`)**

**功能:**
- 将激光扫描数据转换为 2D 极坐标点
- 显示 X (红色) 和 Y (绿色) 轴参考线
- 支持缩放和平移

---

#### 5.6 sensor_msgs/ImageViewer.py

**支持消息类型:** `sensor_msgs/Image`

**类: `ImageViewer` (继承自 `GenericImageViewer`)**

**支持的编码格式:**
- `bgr8`, `bgra8`, `rgb8`
- `mono8`, `8UC1`, `mono16`, `16UC1`

---

#### 5.7 sensor_msgs/CompressedImageViewer.py

**支持消息类型:** `sensor_msgs/CompressedImage`

**类: `CompressedImageViewer` (继承自 `GenericImageViewer`)**

使用 PIL 解码压缩图像（因 cv_bridge 不支持 Python 3）。

---

#### 5.8 sensor_msgs/PointCloud2Viewer.py

**支持消息类型:** `sensor_msgs/PointCloud2`

**类: `PointCloud2Viewer`**

**功能:**
- 3D 点云可视化
- 支持旋转 (left/right)、俯仰 (up/down)
- 支持缩放和相机距离调整
- 颜色按 Z 轴高度映射

#### `keypress(c)`
- `+`/`-`: 调整相机距离
- `[`/`]`: 调整点大小
- `left/right`: 旋转 (spin)
- `up/down`: 俯仰 (tilt)

---

#### 5.9 sensor_msgs/ImuViewer.py

**支持消息类型:** `sensor_msgs/Imu`

**类: `ImuViewer`**

**功能:**
- 显示 3 轴朝向 (yaw/pitch/roll) 时序图
- 显示 3 轴角速度时序图
- 显示 3 轴线加速度时序图

---

#### 5.10 sensor_msgs/NavSatFixViewer.py

**支持消息类型:** `sensor_msgs/NavSatFix`

**类: `NavSatFixViewer`**

**功能:**
- 从 OpenStreetMap 获取地图瓦片
- 显示 GPS 轨迹
- 支持缩放 (zoom level 5-19)

**注意:** 需要网络连接获取地图瓦片。

#### `keypress(c)`
- `+`/`=`: 放大
- `-`: 缩小

---

#### 5.11 nav_msgs/OccupancyGridViewer.py

**支持消息类型:** `nav_msgs/OccupancyGrid`

**类: `OccupancyGridViewer` (继承自 `GenericImageViewer`)**

**颜色编码:**
- 0-100: 灰度（白=空闲，黑=占用）
- <0: 橙色（未知）
- >100: 红色（错误）

---

#### 5.12 nav_msgs/OdometryViewer.py

**支持消息类型:** `nav_msgs/Odometry`

**类: `OdometryViewer` (继承自 `Space2DViewer`)**

**功能:**
- 显示 256 点位置轨迹
- 显示当前位置朝向箭头
- 自动居中到当前位置

---

#### 5.13 nav_msgs/PathViewer.py

**支持消息类型:** `nav_msgs/Path`

**类: `PathViewer` (继承自 `Space2DViewer`)**

**功能:**
- 显示路径点序列
- 自动居中到路径起点

---

### 6. rospy2/ (ROS2 兼容层)

**职责:** 提供 ROS1 风格的 API 用于 ROS2，使得代码可以同时支持 ROS1 和 ROS2。

**模块结构:**
- `__init__.py`: 核心实现
- `constants.py`: 日志级别常量

**主要类和函数:**

| 名称 | 说明 |
|------|------|
| `init_node()` | 初始化节点 |
| `Subscriber()` | 订阅话题 |
| `Publisher()` | 发布话题 |
| `Service()` | 创建服务 |
| `ServiceProxy()` | 服务客户端 |
| `spin()` | 处理回调 |
| `sleep()` | 延时 |
| `Rate()` | 频率控制 |
| `Timer()` | 定时器 |
| `Time()` | 时间对象 |
| `Duration()` | 持续时间 |
| `get_published_topics()` | 获取已发布话题 |
| `get_param()` / `set_param()` | 参数管理 |
| `signal_shutdown()` | 关闭信号 |
| `on_shutdown()` | 关闭回调 |
| `is_shutdown()` | 检查关闭状态 |

**异常类:**
- `ROSException`
- `ROSInitException`
- `ROSInterruptException`
- `ROSInternalException`
- `ROSSerializationException`

---

## 支持的消息类型列表

### nav_msgs
- `nav_msgs/OccupancyGrid`
- `nav_msgs/Odometry`
- `nav_msgs/Path`

### std_msgs
- `std_msgs/Bool`
- `std_msgs/Float32`
- `std_msgs/Float64`
- `std_msgs/Int8`
- `std_msgs/Int16`
- `std_msgs/Int32`
- `std_msgs/Int64`
- `std_msgs/UInt8`
- `std_msgs/UInt16`
- `std_msgs/UInt32`
- `std_msgs/UInt64`

### sensor_msgs
- `sensor_msgs/CompressedImage`
- `sensor_msgs/FluidPressure`
- `sensor_msgs/Illuminance`
- `sensor_msgs/Image`
- `sensor_msgs/Imu`
- `sensor_msgs/LaserScan`
- `sensor_msgs/NavSatFix`
- `sensor_msgs/PointCloud2`
- `sensor_msgs/Range`
- `sensor_msgs/RelativeHumidity`
- `sensor_msgs/Temperature`

### geometry_msgs
- `geometry_msgs/Twist`

---

## 项目运行方式

### 安装

**ROS1 (Catkin):**
```bash
cd ~/catkin_ws/src
git clone https://github.com/dheera/rosshow.git
cd ..
catkin build rosshow
source devel/setup.bash
```

**ROS2 (Colcon):**
```bash
cd ~/ros2_ws/src
git clone https://github.com/dheera/rosshow.git
cd ..
colcon build
source install/setup.bash
```

### 运行

**系统安装后:**
```bash
rosshow <topic_name>
```

**从工作区运行:**
```bash
rosrun rosshow rosshow <topic_name>    # ROS1
ros2 run rosshow rosshow <topic_name>  # ROS2
```

### 命令行选项

```bash
rosshow [-a] [-c1|-c4|-c24] [--reliable] [--transient-local] <topic>
```

| 选项 | 说明 |
|------|------|
| `-a, --ascii` | 使用纯 ASCII 字符（非 Unicode） |
| `-c1` | 强制单色模式 |
| `-c4` | 强制 16 色模式 |
| `-c24` | 强制 24-bit 真彩色 |
| `--reliable` | ROS2: 使用 RELIABLE QoS |
| `--transient-local` | ROS2: 使用 TRANSIENT_LOCAL QoS |

---

## 键盘控制

### 通用控制
- `Ctrl+C`: 退出程序

### Space2DViewer 派生类 (LaserScan, Odometry, Path)
- `+`/`=`: 放大
- `-`: 缩小
- `up/down/left/right`: 平移

### PointCloud2Viewer
- `+`/`-`: 调整相机距离
- `[`/`]`: 调整点大小
- `left/right`: 旋转
- `up/down`: 俯仰

### NavSatFixViewer
- `+`/`=`: 放大
- `-`: 缩小

---

## 文件清单

| 文件路径 | 说明 |
|----------|------|
| `CMakeLists.txt` | ROS1 Catkin 构建配置 |
| `setup.py` | Python 包配置 (ROS2) |
| `package.xml` | ROS 包元信息 |
| `setup.cfg` | 开发脚本配置 |
| `nodes/rosshow` | ROS1 启动脚本 |
| `ros-install-this` | 安装脚本 |
| `snap/snapcraft.yaml` | Snap 打包配置 |

---

## 技术细节

### Unicode Braille 渲染
`TermGraphics` 使用 Unicode Braille 点阵字符 (U+2800-U+28FF) 实现 2x4 的像素网格，每个字符代表 8 个像素点。这种方式可以在终端实现较高的"分辨率"。

### 颜色支持检测
自动检测终端类型和环境变量:
- `COLORTERM=truecolor/24bit` → 24-bit 真彩色
- `TERM=xterm-256color/xterm` → 16 色或更高
- 其他 → 16 色或单色

### 增量渲染
`draw()` 方法会跟踪缓冲区变化，只重绘发生变化的区域，优化性能。

### 动画系统
缩放、平移等操作使用 0.5 秒的线性插值实现平滑动画效果。
