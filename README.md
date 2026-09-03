# UAV







## 项目简介



基于 STM32F405 的四轴无人机飞控系统，集成 LQR 与 PID 控制算法、姿态解算、FreeRTOS 实时操作系统、DShot 电调驱动、IMU 多传感器融合（陀螺仪/加速度计）、蓝牙与 PS2 遥控、光流定高，配套 MATLAB/Simulink/Simscape 仿真验证与 Keil MDK 嵌入式开发完整资料。
STM32F405-based quadcopter flight control system integrating LQR and PID control algorithms, attitude estimation, FreeRTOS real-time operating system, DShot ESC driver, IMU multi-sensor fusion (gyroscope/accelerometer), Bluetooth and PS2 remote control, and optical flow altitude hold, with complete MATLAB/Simulink/Simscape simulation verification and Keil MDK embedded development resources.


## 目录结构



```

无人机 /

├── 四轴无人机 /

│   ├── 程序源码 /

│   │   ├── keil 工程 /                 # Keil MDK 工程（LQR / PID / 电调例程）

│   │   └── MATLAB 程序 /        # MATLAB 仿真与算法验证

│   ├── 传感器手册 /                   # 各传感器数据手册

│   ├── 原理图 /                          # 飞控板原理图

│   ├── 飞行器油门拟合文件 /    # 油门曲线拟合数据

│   └── *.pdf                               # 开发手册、使用手册、调试经验

└── 初学者复刻 /                        # 入门级复刻资料与烧录工具

```



## 开发环境



- **IDE**：Keil MDK 5

- **芯片**：STM32F405RGT6

- **系统**：FreeRTOS

- **仿真**：MATLAB / Simulink / Simscape



## 大文件下载



以下文件体积较大，未纳入 git 版本管理，请通过百度网盘下载：



| 文件 | 大小 | 说明 | 下载链接 |

|------|------|------|----------|

| LQR.avi | 353 MB | Simscape 四轴飞行器 LQR 控制仿真演示视频 | 百度网盘：https://pan.baidu.com/s/13S9hLy8eEmibUFdEDGfKNA?pwd=5m56  |

| 无人机配套软件工具.zip | ~116 MB | 串口调试助手、虚拟串口、无线烧录助手、蓝牙透传软件、FlyControl APP 等 | 百度网盘 ：https://pan.baidu.com/s/14PjQdfBVtd2VivTcREuWbg?pwd=459s  |



## 编译说明



1. 使用 Keil MDK 5 打开 `程序源码/keil工程/LQR/MDK-ARM/Fly_LQR.uvprojx`

2. 选择对应目标编译生成固件

3. 使用无线烧录助手或 ST-Link 烧录到飞控板



## 注意事项



- 编译产物（`.o`、`.crf`、`.axf` 等）已通过 `.gitignore` 过滤，clone 后需自行编译

- 第三方软件工具不纳入 git，请到上方百度网盘链接下载

- 仿真视频不纳入 git，请到上方百度网盘链接下载



