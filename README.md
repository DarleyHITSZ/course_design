# 书法大咖 · 六轴机械臂写毛笔字

哈尔滨工业大学（深圳）机器人学课程设计。用 HL-0905 六轴机械臂夹持毛笔，在桌面上书写汉字
**"工大"**：自写字形库与笔画规划，按"提按"控制笔画粗细，经 Simscape Multibody 回代校验后导出
关节轨迹点位文件，可直接送控制器上机。

![墨迹预览](output/writing_preview.png)

- 规划、验证、绘图全部在 MATLAB 中完成，**不依赖 Robotics Toolbox**（运动学沿用实验四的 POE 实现）
- 现有字形：`工`、`大`（简单笔画）；扩展方法见 [§9](#9-扩展与已知限制)
- 产物：`output/writing_joint.txt`（6 列关节角，deg）、墨迹预览、轨迹图、误差汇总

## 1. 快速开始

### 1.1 前置条件

- MATLAB **R2026b**（含 Simulink、Simscape、Simscape Multibody）
- `Matlab/HL-0905/HL0905_2.slx` + 7 个关节 STL 网格（约 1.5 GB）。**本工作区已包含全部 8 个文件**
  （已从实验四目录复制：`J0..J6_默认_sldprt.STL` + `HL0905_2.slx` + `HL-0905.xml`）。
  STL 体积过大、未纳入 git，所以**从仓库全新克隆**时才需要手动复制
  `实验四/Matlab/HL-0905/*.STL` 到此目录；缺失时 `prepare_model` 加载模型会报装配失败。

### 1.2 运行

```matlab
% 完整流程：字形 -> 笔尖轨迹 -> 关节轨迹 -> Simscape 校验 -> 导出
addpath('Matlab'); run_design('工大')

% 只做规划与导出（秒级，改字库/参数时用；交付前必须跑完整流程）
run_design('工大', struct('simulate',false))

% 复跑已保存的轨迹并绘图（不重新规划）
simulate_design('writing')

% 在 Mechanics Explorer 里回放真实机械臂（需 MATLAB 桌面，不能 -batch）
play_writing
```

```bat
:: 无界面执行（回归首选）
D:\MATLAB\R2026b\bin\matlab.exe -batch "addpath('Matlab'); run_design('工大')"
```

完整流程约 **2–4 分钟**：逐点 IK 约 90 s、Simscape 回代仿真 5–20 s，其余是 1.5 GB 模型加载与绘图。

### 1.3 看结果

| 文件 | 用途 |
| --- | --- |
| `output/writing_preview.png` | 墨迹预览（按深度渲染笔迹宽度），检查字形与粗细 |
| `output/writing_path.png` | 六联图：笔尖俯视/3D 轨迹、关节角、关节速度、关节加速度、提按与速度 |
| `output/writing_joint.txt` | 上机点位文件：6 列关节角（deg），无表头 |
| `output/design_summary.json` | 参数与仿真误差汇总 |

## 2. 目录结构

```text
course_design/
├── sim_writing.m              # 一键脚本（顶部 doPlay 控制跑完是否回放）
├── Matlab/
│   ├── BrushConfig.m          # 纯参数表（法兰接触点/笔长/速度/字格/容差/初始位形）
│   ├── NormalizeConfig.m      # mm/deg -> SI，并由法兰接触点反推纸面与 Rtool
│   ├── CharLib.m              # 字形库：笔画几何 + 笔迹宽度曲线
│   ├── BrushPlanner.m         # 笔画 -> 笔尖轨迹（抬笔/空行程/落笔/绘制/提笔 + 提按）
│   ├── WriteTrajectory.m      # 笔尖轨迹 -> 关节轨迹（逐点 IK + 雅可比反解 qd/qdd）
│   ├── render_writing.m       # 由压入深度渲染墨迹位图
│   ├── plot_writing.m         # 墨迹预览 + 六联图（白底，可直接进报告）
│   ├── play_writing.m         # 在 Mechanics Explorer 里回放真实机械臂
│   ├── run_design.m           # 主流程 + 验收断言 + 导出
│   ├── simulate_design.m      # 复跑轨迹并绘图
│   ├── prepare_model.m        # 由原始模型生成可注入轨迹的 output/HL0905_2_writing.slx
│   ├── HL0905Config.m         # Xi/g0/qlim（由模型几何提取，勿手改）
│   ├── Fkine.m / POE.m        # POE 正运动学 + 雅可比
│   ├── Ikine6s.m              # 阻尼最小二乘连续解 IK（种子接力）
│   ├── MotionPlanning.m       # 同步归一化梯形速度规划（多轴共用一条 s(t)）
│   ├── TrajGena.m             # 归一化曲线 -> 关节 q/qd/qdd
│   ├── probe_model.m          # 生成 output/probes.mat（标定探针）
│   └── HL-0905/               # 原始 Simscape 模型 + STL（约 1.5 GB）
├── output/                    # 全部生成物（勿手改）
└── docs/
    ├── requirements.md        # 课设要求摘要与映射
    ├── 课设要求.pdf            # 原始要求（9 页）
    └── 实验四指导书.pdf        # 运动学/规划推导来源
```

## 3. 流水线

```text
CharLib          字形 -> 笔画（字格系 uv + 宽度曲线 + 逐笔速度）
   │
BrushPlanner     笔画 -> 笔尖世界轨迹
   │ 抬笔(竖直) / 空行程(水平, 80 mm/s) / 落笔(竖直, 12 mm/s) / 绘制(逐笔速度)
   │ 每段用 MotionPlanning 做梯形规划，段首段尾速度为 0
   │ 压入深度 d = w²/(8R) → 笔尖 z = paperZ - d
   ▼
WriteTrajectory  笔尖轨迹 -> 关节轨迹
   │ IK 目标：法兰位姿 Tf = [Rtool, tip - L*Rz]；逐点以上一点为种子
   │ 关节速度：Jtip = [Jv - skew(raz)*Jw; Jw]，qd = Jtip \ [v;0;0;0]
   ▼
precheck         有限/单调/qlim/|qd|/|qdd| 断言
   ▼
prepare_model    修补原始模型：J1 允许外部 qd/qdd、To Workspace 改 Timeseries
   ▼
sim(in)          Simscape Multibody 回代（ode15s, MaxStep 1 ms）
   │ 位置误差 vs POE、姿态误差、笔尖路径偏差
   ▼
导出             writing_joint.txt / csv / png / design_summary.json / InitFcn 回写
```

## 4. 关键实现

### 4.1 末端位姿：笔是法兰坐标系的 -Z 方向

法兰在初始位形 `q0 = [0 30 80 0 70 0]°` 处的姿态为 `diag(1,-1,-1)`，即法兰 +Z 朝下，正好适合
"笔尖朝下"的书写姿态。工具坐标系取

```matlab
Rtool = Rz(yaw) * diag([1 -1 -1]) * rotx(tilt)   % BrushConfig.m
```

- `yaw`：绕世界 Z 转笔杆（调整笔画方向感），默认 0
- `tilt`：笔杆相对竖直的倾角，默认 0（垂直书写）；增大 tilt 需要改 `Rtool` 并重跑仿真
- 笔尖在法兰系中固定为 `[0;0;L]`，`L = 120 mm`，因此 **IK 目标 = 笔尖位置 - L*Rz**

### 4.2 提按：深度与宽度的物理关系

把笔尖近似为半径 `R = 4 mm` 的球冠，压入深度 `d` 与接触圆半径 `a` 满足 `a = sqrt(2Rd)`。
字库里直接给"笔迹宽度" `w = 2a`，于是

```matlab
d = w^2 / (8 R)        % BrushPlanner.m，深度上限 maxPress = 1.5 mm
```

宽度最大 6.5 mm（撇）对应深度约 1.32 mm（上限 1.5 mm）；收笔处宽度收到 20–30 %，深度降到 ~0.1 mm，
于是"提笔"即抬 z。
这套模型让 `CharLib` 里的宽度曲线（起笔收窄、行笔饱满、收笔出锋）直接变成可见的粗细变化。

### 4.3 速度与粗细

- 绘制速度：横 20 mm/s、竖 18 mm/s、撇捺 24 mm/s（`CharLib.m` 每笔的 `speed` 字段）
- 快慢影响"墨量"观感：慢笔更粗更黑，控制器/纸面不同需现场微调 `BrushConfig.speedDraw`
- 空行程 80 mm/s、落笔/抬笔 12 mm/s，均带梯形加减速（0.4 / 0.6 m/s²），避免起笔拖墨与抬笔甩墨

### 4.4 时间对齐（多轴同步）

所有轴共用同一条归一化梯形曲线 `s(t)`（`MotionPlanning.m`）：阶段时长按 `Ts` 向上取整后
重解峰值速度，保证 `s(end) = 1`。每个移动段一次规划，逐轴按 `q = q0 + s*Δq` 展开。
**不要各轴独立规划再缩放**——那样各轴到达时间不同，笔尖会画歪。

### 4.5 IK 连续性与关节速度

`Ikine6s` 用阻尼最小二乘（`dq = J'((JJ'+1e-10 I)\e)`，步长限幅 0.15 rad），必须以上一点解为种子：

```matlab
c.seed = seed; q(i,:) = Ikine6s(Tf, c); seed = q(i,:);   % WriteTrajectory.m
```

关节速度由**笔尖雅可比**反解（角速度约束为 0）：

```matlab
Jtip = [J(1:3,:) - skew(L*Rz)*J(4:6,:); J(4:6,:)];   % 6x6
qd   = Jtip \ [v; 0; 0; 0];
```

`qdd` 必须**按列（时间）逐列求导**：`for j=1:6, qdd(:,j)=gradient(qd(:,j),Ts); end`。
直接用 `gradient(qd,Ts)` 会沿"关节维"求差，得到上万 deg/s² 的假加速度。

### 4.6 墨迹预览

`render_writing.m` 把每个采样点按 `rw = sqrt(2R d)` 画成圆盘叠加成位图，再反色保存。
它不模拟真实墨扩散，但足以在不开仿真时快速判断字形、粗细与相对位置。

### 4.7 机械臂运动回放（Mechanics Explorer）

和实验四的码垛仿真一样，直接用模型里的 **HL-0905 实体模型**（Simscape Multibody）回放，
不需要另画示意几何：

```matlab
run_design('工大')                    % 先跑出轨迹（output/writing_simulation.mat）
play_writing                          % 打开 Mechanics Explorer 并回放

play_writing('gongda')                % 指定 tag
simulate_design('writing',true,true)  % 复跑 + 六联图 + 回放
run_design('工大',struct('play',true))% 规划+校验+回放，一条命令走完
```

`play_writing` 做的事：`prepare_model(cfg,true)` 打开 `SimMechanicsOpenEditorOnUpdate`，
把已验证的 `t/q/qd/qdd` 注入模型，按 `p.t(end)` 设 StopTime 后 `sim`。
仿真比实时快（约 5 s 跑完 32 s 轨迹），跑完后在 Mechanics Explorer 里按 **▶** 可 1 倍速重播，
时间轴能拖拽、可调速。

两个注意点：

- **Mechanics Explorer 只在 MATLAB 桌面里可见**，`-batch` 下不会弹窗（仿真照常通过，可用于回归）。
- 模型里的 7 个 `Solid` 几何原本只写了裸文件名（`J0_默认_sldprt.STL`），模型另存到 `output/`
  后按模型目录找不到 STL，实体会显示不出来。`prepare_model` 会把 `ExtGeomFileName` 改写成
  `Matlab/HL-0905/` 下的绝对路径（只影响显示，不影响动力学），所以工作区搬家后重跑一次
  `prepare_model` 即可恢复。

## 5. 参数表（`Matlab/BrushConfig.m`）

`BrushConfig.m` 里只有**要填的数字**，全部按示教器/尺子读数量取，不需要换算：

1. **`flangeContact_mm`**：手动把笔尖移到纸面中心、轻触纸面，抄下此时**法兰中心**的机器人坐标（mm）；
2. **`brushLength_mm`**：量法兰中心到笔尖的距离（mm）。

程序据此反推纸面位置 `tip = flange + L·Rtool(:,3)`（书写时法兰 +Z 朝下），
所以**不用另外填纸面高度**。默认值 `[580 0 640] + 120 mm` 正好对应仿真纸面 `(580, 0, 520) mm`。

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `flangeContact_mm` | `[580 0 640]` mm | **笔尖接触纸面中心时的法兰中心坐标**（基坐标系，示教器读数） |
| `brushLength_mm` | 120 mm | 笔尖到法兰中心的距离；换夹具必须重测 |
| `brushRadius_mm` | 4 mm | 笔头等效球半径，决定 `d = w²/(8R)` 的比例 |
| `maxPress_mm` | 1.5 mm | 最大压入深度，防止压弯笔杆/戳破纸 |
| `lift_mm` | 10 mm | 抬笔高度（相对纸面） |
| `yaw_deg`, `tilt_deg` | 0, 0 | 笔杆绕 Z 转角、相对竖直倾角（0=中锋垂直运笔） |
| `speedDraw_mm_s` | 20 mm/s | 常规绘制速度（越快越细越淡） |
| `speedPress_mm_s` | 12 mm/s | 落笔/提笔的竖直速度 |
| `speedMove_mm_s` | 80 mm/s | 空行程速度 |
| `accDraw_mm_s2` / `accMove_mm_s2` | 400 / 600 mm/s² | 绘制段 / 空行程加速度 |
| `cell_mm`, `cellGap_mm` | 80, 20 mm | 字格边长与字距，"工大"总宽 180 mm |
| `Ts_s` | 0.001 s | 采样周期（点位文件的时间粒度） |
| `q0_deg` | `[0 30 80 0 70 0]°` | 起始位形，同时决定笔尖起始位置 |
| `qdMax_rad_s`, `qddMax_rad_s2` | π/3（60 °/s）, π（180 °/s²） | 预检用的关节速度/加速度护栏。**为自定值，非厂家额定值**；实测书写 14.6 °/s、121.7 °/s²，约 4×/1.5× 余量。查到手册后建议取额定值的 60%~70% |
| `posTol_mm`, `oriTol_deg`, `pathTol_mm` | 0.01, 0.001, 0.01 | 仿真校验阈值（一般不动） |

单位换算与纸面反推集中在 `Matlab/NormalizeConfig.m`（`run_design`/`simulate_design` 开头调用），
`BrushConfig.m` 本身保持"纯参数表"。运行时会打印两行供现场核对：

```text
法兰接触坐标 (580.0, 0.0, 640.0) mm，笔长 L=120 mm，字格 80 mm
=> 纸面（笔尖）中心 (580.0, 0.0, 520.0) mm，抬笔高度 10 mm
```

## 6. 输出文件（`output/`）

| 文件 | 内容 |
| --- | --- |
| `writing_joint.txt` | **上机点位文件**：N×6 关节角（deg），`%.9f`，无表头/时间列 |
| `writing_timeline.csv` | 时间序列：t, q, qd, qdd（rad） |
| `writing_tip_path.csv` | 笔尖世界轨迹（mm）+ 压入深度、速度、段名 |
| `writing_simulation.mat` | `p`（关节轨迹）、`plan`（笔尖计划）、`metric`（误差） |
| `writing_preview.png` | 墨迹预览 |
| `writing_path.png` | 六联图：俯视/3D/关节角/关节速度/关节加速度/提按与速度 |
| `design_summary.json` | 参数、笔尖范围、误差指标、文件索引 |
| `model_workspace.mat` | `t,q,qd,qdd`，供模型 InitFcn 加载 |
| `HL0905_2_writing.slx` | 修补后的模型，InitFcn 已指向 `model_workspace.mat` |
| `design.log` | 运行日志（追加写；可随时清空，不影响产物） |

## 7. 上机流程

**只需改 `BrushConfig.m` 里的两个数**（都按示教器/尺子读数，见 §5）：

```matlab
cfg.flangeContact_mm=[602 15 638];   % 笔尖接触纸面中心时的法兰中心（示教器读数，mm）
cfg.brushLength_mm=125;              % 法兰中心到笔尖的距离（尺子量，mm）
```

1. **对刀（必做）**：把笔尖移到纸面中心轻触纸面 → 抄下示教器上**法兰中心**坐标 → `flangeContact_mm`；
   量法兰到笔尖距离 → `brushLength_mm`；`q0_deg` 改成现场初始位形。纸面高度差 1 mm 就会压不实或戳纸。
2. **空跑**：`lift_mm` 加大了跑一遍（或把 `flangeContact_mm` 的 z 抬高 20 mm），确认轨迹无碰撞、无奇异。
3. **慢速试写**：`speedDraw_mm_s` 降到 10、`maxPress_mm` 降到 1.0，试写单笔，看墨迹宽度。
4. **正式书写**：恢复参数跑 `run_design('工大')`，按控制器要求加载 `output/writing_joint.txt`
   （6 列 deg，逐点位置指令；若控制器要求速度，用 `writing_timeline.csv`）。
5. **收尾**：提笔离开纸面、清洁笔尖；把 `design_summary.json` 与墨迹照片存档，供汇报使用。

> 安全：首次上机务必把示教器速度倍率降到 10 %，手放急停；笔尖接触纸面前不要开启连续执行。

## 8. 验证与指标

`run_design` 在导出前强制三条断言（阈值见 §5）：

| 指标 | 含义 | 实测（当前产物，32341 点 / 32.34 s） |
| --- | --- | --- |
| `max_position_error_mm` | Simscape 末端位置 vs POE 复算 | 3.81e-12 mm |
| `max_orientation_error_deg` | 姿态偏差 | 1.23e-13 ° |
| `max_path_error_mm` | 笔尖轨迹 vs 规划路径 | 1.99e-8 mm |
| `max_speed_deg_s` / `max_acceleration_deg_s2` | 关节速度/加速度峰值 | 14.56 °/s / 121.66 °/s²（限值 60 / 180） |

另有一条几何标定断言（`run_exp4` 遗留）：POE 与模型探针 `output/probes.mat` 的最大偏差 < 1e-8
（当前 8.55e-15）。**换机器人/改模型后必须重新标定**，方法见 `docs/实验四指导书.pdf` §3。

## 9. 扩展与已知限制

### 新增字形

1. 在 `Matlab/CharLib.m` 的 `switch` 中加一个 `case`，用 `line2`（直线）或 `bez2`（二次贝塞尔）描述笔画；
2. 宽度用节点表 `[t0 w0; t1 w1; ...]`（归一化弧长 -> 宽度比例），撇捺用 4–5 个节点做提按；
3. 每笔可单独给 `speed`（mm/s）；
4. 运行 `run_design('新字', struct('simulate',false))`，先看 `writing_preview.png` 调形，再跑完整流程。

字格坐标系：原点在字格中心，u 向右、v 向上，单位为米；笔画按弧长 0.5 mm 重采样。

### 已知限制

- 只在**平面**上书写（纸面水平）；无曲面/自适应高度
- 只有开环位置控制，**无力控/接触力反馈**，纸面不平靠 `maxPress` 留余量
- 字库仅 `工`、`大`，笔画是单线 + 宽度曲线，不是真正的楷书轮廓填充
- 未考虑墨水扩散、笔毛分叉、纸张吸墨等物理过程
- 实体回放（`play_writing`）依赖 MATLAB 桌面；`-batch` 下只做数值校验，不弹 Mechanics Explorer
- `output/HL0905_2_writing.slx` 是构建产物：每次 `run_design` 都从
  `Matlab/HL-0905/HL0905_2.slx` 重新生成，不要手工改它

### 后续可做

- 楷书困难任务（`永`）：需要更细的提按节点、转折处圆角、笔画间的"笔断意连"
- 用 `char` 轮廓（如矢量字体）自动生成笔画中线 + 宽度，替代手写字库
- 加力传感器做恒力控制，纸面不平也能保持墨宽一致
- 多字排版（横向/纵向）、落款与印章

## 10. 相关文档

- [`docs/requirements.md`](docs/requirements.md)：课设要求摘要与实现映射
- [`AGENTS.md`](AGENTS.md)：给 AI 代理/自动化脚本的协作约定（模块职责、不变量、陷阱）
- [`docs/实验四指导书.pdf`](docs/实验四指导书.pdf)：POE 运动学、梯形规划、Simscape 建模范例
- 运动学标定与模型修补细节：`Matlab/HL0905Config.m`、`Matlab/prepare_model.m`、`Matlab/probe_model.m`
