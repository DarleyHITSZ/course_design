# AGENTS.md

面向在本仓库中工作的 AI 代理 / 自动化脚本。人类阅读请先看 [`README.md`](README.md)。

## 1. 项目事实

- **是什么**：机器人学课程设计"书法大咖"——HL-0905 六轴机械臂夹持毛笔写汉字"工大"。
  自写字形库 → 笔尖轨迹（含提按/抬笔/空行程）→ 关节轨迹（POE IK）→ Simscape Multibody
  回代校验 → 导出关节点位文件上机。
- **技术栈**：MATLAB R2026b（`D:\MATLAB\R2026b`）、Simulink、Simscape、Simscape Multibody。
  **不需要 Robotics Toolbox**，运动学沿用实验四的 POE 实现。
- **不是**：不是 git 仓库的历史遗留多项目结构（当前只有本课设）；没有单元测试框架、没有构建系统。
  正确性由 `assert` + `output/design_summary.json` 保证。
- **平台**：Windows。路径与模型含中文（`docs/课设要求.pdf`、模型子系统 `六轴机器人`/`轨迹规划`/`输出结果`），
  脚本按 UTF-8 处理。
- **体量**：`Matlab/HL-0905/` 约 1.5 GB（SLX + STL），`output/` 全是生成物。**不要提交 `output/`**。
- **状态**：书写流程已上机实跑通过（2026-11-23，实拍 `output/实际书写结果.jpg`）。首行 = 末行 = 待机位形（纸面中心上方 30 mm），
  可连续回放；控制器只接受**与现场位形一致**的首行。

## 2. 常用命令

```matlab
% 完整流程（规划 -> 仿真校验 -> 导出），约 2–4 分钟
addpath('Matlab'); run_design('工大')

% 只规划与导出（秒级；改字形/参数时先用它看 writing_preview.png）
run_design('工大', struct('simulate',false))

% 复跑已生成的轨迹并绘图
simulate_design('writing')

% 在 Mechanics Explorer 里回放真实机械臂（需 MATLAB 桌面，不能 -batch）
play_writing
simulate_design('writing',true,true)   % 复跑 + 六联图 + 回放

% 换字（需先在 CharLib.m 里实现该字）
run_design('大')
```

```bat
:: 无界面回归（首选）
D:\MATLAB\R2026b\bin\matlab.exe -batch "addpath('Matlab'); run_design('工大')"
```

跑 MATLAB 用 `-batch`，不要开 GUI；`run_design` 内部已把图窗设为不可见。

上机前先把机器人 jog 到 `run_design` 打印的「待机起始位形」（= 点位文件首行），否则控制器拒绝加载轨迹。
实跑基准（2026-11-23）：36163 点 / 36.16 s，首行 = 末行 = 待机位形。

## 3. 模块职责（改代码前先定位）

| 文件 | 职责 | 改动风险 |
| --- | --- | --- |
| `Matlab/BrushConfig.m` | **纯参数表**：只有要填的数字（mm/deg/mm·s⁻¹），无逻辑无校验。上机只填 `qTouch_deg`（触纸六关节角）与 `brushLength_mm`；调手感用 `standby_mm`/`pressOffset_mm` | 高：填错=撞纸/戳纸 |
| `Matlab/NormalizeConfig.m` | 把 BrushConfig 的 mm/deg 换成 SI（m/rad/s）；纸面位置与 `Rtool` 由 `run_design` 用 `qTouch_deg` 的 FK 反推（`NormalizeConfig` 只做单位换算与兜底） | 中：改字段名要同步这里 |
| `Matlab/CharLib.m` | 字形库：笔画几何（字格 uv）+ 宽度节点表 + 逐笔速度 | 中：只影响字形 |
| `Matlab/BrushPlanner.m` | 笔画 → 笔尖轨迹；抬笔/空行程/落笔/绘制/提笔 + **结尾回待机**分段，提按深度叠加 `pressOffset` | 高：轨迹结构与速度都在这里 |
| `Matlab/WriteTrajectory.m` | 笔尖 → 关节：逐点 IK + 笔尖雅可比反解 `qd/qdd` | 高：IK 种子/雅可比错了会毁掉整条轨迹 |
| `Matlab/render_writing.m` | 由深度渲染墨迹位图（预览用） | 低 |
| `Matlab/plot_writing.m` | 墨迹预览 + 六联图（含关节速度/加速度）；`lightax` 强制白底黑字（本机是深色主题） | 低 |
| `Matlab/play_writing.m` | 在 Mechanics Explorer 里用 HL-0905 实体模型回放轨迹 | 低：复用已校验轨迹 |
| `Matlab/run_design.m` | 主流程 + 断言 + 导出 + 回写模型 InitFcn；开头由 `qTouch_deg` 反解 `Rtool`/纸面/待机起始位形 `q0` | 高：验收逻辑与产物格式 |
| `Matlab/simulate_design.m` | 复跑轨迹、绘图 | 中 |
| `Matlab/prepare_model.m` | 由原始模型生成 `output/HL0905_2_writing.slx`；第二参数 `visualize` 控制是否开 Mechanics Explorer；顺带把 7 个 `ExtGeomFileName` 改成绝对路径 | 中：中文块名 + `newline` 拼接，见 §5 |
| `Matlab/HL0905Config.m` | `Xi`/`g0`/`qlim`，机器提取的数值，**不要手改** | 高：与模型几何强耦合 |
| `Matlab/Fkine.m` / `POE.m` | POE 正运动学 + 空间/末端雅可比 | 高：IK 与 `qd` 都依赖 |
| `Matlab/Ikine6s.m` | DLS 连续解 IK；要求 `config.seed`、`Xi/g0/qlim` 齐全 | 高：换策略会破坏逐点连续性 |
| `Matlab/MotionPlanning.m` | 同步归一化梯形：输出 `t/s/v/a/progress/speed/stages` | 高：所有段共用 |
| `Matlab/TrajGena.m` | 归一化曲线 → 关节 `q/qd/qdd`（实验四遗留，书写流程未用） | 中 |
| `Matlab/probe_model.m` | 生成 `output/probes.mat`（7 次单关节探针，标定用） | 低 |
| `sim_writing.m` | 一键脚本；顶部 `doPlay` 控制跑完是否调 `play_writing` | 低 |

数据流：`CharLib` → `BrushPlanner` → `WriteTrajectory` → `run_design`（预检 → `prepare_model` →
`sim` → 断言 → 导出 → InitFcn）。

## 4. 必须保持的不变量

1. **点位格式**：`output/writing_joint.txt` 6 列、`%.9f`、单位 **degree**、ASCII、无表头无时间列。
   下游控制器按列解析，多一列就崩。
2. **导出时序**：导出必须在仿真校验通过之后（`run_design.m` 里 `assert` 在 `export_points` 之前），
   失败不得留下半成品点位文件。
3. **采样周期** `Ts = 0.001`；`MotionPlanning` 阶段时长向上取整到 `Ts` 后必须重解峰值速度，
   保证 `s(end)=1`。
4. **时间对齐**：多轴只能共享一条归一化 `s(t)`，禁止各轴独立规划再缩放。
5. **IK 连续性**：逐点 IK 必须以上一点解为种子（`seed = Ikine6s(Tf,c)`），禁止固定种子重解。
6. **提按模型**：`d = w²/(8R)` 且 `d <= maxPress`，实际下压 `pressOffset + d`；宽度曲线与深度必须由同一套参数推导，
   不要单独硬编码 z 高度。（注意：实机上墨宽由笔锋决定，见 §5 高估条目）
7. **`qdd` 按时间求导**：`for j=1:6, qdd(:,j)=gradient(qd(:,j),Ts); end`。
   `gradient(qd,Ts)` 会沿关节维求差，产生上万 deg/s² 的假加速度（曾经踩过）。
8. **采样段总长**：`BrushPlanner.samplePath` 用 `L = pc(end)`（累加值）而不是 `sum(...)`，
   否则 `s(end)` 可能超出 `pc(end)` 一个 ulp，`interp1` 端点返回 NaN。
9. **预检阈值**：`run_design>precheck` 断言 q 有限、t 严格递增、`qlim` 内、`|qd| < qdMax`、
   `|qdd| < qddMax`。放宽阈值前先确认不是 bug。
10. **标定一致性**：POE 与模型探针最大偏差 < 1e-8（当前 8.55e-15）。改模型几何后必须重新提取
    `HL0905Config.m` 并重跑 `probe_model.m`。
11. **首尾位形一致**：`writing_joint.txt` 首行 == 末行（都停在纸面中心正上方 `standby_mm`）。控制器按首行对齐现场位形，
    改待机高度后必须把机器人 jog 到 `run_design` 打印的「待机起始位形」。
12. **派生不自填**：`Rtool`、`paperX/Y/Z`、`q0` 只能由 `qTouch_deg` 的 FK 派生；禁止手填 `paperZ` 或另外猜 `q0_deg`，
    否则纸面与实际不符（压不实/戳纸）或控制器报 joint distance too far。

## 5. 陷阱与故障排查

| 现象 | 原因 / 处理 |
| --- | --- |
| `IK did not converge` | 目标不可达/跨分支。检查 `flangeContact_mm`/`brushLength_mm`、`lift_mm`、字格偏移；先 `simulate=false` 定位失败点。 |
| 打印的纸面中心与预期差一截 | 示教时读的是法兰还是笔尖搞混了：`flangeContact_mm` 必须是**法兰中心**；漏写 `NormalizeConfig` 时 `paperX/Y/Z` 不存在，会直接报字段错误。 |
| 控制器报 `The joint distance of Current location and PPB start location are too far away` | 首行位形与现场差太多。`qTouch_deg` 填对后先看打印的「待机起始位形」，把机器人 jog 过去再回放（旧版首行 `[0 30 80 0 70 0]` 会差 J6 199°）。 |
| 纸面坐标/轨迹整体乱掉、`paper` 变成 3×3 | `run_design` 里纸面反推写成了 `(1000*Tt(1:3,4)+L*Rtool(:,3)')'`，广播成矩阵。必须 `(1000*Tt(1:3,4))' + L*Rtool(:,3)'`。 |
| IK 报 `Joint limit exceeded`（用触纸种子） | 还在用旧解析式 `Rtool`（`yaw=0`）。必须让 `Rtool` 取自 `qTouch_deg` 的 FK（实测笔杆偏竖直 0.92°）。 |
| 纸上墨宽比 `writing_preview.png` 细很多 | 球头模型在深压段高估 3–4 倍：`pressOffset_mm=4` 时模型算 8.9–13 mm，实测只有 2.8–3.3 mm。预览只当上限，别靠加压加粗。 |
| 起笔/收笔处有小墨点 | 落笔竖直段速度 12 mm/s 太慢、停顿积墨；`BrushPlanner` 落笔段提速到 18 或缩短落笔前停留。 |
| `段采样出现非有限值` | `samplePath` 里 NaN（端点 ulp 问题），见 §4.8。 |
| `关节加速度超限` | 常见原因是用 `gradient(qd,Ts)`（§4.7）；其次是真的太猛，降速度/加 `accDraw`。 |
| 墨迹预览字形怪 | `CharLib` 笔画顺序/方向不对（应从起笔到收笔），或宽度节点表首尾没收窄。 |
| 仿真里末端乱飞 | `prepare_model` 未尽到修补：确认 `Converterq` 的 `UdotUserProvided='2'`、两个 From Workspace 变量名 `[t,qd(:,1)]`/`[t,qdd(:,1)]` 已连到第 2/3 输入。 |
| 模型块路径报错 | 子系统名是中文，且 `'Simulink-PS'` 与 `'Converterq'` 之间是**真实换行**：用 `[... newline 'Converterq']`，不要写 `\n`。 |
| 改 `output/HL0905_2_writing.slx` 无效 | 它是构建产物，每次 `run_design` 从 `Matlab/HL-0905/HL0905_2.slx` 重新生成。要永久改请改 `prepare_model.m`。 |
| 打开 verified 模型仿真用旧轨迹 | `run_design` 把 `InitFcn` 指向 `output/model_workspace.mat`；要换轨迹重跑 `run_design`。 |
| `Rigid Transform ... 未连接的 F 端口` 警告 | 原模型遗留，无害。 |
| 仓库里出现 `slprj/`、`*.slxc`、`*.asv` | Simulink/Simscape 运行缓存，已在 `.gitignore` 忽略；可随时删（`rm -rf slprj *.slxc`），不影响产物。 |
| `output/design.log` 越积越大 | diary 追加写，可 `: > output/design.log` 清空；它不是交付物。 |
| `cp -r Matlab/HL-0905 .../Matlab/` 把模型铺平 | 目标 `Matlab/` 不存在时 `cp` 会把源目录内容铺开；先 `mkdir -p` 再 `cp -r`。 |
| 读 `docs/课设要求.pdf` 失败 | 该 PDF 含 `gs` 算子，`ops_read_pdf` 报 `UNSUPPORTED_DOCUMENT`；Word COM 转换会挂死。**用 Python + pypdf 取文本、pymupdf 渲染页面**（本机 `C:\Users\Darley\.dsh\dsh-runtimes\dsh-next-primary-runtime\dependencies\python\python.exe` 已装这两个包）。 |

## 6. 修改指南

**新增字形**（如 `永`）：

1. `Matlab/CharLib.m` 加 `case '永'`，用 `line2`/`bez2` 造笔画，宽度用 `mk(...)` 的节点表；
2. 笔画起点/终点顺序要与书写顺序一致（BrushPlanner 按顺序连接，段间自动抬笔/空行程）；
3. `run_design('永',struct('simulate',false))` 先用 `writing_preview.png` 调形；
4. 确认可达（纸面 z 附近 `x 0.49–0.67, y ±0.04` 已验证可达），再跑完整流程。

**调书写质量**：

- 粗细 → `CharLib` 宽度节点表（实机最有效）/ `maxPress` / `brushRadius`；加压深几乎不变宽，见 §5
- 压深 → `pressOffset`（整体加/减 2–4 mm，调墨色浓淡）
- 待机 → `standby_mm`（首尾停位高度，30 mm 左右；设 0 会在纸面中心收笔，留墨点）
- 快慢 → 每笔 `speed`、`speedDraw`；越快越细、墨越淡
- 抬笔高度 → `lift`（太小会拖墨，太大会增加空行程时间）
- 字距排版 → `cell`、`cellGap`，以及 `BrushPlanner` 里的 `off`

**改纸面/工位**（上机标定）：只改 `Matlab/BrushConfig.m` 的两个数——
`qTouch_deg`（笔尖刚触纸时的六个关节角，deg）与 `brushLength_mm`（法兰到笔尖距离）。
`run_design` 用 `qTouch_deg` 的 FK 一次派生三样东西：书写姿态 `Rtool`、纸面中心 `paperX/Y/Z`、待机起始位形 `q0`
（`paper = (1000*Tt(1:3,4))' + L*Rtool(:,3)'`，**转置必须在加法之前**），不要手填 `paperZ`，也不要另外猜 `q0_deg`；
只有读不到关节角时才退回 `flangeContact_mm`。运行时会打印「示教触纸姿态 / 待机起始位形 / 法兰接触坐标 / => 纸面（笔尖）中心」四行，
现场核对这四行即可，纸面高度差 1 mm 就会压不实或戳纸。

**改导出格式**：先看控制器文档，再同时改 `run_design>export_points` 与 §4.1 的格式约定。

## 7. 代码风格

- 标识符英文、注释中文；注释解释"为什么"（如 `% 用累加值作总长，保证 s<=pc(end)`）。
- 现有风格是紧凑短变量 + 分号连写，改动时保持一致，不要顺手重排整个函数。
- 数值判据一律用 `assert`，失败信息写清期望值（参考 `'关节加速度超限 %.1f deg/s^2'`）。
- 单位在注释中显式标注：`/mm`、`/deg`、`/rad`、`/s`、`/m`。混单位是本项目最易犯的错误。
- 不要引入工具箱依赖；保持"仅 MATLAB 基础函数 + Simulink/Simscape"。

## 8. 交付前检查清单

- [ ] `matlab -batch "addpath('Matlab'); run_design('工大')"` 通过，末尾打印完成行且无 `跳过仿真`。
- [ ] `output/design_summary.json` 中三条误差仍在 1e-9 量级以内（不是"刚过阈值"）。
- [ ] `output/writing_preview.png` 里能认出"工大"，笔画无断裂、无多余拖墨。
- [ ] `output/writing_joint.txt` 行数 = `design_summary.json.points`，6 列，值在 `qlim` 内。
- [ ] 首行 == 末行，且等于 `run_design` 打印的「待机起始位形」（上机前 jog 到那里）。
- [ ] 上机实拍存 `output/实际书写结果.jpg`，并把尺寸/墨宽/异常写进 README §8「上机实测」。
- [ ] 未手改 `output/` 下的生成物；诊断脚本命名带 `tmp_` 前缀并及时删除。
- [ ] 清掉缓存与中间物：`slprj/`、`*.slxc`、`*.asv`、根目录残留的 `tmp_*`；`output/` 只留 §6 列出的文件。
- [ ] README 中的参数表、命令、指标与代码一致（改接口时同步更新）。
