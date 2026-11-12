function cfg=BrushConfig()
% BrushConfig  上机参数。**只需要改下面这组数字**，单位见每行注释（mm / deg / mm·s⁻¹）。
%
% 对刀方法：手动把笔尖移到纸面中心并轻触纸面，抄下此时**法兰中心**的坐标填进 flangeContact_mm；
% 再量一次法兰中心到笔尖的距离填 brushLength_mm。程序据此反推纸面（笔尖）位置，
% 不需要另外算纸面高度。仿真里默认值对应纸面 (580, 0, 520) mm。

% ---------- 上机填写区 ----------
cfg.flangeContact_mm=[580 0 640];   % 笔尖接触纸面中心时的法兰中心坐标（基坐标系，mm）
cfg.brushLength_mm=120;             % 笔尖到法兰中心的距离（mm）
cfg.brushRadius_mm=4;               % 笔头等效球半径（mm），笔迹宽度<->压入深度换算用
cfg.maxPress_mm=1.5;                % 允许的最大压入深度（mm）
cfg.lift_mm=10;                     % 抬笔高度（mm）
cfg.yaw_deg=0;                      % 笔杆绕世界 Z 转角（deg）
cfg.tilt_deg=0;                     % 笔杆侧倾角（deg），0=垂直运笔

cfg.speedDraw_mm_s=20;              % 绘制速度（横、竖，mm/s）
cfg.speedPress_mm_s=12;             % 落笔/提笔的竖直速度（mm/s）
cfg.speedMove_mm_s=80;              % 空行程速度（mm/s）
cfg.accDraw_mm_s2=400;              % 绘制段加速度（mm/s²）
cfg.accMove_mm_s2=600;              % 空行程加速度（mm/s²）

cfg.cell_mm=80;                     % 单字格边长（mm）
cfg.cellGap_mm=20;                  % 字间距（mm）
cfg.Ts_s=0.001;                     % 采样周期（s）
cfg.q0_deg=[0 30 80 0 70 0];        % 起始关节角（deg）

% ---------- 校验阈值（一般不动）----------
% 下面两个上限是"自定护栏"，不是 HL-0905 的厂家额定值：只在 precheck 里断言，
% 不参与规划。实测书写 max|qd|=14.8 deg/s、max|qdd|=126.5 deg/s^2，留约 4x/1.4x 余量。
% 若查到控制器/机器人手册的额定关节速度与加速度，改成手册值的 60%~70% 更有依据。
cfg.qdMax_rad_s=pi/3;               % 关节速度上限 60 deg/s（rad/s）
cfg.qddMax_rad_s2=pi;               % 关节加速度上限 180 deg/s^2（rad/s²）
cfg.posTol_mm=0.01;                 % 仿真位置残差上限（mm）
cfg.oriTol_deg=0.001;               % 仿真姿态残差上限（deg）
cfg.pathTol_mm=0.01;                % 路径偏差上限（mm）

cfg.root=fileparts(fileparts(mfilename('fullpath')));   % course_design/ 根目录
end
