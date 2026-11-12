function cfg=NormalizeConfig(cfg)
% NormalizeConfig  BrushConfig 的 mm/deg 参数 -> 内部 SI 单位（m / rad / s），并反推纸面位置。
%
%   cfg = NormalizeConfig(BrushConfig());
%
% 纸面（笔尖）位置由法兰接触坐标反推：tip = flange + L*Rtool(:,3)，书写时法兰 +Z 朝下。
% 重复调用无副作用（已有 SI 字段时直接返回）。
if isfield(cfg,'normalized')&&cfg.normalized, return; end

cfg.brushLength = cfg.brushLength_mm/1000;
cfg.brushRadius = cfg.brushRadius_mm/1000;
cfg.maxPress    = cfg.maxPress_mm/1000;
cfg.lift        = cfg.lift_mm/1000;
cfg.speedDraw   = cfg.speedDraw_mm_s/1000;
cfg.speedPress  = cfg.speedPress_mm_s/1000;
cfg.speedMove   = cfg.speedMove_mm_s/1000;
cfg.accDraw     = cfg.accDraw_mm_s2/1000;
cfg.accMove     = cfg.accMove_mm_s2/1000;
cfg.cell        = cfg.cell_mm/1000;
cfg.cellGap     = cfg.cellGap_mm/1000;
cfg.Ts          = cfg.Ts_s;
cfg.qdMax       = cfg.qdMax_rad_s;
cfg.qddMax      = cfg.qddMax_rad_s2;
cfg.q0          = pi/180*cfg.q0_deg(:)';

% 书写姿态：法兰 +Z 朝下（与 HL0905 零位一致），yaw 绕世界 Z，tilt 为侧倾
cfg.yaw = pi/180*cfg.yaw_deg;
cfg.tilt= pi/180*cfg.tilt_deg;
cfg.Rtool=[cos(cfg.yaw) -sin(cfg.yaw) 0; sin(cfg.yaw) cos(cfg.yaw) 0; 0 0 1] ...
          *diag([1 -1 -1])*rotx(cfg.tilt);

% 笔尖接触点 = 法兰 + L*Rtool(:,3)
tip_mm=cfg.flangeContact_mm(:)'+cfg.brushLength_mm*cfg.Rtool(:,3)';
cfg.paperX=tip_mm(1)/1000;
cfg.paperY=tip_mm(2)/1000;
cfg.paperZ=tip_mm(3)/1000;
cfg.paper_mm=tip_mm;
cfg.normalized=true;
end

function R=rotx(a)
R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
end
