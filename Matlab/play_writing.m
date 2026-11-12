function play_writing(tag)
% play_writing  在 Mechanics Explorer 里用 HL-0905 实体模型回放书写轨迹。
%
%   play_writing               % 回放 output/writing_simulation.mat
%   play_writing('gongda')     % 指定 tag
%
% 与实验四码垛仿真一致：把已校验的关节轨迹注入 Simscape Multibody 模型，
% 仿真时 Mechanics Explorer 显示真实连杆几何的运动。
% 注意：Mechanics Explorer 只在 MATLAB 桌面里可见；-batch 下不会弹窗（但仿真照常）。
% 仿真比实时快（约 5 s 跑完 32 s 轨迹），所以跑完后按 Mechanics Explorer 的 ▶
% 可 1 倍速回放，或用时间轴拖拽、调速。
if nargin<1||isempty(tag), tag='writing'; end
cfg=NormalizeConfig(BrushConfig()); c=HL0905Config();
mat=fullfile(cfg.root,'output',[tag '_simulation.mat']);
assert(isfile(mat),'缺少 %s，请先运行 run_design(''工大'')',mat);
S=load(mat); p=S.p;

mdl=prepare_model(cfg,true);
assignin('base','t',p.t);
assignin('base','q',p.q);
assignin('base','qd',p.qd);
assignin('base','qdd',p.qdd);
set_param(mdl,'StopTime',sprintf('%.12g',p.t(end)), ...
 'Solver','ode15s','MaxStep','0.001','RelTol','1e-7','AbsTol','1e-9');

fprintf('回放 %s：%d 点，%.2f s（Mechanics Explorer 已打开）\n',tag,numel(p.t),p.t(end));
fprintf('若动画已经跑完：在 Mechanics Explorer 里按 ▶ 可 1 倍速重播，时间轴可拖拽/调速。\n');
sim(mdl);
fprintf('仿真结束。\n');
end
