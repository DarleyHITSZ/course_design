function r=run_design(chars,opts)
% run_design  课程设计主流程：字形 -> 笔尖轨迹 -> 关节轨迹 -> Simscape 校验 -> 导出。
%
%   run_design('工大')                            % 完整流程（含 Simscape 仿真校验）
%   run_design('工大',struct('simulate',false))   % 仅规划与导出（秒级迭代，交付前务必跑完整流程）
%   run_design('永')                              % 楷书笔画
%   run_design('工大',struct('play',true))         % 跑完并在 Mechanics Explorer 里回放
%
% 产物（output/）：
%   <tag>_joint.txt      6 列关节角（deg），无表头，可直接送控制器/PPB
%   <tag>_simulation.mat p(关节轨迹) + plan(笔尖计划) + metric(仿真误差)
%   <tag>_preview.png    墨迹预览（按压入深度渲染笔迹宽度）
%   <tag>_path.png       笔尖俯视/3D/关节曲线/深度速度
%   <tag>_timeline.csv   时间序列（rad）
%   <tag>_tip_path.csv   笔尖世界轨迹（mm）+ 深度/速度/段名
%   design_summary.json  参数与误差汇总
%   HL0905_2_writing.slx 可注入轨迹的模型（InitFcn 指向 model_workspace.mat）
if nargin<1||isempty(chars),chars='工大';end
if nargin<2,opts=struct();end
if ~isfield(opts,'simulate'),opts.simulate=true;end
if ~isfield(opts,'tag'),opts.tag='writing';end
if ~isfield(opts,'play'),opts.play=false;end
if isfield(opts,'cfg')&&~isempty(opts.cfg)
 cfg=NormalizeConfig(opts.cfg);   % 允许调用方传入改过参数的 BrushConfig()
else
 cfg=NormalizeConfig(BrushConfig());
end
c=HL0905Config();root=cfg.root;
% 若给了示教触纸关节角，用 FK 反推书写姿态与纸面中心：比手填 yaw/tilt 更准，
% 且保证起始位形＝上机时的实际位形（否则控制器会报起点太远）。
if isfield(cfg,'qTouch_deg')&&~isempty(cfg.qTouch_deg)
 qt=cfg.qTouch_deg(:)*pi/180; Tt=Fkine(c.Xi,qt,c.g0);
 cfg.Rtool=Tt(1:3,1:3);
 cfg.paper_mm=(1000*Tt(1:3,4))'+cfg.brushLength_mm*cfg.Rtool(:,3)';
 cfg.paperX=cfg.paper_mm(1)/1000;cfg.paperY=cfg.paper_mm(2)/1000;cfg.paperZ=cfg.paper_mm(3)/1000;
 fprintf('示教触纸姿态：法兰 (%.1f, %.1f, %.1f) mm，笔杆偏离竖直 %.2f deg\n', ...
         1000*Tt(1:3,4),acosd(max(-1,min(1,-cfg.Rtool(3,3)))));
 % 待机位：纸面中心正上方 standby_mm；起始位形由 IK 反解，保证文件首行=现场可停到的位形
 cfg.standbyTip=[cfg.paperX cfg.paperY cfg.paperZ+cfg.standby];
 Tb=eye(4);Tb(1:3,1:3)=cfg.Rtool;Tb(1:3,4)=cfg.standbyTip(:)-cfg.brushLength*cfg.Rtool(:,3);
 c.seed=qt; qq=Ikine6s(Tb,c); cfg.q0=qq(:)';
 fprintf('待机起始位形 (deg): [%.3f %.3f %.3f %.3f %.3f %.3f]\n',cfg.q0*180/pi);
end
out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end
diary(fullfile(out,'design.log'));cleanup=onCleanup(@()diary('off'));%#ok<NASGU>
fprintf('\n==== 机械臂毛笔书写：%s ====\n',chars);
fprintf('法兰接触坐标 (%.1f, %.1f, %.1f) mm，笔长 L=%.0f mm，字格 %.0f mm\n', ...
        cfg.flangeContact_mm,cfg.brushLength*1000,cfg.cell*1000);
fprintf('=> 纸面（笔尖）中心 (%.1f, %.1f, %.1f) mm，抬笔高度 %.0f mm\n', ...
        cfg.paper_mm,cfg.lift*1000);

% 1) 起笔位姿（由 q0 推出笔尖位置，不硬编码纸面中心）
T0=Fkine(c.Xi,cfg.q0,c.g0);
startTip=(T0(1:3,4)+cfg.Rtool*[0;0;cfg.brushLength])';
fprintf('起始笔尖 (%.3f, %.3f, %.3f) m\n',startTip);

% 2) 字形 -> 笔尖轨迹（抬笔/空行程/落笔/绘制/提笔、提按深度、逐段速度）
strokes=CharLib(chars,cfg);
plan=BrushPlanner(strokes,cfg,startTip);
N=size(plan.tip,1);
fprintf('笔画 %d 条，笔尖轨迹 %d 点 / %.2f s\n',numel(strokes),N,(N-1)*cfg.Ts);

% 3) 笔尖轨迹 -> 关节轨迹（逐点 IK + 笔尖雅可比）
p=WriteTrajectory(plan,c,cfg);
precheck(p,c,cfg);

% 4) Simscape 回代校验（不通过则不导出，避免半成品点位文件）
metric=[];
if opts.simulate
 [metric,simout]=simulate_run(p,cfg,c); %#ok<ASGLU>
 assert(metric.max_position_error_mm<cfg.posTol_mm && ...
        metric.max_orientation_error_deg<cfg.oriTol_deg && ...
        metric.max_path_error_mm<cfg.pathTol_mm,'仿真校验未通过，检查关节轨迹/模型装配');
 save(fullfile(out,[opts.tag '_simulation.mat']),'p','plan','metric');
else
 fprintf('跳过仿真（simulate=false），仅做规划与导出\n');
end

% 5) 导出
export_points(p,out,opts.tag);
export_csv(p,out,opts.tag);
plot_writing(p,plan,cfg,out,opts.tag);
write_summary(chars,p,plan,cfg,metric,out,opts.tag);
if opts.simulate, set_model_init(p,out); end

r.p=p;r.plan=plan;r.metric=metric;
fprintf('\n完成：output/%s_joint.txt（%d 行，%.2f s）\n',opts.tag,N,(N-1)*cfg.Ts);
fprintf('墨迹预览：output/%s_preview.png   轨迹图：output/%s_path.png\n',opts.tag,opts.tag);
if opts.play, play_writing(opts.tag); end
end

% ==================================================================
function precheck(p,c,cfg)
assert(all(isfinite(p.q),'all')&&all(diff(p.t)>0),'轨迹时间或关节角非法');
assert(all(p.q>=c.qlim(:,1)'-1e-8&p.q<=c.qlim(:,2)'+1e-8,'all'),'关节限位越界');
vmax=max(abs(p.qd),[],'all');amax=max(abs(p.qdd),[],'all');
assert(vmax<cfg.qdMax,'关节速度超限 %.1f deg/s',vmax*180/pi);
assert(amax<cfg.qddMax,'关节加速度超限 %.1f deg/s^2',amax*180/pi);
fprintf('预检通过：max|qd|=%.1f deg/s，max|qdd|=%.1f deg/s^2\n',vmax*180/pi,amax*180/pi);
end

function [metric,simout]=simulate_run(p,cfg,c)
mdl=prepare_model(cfg);
in=Simulink.SimulationInput(mdl);
in=in.setVariable('t',p.t).setVariable('q',p.q).setVariable('qd',p.qd).setVariable('qdd',p.qdd);
in=in.setModelParameter('StopTime',sprintf('%.12g',p.t(end)),'ReturnWorkspaceOutputs','on', ...
 'Solver','ode15s','MaxStep','0.001','RelTol','1e-7','AbsTol','1e-9');
fprintf('Simscape 仿真中（%d 点，%.2f s）...\n',numel(p.t),p.t(end));
t0=tic;simout=sim(in);fprintf('仿真耗时 %.1f s\n',toc(t0));
ts=simout.Px.Time;
pos=[simout.Px.Data(:) simout.Py.Data(:) simout.Pz.Data(:)];   % mm
qsim=interp1(p.t,p.q,ts);
ref=zeros(numel(ts),3);er=zeros(numel(ts),1);
for n=1:numel(ts)
 T=Fkine(c.Xi,qsim(n,:),c.g0);
 ref(n,:)=T(1:3,4)'*1000;
 D=T(1:3,1:3)*simout.R.Data(:,:,n)';
 er(n)=atan2(norm([D(3,2)-D(2,3);D(1,3)-D(3,1);D(2,1)-D(1,2)])/2,(trace(D)-1)/2)*180/pi;
end
ep=vecnorm(pos-ref,2,2);
raz=(cfg.brushLength*cfg.Rtool(:,3))'*1000;      % 法兰->笔尖（mm）
tipSim=pos+raz;
tipPlan=p.target*1000;
epath=vecnorm(tipSim-tipPlan(dsearchn(tipPlan,tipSim),:),2,2);
metric.points=numel(p.t);
metric.duration_s=p.t(end);
metric.max_position_error_mm=max(ep);
metric.rms_position_error_mm=sqrt(mean(ep.^2));
metric.max_orientation_error_deg=max(er);
metric.max_path_error_mm=max(epath);
metric.max_speed_deg_s=max(abs(p.qd),[],'all')*180/pi;
metric.max_acceleration_deg_s2=max(abs(p.qdd),[],'all')*180/pi;
metric.max_press_depth_mm=max(p.depth)*1000;
metric.max_speed_mm_s=max(p.speed)*1000;
disp(metric);
end

function export_points(p,out,tag)
fid=fopen(fullfile(out,[tag '_joint.txt']),'w');assert(fid>0,'无法写点位文件');
fprintf(fid,'%.9f %.9f %.9f %.9f %.9f %.9f\n',(p.q*180/pi)');
fclose(fid);
end

function export_csv(p,out,tag)
M=[p.t p.q p.qd p.qdd];
fmt='%.9f';for k=1:size(M,2)-1,fmt=[fmt ',%.9f'];end%#ok<AGROW>
fid=fopen(fullfile(out,[tag '_timeline.csv']),'w');assert(fid>0);
hdr=['t,q1,q2,q3,q4,q5,q6,qd1,qd2,qd3,qd4,qd5,qd6,' ...
     'qdd1,qdd2,qdd3,qdd4,qdd5,qdd6'];
fprintf(fid,'%s\n',hdr);
fprintf(fid,[fmt '\n'],M');
fclose(fid);
fid=fopen(fullfile(out,[tag '_tip_path.csv']),'w');assert(fid>0);
fprintf(fid,'t,x_mm,y_mm,z_mm,depth_mm,speed_mm_s,segment\n');
for i=1:numel(p.t)
 fprintf(fid,'%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s\n',p.t(i), ...
   p.target(i,1)*1000,p.target(i,2)*1000,p.target(i,3)*1000, ...
   p.depth(i)*1000,p.speed(i)*1000,p.segName{i});
end
fclose(fid);
end


function write_summary(chars,p,plan,cfg,metric,out,tag)
s=struct();
s.chars=chars;
s.generated_at=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
s.points=numel(p.t);
s.duration_s=p.t(end);
s.paper_mm=cfg.paper_mm;
s.brush=struct('length_m',cfg.brushLength,'radius_m',cfg.brushRadius, ...
 'max_press_m',cfg.maxPress,'tool_rotation',cfg.Rtool);
s.speeds_mm_s=struct('draw',cfg.speedDraw*1000,'press',cfg.speedPress*1000, ...
 'move',cfg.speedMove*1000);
s.plan=struct('tip_range_x_m',[min(plan.tip(:,1)) max(plan.tip(:,1))], ...
 'tip_range_y_m',[min(plan.tip(:,2)) max(plan.tip(:,2))], ...
 'max_press_depth_mm',max(plan.depth)*1000,'segments',numel(unique(plan.segName)));
s.metric=metric;
s.files=struct('points',[tag '_joint.txt'],'preview',[tag '_preview.png'], ...
 'path',[tag '_path.png'],'timeline',[tag '_timeline.csv'], ...
 'tip_path',[tag '_tip_path.csv'],'workspace','model_workspace.mat');
fid=fopen(fullfile(out,'design_summary.json'),'w');assert(fid>0);
fprintf(fid,'%s\n',jsonencode(s,'PrettyPrint',true));fclose(fid);
end

function set_model_init(p,out)
t=p.t;q=p.q;qd=p.qd;qdd=p.qdd;
save(fullfile(out,'model_workspace.mat'),'t','q','qd','qdd');
mdlfile=fullfile(out,'HL0905_2_writing.slx');
if ~isfile(mdlfile),return;end
if bdIsLoaded('HL0905_2_writing')
 mdl='HL0905_2_writing';
else
 load_system(mdlfile);mdl='HL0905_2_writing';
end
init=['exp4_model_dir=fileparts(get_param(bdroot,''FileName''));' ...
      'load(fullfile(exp4_model_dir,''model_workspace.mat''));'];
set_param(mdl,'InitFcn',init,'StopTime',sprintf('%.12g',p.t(end)));
save_system(mdl);close_system(mdl,0);
end
