function [simout,trajectory]=simulate_design(tag,showFigures,doPlay)
% simulate_design  复跑已生成的书写轨迹并绘图（不重新规划，秒级）。
%
%   simulate_design                    % 复跑 output/writing_simulation.mat
%   simulate_design('gongda',false)    % 指定 tag 且不显示图形
%   simulate_design('writing',true,true)  % 另加 Mechanics Explorer 实体模型回放
%
% 依赖：output/<tag>_simulation.mat（run_design 产物）与 Matlab/HL-0905/HL0905_2.slx。
if nargin<1||isempty(tag),tag='writing';end
if nargin<2,showFigures=true;end
if nargin<3,doPlay=false;end
cfg=NormalizeConfig(BrushConfig());c=HL0905Config();
mat=fullfile(cfg.root,'output',[tag '_simulation.mat']);
assert(isfile(mat),'缺少 %s，请先运行 run_design(''工大'')',mat);
S=load(mat);p=S.p;

mdl=prepare_model(cfg);
in=Simulink.SimulationInput(mdl);
in=in.setVariable('t',p.t).setVariable('q',p.q).setVariable('qd',p.qd).setVariable('qdd',p.qdd);
in=in.setModelParameter('StopTime',sprintf('%.12g',p.t(end)),'ReturnWorkspaceOutputs','on', ...
 'Solver','ode15s','MaxStep','0.001','RelTol','1e-7','AbsTol','1e-9');
fprintf('复跑 %s：%d 点，%.2f s\n',tag,numel(p.t),p.t(end));
simout=sim(in);

ts=simout.Px.Time;
pos=[simout.Px.Data(:) simout.Py.Data(:) simout.Pz.Data(:)];
qsim=interp1(p.t,p.q,ts);
ref=zeros(numel(ts),3);er=zeros(numel(ts),1);
for n=1:numel(ts)
 T=Fkine(c.Xi,qsim(n,:),c.g0);
 ref(n,:)=T(1:3,4)'*1000;
 D=T(1:3,1:3)*simout.R.Data(:,:,n)';
 er(n)=atan2(norm([D(3,2)-D(2,3);D(1,3)-D(3,1);D(2,1)-D(1,2)])/2,(trace(D)-1)/2)*180/pi;
end
ep=vecnorm(pos-ref,2,2);
fprintf('位置误差 %.3e mm，姿态误差 %.3e deg\n',max(ep),max(er));

f=figure('Visible','on','Position',[50 50 1200 800]);tiledlayout(2,2);
nexttile;plot(p.t,p.q*180/pi);grid on;xlabel('Time (s)');ylabel('Joint angle (deg)');
title('关节角（含抬笔/空行程）');
nexttile;plot(p.t,p.qd*180/pi);grid on;xlabel('Time (s)');ylabel('Joint velocity (deg/s)');title('关节速度');
nexttile;plot3(pos(:,1),pos(:,2),pos(:,3),'b');hold on;plot3(ref(:,1),ref(:,2),ref(:,3),'r--');
grid on;axis equal;xlabel('X (mm)');ylabel('Y (mm)');zlabel('Z (mm)');view(35,20);
legend('Simscape','POE');title('末端法兰轨迹');
nexttile;plot(ts,ep);grid on;xlabel('Time (s)');ylabel('Position error (mm)');title('POE 复算误差');
trajectory=p;
if ~showFigures, close(f); end
if doPlay
 play_writing(tag);                      % Mechanics Explorer 里回放真实机械臂
end
end
