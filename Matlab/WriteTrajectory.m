function p=WriteTrajectory(plan,c,cfg)
% WriteTrajectory  笔尖世界轨迹 -> 关节轨迹（逐点 IK + 雅可比反解关节速度）。
% 笔尖位于法兰系 [0;0;L]，故 IK 目标为 flange = tip - L*Rz。
% 输出 p 与原实验一致的字段：t/q/qd/qdd/target/rotation/name。
R=cfg.Rtool; L=cfg.brushLength; Ts=cfg.Ts;
N=size(plan.tip,1);
raz=L*R(:,3);                      % 法兰->笔尖的世界向量（姿态固定，常量）
q=zeros(N,6); qd=zeros(N,6); v=zeros(N,3);
v(2:N-1,:)=(plan.tip(3:N,:)-plan.tip(1:N-2,:))/(2*Ts);

seed=cfg.q0;
for i=1:N
 Tf=[R plan.tip(i,:)'-raz;0 0 0 1];
 c.seed=seed; q(i,:)=Ikine6s(Tf,c);
 seed=q(i,:);
 if mod(i,5000)==0, fprintf('  IK %5d/%d\n',i,N); end
end

% 关节速度：笔尖线速度 + 零角速度 → 6x6 任务雅可比反解
for i=1:N
 [~,J]=Fkine(c.Xi,q(i,:)',c.g0);
 Jtip=[J(1:3,:)-skew(raz)*J(4:6,:);J(4:6,:)];
 qd(i,:)=(Jtip\[v(i,:)';0;0;0])';
end
qdd=zeros(N,6);
for j=1:6, qdd(:,j)=gradient(qd(:,j),Ts); end   % 按行（时间）求导，勿直接用 gradient(qd,Ts)
qdd(1,:)=0; qdd(end,:)=0;

p.t=(0:N-1)'*Ts;
p.q=q; p.qd=qd; p.qdd=qdd;
p.target=plan.tip; p.rotation=R;
p.depth=plan.depth; p.speed=plan.speed;
p.segName=plan.segName; p.stroke=plan.stroke;
p.name='writing';
p.brushLength=L; p.paperZ=cfg.paperZ;
end

function S=skew(v)
S=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end
