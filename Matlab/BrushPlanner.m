function plan=BrushPlanner(strokes,cfg,startTip)
% BrushPlanner  笔画序列 -> 笔尖世界轨迹（含落笔/提笔/空行程与提按深度）。
% 输入：strokes（CharLib 输出）、cfg（BrushConfig）、startTip（1x3 起始笔尖位置）
% 输出 plan 字段：
%   tip      笔尖世界坐标 N x 3 (m)
%   speed    沿路径速度 N x 1 (m/s)
%   depth    压入深度 N x 1 (m)
%   segName  所属段名 cell（抬笔/空行程/落笔/绘制/提笔）
%   stroke   所属笔画名 cell，charIndex 所属字序号
nchar=max([strokes.charIndex]);
off=((1:nchar)-(nchar+1)/2)*(cfg.cell+cfg.cellGap);

tip=zeros(0,3); spd=zeros(0,1); segName={}; strokeName={}; ci=zeros(0,1);
cur=startTip(:)'; first=true;
for k=1:numel(strokes)
 st=strokes(k);
 u=st.uv(:,1)+off(st.charIndex);
 dep=min(cfg.maxPress,st.w.^2/(8*cfg.brushRadius));
 P=[cfg.paperX+u, cfg.paperY+st.uv(:,2), cfg.paperZ-dep];
 A=[P(1,1) P(1,2) cfg.paperZ+cfg.lift];

 if first   % 起始点先竖直抬到安全高度
  U=[cur(1:2) cfg.paperZ+cfg.lift];
  [q,sv]=samplePath([cur;U],cfg.speedPress,cfg.accDraw,cfg.Ts);
  [tip,spd,segName,strokeName,ci]=appendSeg(tip,spd,segName,strokeName,ci,q,sv,'抬笔','',0);
  cur=U; first=false;
 end

 if norm(A-cur)>1e-9   % 空行程
  [q,sv]=samplePath([cur;A],cfg.speedMove,cfg.accMove,cfg.Ts);
  [tip,spd,segName,strokeName,ci]=appendSeg(tip,spd,segName,strokeName,ci,q,sv,'空行程','',st.charIndex);
  cur=A;
 end

 [q,sv]=samplePath([A;P(1,:)],cfg.speedPress,cfg.accDraw,cfg.Ts);      % 落笔
 [tip,spd,segName,strokeName,ci]=appendSeg(tip,spd,segName,strokeName,ci,q,sv,'落笔',st.name,st.charIndex);

 [q,sv]=samplePath(P,st.speed,cfg.accDraw,cfg.Ts);                     % 绘制
 [tip,spd,segName,strokeName,ci]=appendSeg(tip,spd,segName,strokeName,ci,q,sv,'绘制',st.name,st.charIndex);

 cur=[P(end,1) P(end,2) cfg.paperZ+cfg.lift];
 [q,sv]=samplePath([P(end,:);cur],cfg.speedPress,cfg.accDraw,cfg.Ts);  % 提笔
 [tip,spd,segName,strokeName,ci]=appendSeg(tip,spd,segName,strokeName,ci,q,sv,'提笔',st.name,st.charIndex);
end

plan.tip=tip;
plan.speed=spd;
plan.depth=max(0,cfg.paperZ-tip(:,3));
plan.segName=segName;
plan.stroke=strokeName;
plan.charIndex=ci;
plan.Ts=cfg.Ts;
plan.startTip=startTip(:)';
end

function [pos,spd]=samplePath(P,vmax,acc,Ts)
% 沿折线按梯形速度规划采样；返回 Ts 等间隔的笔尖位置与实际速度
pc=[0;cumsum(sqrt(sum(diff(P).^2,2)))];
L=pc(end);                       % 用累加值作总长，保证 s<=pc(end)（否则 interp1 端点出 NaN）
if L<1e-9
 pos=P(1,:); spd=0; return
end
m=MotionPlanning(L,vmax,acc,acc,Ts);
s=min(max(m.progress,0),1)*L;
pos=[interp1(pc,P(:,1),s,'linear'),interp1(pc,P(:,2),s,'linear'),interp1(pc,P(:,3),s,'linear')];
spd=m.speed*L;
if any(~isfinite(pos),'all')
 error('BrushPlanner:badSegment','段采样出现非有限值：L=%.6g vmax=%.4g acc=%.4g stages=%s n=%d', ...
   L,vmax,acc,mat2str(m.stages),size(pos,1));
end
end

function [tip,spd,segName,strokeName,ci]=appendSeg(tip,spd,segName,strokeName,ci,pos,sv,name,stroke,ch)
if isempty(tip)
 keep=true(size(pos,1),1);
else
 keep=[false;true(size(pos,1)-1,1)];   % 去掉与上一段末点重复的首点
end
if ~any(keep), return; end
n=sum(keep);
tip=[tip;pos(keep,:)]; spd=[spd;sv(keep)];
segName=[segName;repmat({name},n,1)];
strokeName=[strokeName;repmat({stroke},n,1)];
ci=[ci;repmat(ch,n,1)];
end
