function strokes=CharLib(chars,cfg)
% CharLib  字形库：返回指定汉字的笔画序列（字格系，单位 m）。
% 字格系：原点在字格中心，u 向右，v 向上；笔画点按弧长等距重采样。
% 每个笔画字段：name / uv(Nx2) / w(Nx1 笔迹宽度) / speed / charIndex
%
% 已实现字形：工、大。新增字形时在 switch 中追加分支即可。
strokes=struct('name',{},'uv',{},'w',{},'speed',{},'charIndex',{});
for k=1:numel(chars)
 switch chars(k)
  case '工'
   strokes(end+1)=mk('工·上横',line2([-0.024 0.022],[0.024 0.022]),0.0045, ...
     [0 .75;.08 1;.88 1;1 .55],cfg.speedDraw,k);
   strokes(end+1)=mk('工·竖',line2([0 0.022],[0 -0.018]),0.0055, ...
     [0 .80;.06 1;.92 1;1 .70],0.018,k);
   strokes(end+1)=mk('工·下横',line2([-0.030 -0.018],[0.030 -0.018]),0.0050, ...
     [0 .70;.06 1;.90 1;1 .60],cfg.speedDraw,k);
  case '大'
   % 楷书"大"：横略上仰，撇自横上方起笔、先陡后缓，捺自撇横交点下方起笔、出锋向右下
   strokes(end+1)=mk('大·横',line2([-0.027 0.014],[0.027 0.018]),0.0045, ...
     [0 .65;.05 1;.92 1;1 .55],cfg.speedDraw,k);
   strokes(end+1)=mk('大·撇',bez2([0.000 0.028],[-0.006 0.006],[-0.032 -0.028]),0.0065, ...
     [0 .72;.12 .95;.42 1;.78 .80;1 .18],0.022,k);
   strokes(end+1)=mk('大·捺',bez2([0.005 0.017],[0.017 -0.005],[0.032 -0.028]),0.0058, ...
     [0 .55;.50 .88;.78 1;.90 .92;1 .25],0.022,k);
  otherwise
   error('CharLib:unknown','CharLib 未实现字形 "%s"',chars(k));
 end
end
end

function s=mk(name,uv,wmax,knots,speed,ci)
% 按弧长等距重采样后，用节点表插值出宽度曲线
u=resample_uv(uv,0.0005);
t=linspace(0,1,size(u,1))';
s.name=name;
s.uv=u;
s.w=wmax*interp1(knots(:,1),knots(:,2),t,'linear');
s.speed=speed;
s.charIndex=ci;
end

function s=line2(A,B)
s=[A;B];
end

function s=bez2(P0,P1,P2)
t=linspace(0,1,81)';u=1-t;
s=[u.^2*P0(1)+2*u.*t*P1(1)+t.^2*P2(1), ...
   u.^2*P0(2)+2*u.*t*P1(2)+t.^2*P2(2)];
end

function out=resample_uv(uv,step)
d=[0;cumsum(sqrt(sum(diff(uv).^2,2)))];
if d(end)<step, out=uv; return; end
n=max(3,ceil(d(end)/step)+1);
sd=linspace(0,d(end),n)';
out=[interp1(d,uv(:,1),sd,'linear'),interp1(d,uv(:,2),sd,'linear')];
end
