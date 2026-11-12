function [img,xr,yr]=render_writing(plan,cfg,scale)
% render_writing  由笔尖轨迹与压入深度渲染"墨迹"位图（俯视图）。
% 宽度模型：接触半径 a = sqrt(2*R*d)，与 BrushPlanner 的 d = w^2/(8R) 互逆。
% 返回 img（H x W，0/1）与坐标范围 [xmin xmax]、[ymin ymax]（m）。
if nargin<3, scale=2500; end          % 像素/米（0.4 mm/px）
pad=0.008;
xr=[min(plan.tip(:,1))-pad, max(plan.tip(:,1))+pad];
yr=[min(plan.tip(:,2))-pad, max(plan.tip(:,2))+pad];
W=ceil((xr(2)-xr(1))*scale)+1; H=ceil((yr(2)-yr(1))*scale)+1;
img=false(H,W);
R=cfg.brushRadius;
for i=1:size(plan.tip,1)
 d=plan.depth(i);
 if d<=0, continue; end
 rw=max(0.0002,sqrt(2*R*d));        % 接触半径（m）
 cx=(plan.tip(i,1)-xr(1))*scale+1;
 cy=(yr(2)-plan.tip(i,2))*scale+1;  % 图像行向下，v 轴向上
 rp=max(1,rw*scale);
 x0=max(1,floor(cx-rp)); x1=min(W,ceil(cx+rp));
 y0=max(1,floor(cy-rp)); y1=min(H,ceil(cy+rp));
 if x1<x0||y1<y0, continue; end
 [X,Y]=meshgrid(x0:x1,y0:y1);
 m=(X-cx).^2+(Y-cy).^2<=rp^2;
 img(y0:y1,x0:x1)=img(y0:y1,x0:x1)|m;
end
end
