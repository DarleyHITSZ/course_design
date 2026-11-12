function plot_writing(p,plan,cfg,out,tag)
% plot_writing  生成墨迹预览与轨迹总览图（白底，可直接放进报告/PPT）。
%   <tag>_preview.png  墨迹位图（按压入深度渲染宽度）
%   <tag>_path.png     六联图：俯视 / 3D / 关节角 / 关节速度 / 关节加速度 / 提按与速度
[img,xr,yr]=render_writing(plan,cfg);
imwrite(uint8(~img)*255,fullfile(out,[tag '_preview.png']));

draw=strcmp(p.segName,'绘制');
X=p.target*1000;
f=figure('Color','w','Position',[40 40 1450 1150],'Name',[tag ' 轨迹总览'],'NumberTitle','off');
tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
joints={'q1','q2','q3','q4','q5','q6'};

% 俯视
ax=nexttile;hold(ax,'on');
plot(ax,X(~draw,1),X(~draw,2),'-','Color',[0.72 0.72 0.72],'LineWidth',0.6);
scatter(ax,X(draw,1),X(draw,2),8,p.depth(draw)*1000,'filled');
axis(ax,'equal');grid(ax,'on');box(ax,'on');
xlabel(ax,'X (mm)');ylabel(ax,'Y (mm)');
title(ax,'笔尖俯视轨迹（灰=抬落笔/空行程，彩色=绘制）');
cb=colorbar(ax);cb.Color='k';cb.Label.String='压入深度 (mm)';cb.Label.Color='k';
lightax(ax);

% 3D
ax=nexttile;hold(ax,'on');
plot3(ax,X(~draw,1),X(~draw,2),X(~draw,3),'-','Color',[0.72 0.72 0.72],'LineWidth',0.6);
scatter3(ax,X(draw,1),X(draw,2),X(draw,3),8,p.depth(draw)*1000,'filled');
patch(ax,[xr(1) xr(2) xr(2) xr(1)]*1000,[yr(1) yr(1) yr(2) yr(2)]*1000, ...
      ones(1,4)*cfg.paperZ*1000,[0.96 0.94 0.90],'EdgeColor',[0.6 0.6 0.6]);
grid(ax,'on');box(ax,'on');
xlabel(ax,'X (mm)');ylabel(ax,'Y (mm)');zlabel(ax,'Z (mm)');view(ax,-30,18);
title(ax,'笔尖 3D 轨迹与纸面');
lightax(ax);

% 关节角
ax=nexttile;
plot(ax,p.t,p.q*180/pi,'LineWidth',0.8);grid(ax,'on');box(ax,'on');
xlabel(ax,'Time (s)');ylabel(ax,'关节角 (deg)');
title(ax,'关节角（含抬笔/空行程）');
lg=legend(ax,joints,'Location','northeast','FontSize',7);
lg.TextColor='k';lg.Color='w';lg.EdgeColor=[0.6 0.6 0.6];
lightax(ax);

% 关节速度（颜色顺序同关节角）
ax=nexttile;
plot(ax,p.t,p.qd*180/pi,'LineWidth',0.8);grid(ax,'on');box(ax,'on');
xlabel(ax,'Time (s)');ylabel(ax,'关节角速度 (deg/s)');
title(ax,sprintf('关节角速度（max %.1f deg/s，限值 %.0f deg/s）', ...
      max(abs(p.qd),[],'all')*180/pi,cfg.qdMax*180/pi));
lightax(ax);

% 关节加速度（颜色顺序同关节角）
ax=nexttile;
plot(ax,p.t,p.qdd*180/pi,'LineWidth',0.8);grid(ax,'on');box(ax,'on');
xlabel(ax,'Time (s)');ylabel(ax,'关节角加速度 (deg/s^2)');
title(ax,sprintf('关节角加速度（max %.1f deg/s^2，限值 %.0f deg/s^2）', ...
      max(abs(p.qdd),[],'all')*180/pi,cfg.qddMax*180/pi));
lightax(ax);

% 提按与速度
ax=nexttile;
yyaxis(ax,'left');plot(ax,p.t,p.depth*1000,'LineWidth',0.9);ylabel(ax,'压入深度 (mm)');
ylim(ax,[0 1.5]);
yyaxis(ax,'right');plot(ax,p.t,p.speed*1000,'LineWidth',0.9);ylabel(ax,'速度 (mm/s)');
grid(ax,'on');box(ax,'on');xlabel(ax,'Time (s)');title(ax,'提按与速度');
ax.Color='w';ax.XColor='k';ax.GridColor=[0.15 0.15 0.15];ax.GridAlpha=0.15;
ax.Title.Color='k';ax.XLabel.Color='k';
ax.YAxis(1).Color=[0 0.447 0.741];ax.YAxis(2).Color=[0.85 0.325 0.098];

exportgraphics(f,fullfile(out,[tag '_path.png']),'Resolution',160,'BackgroundColor','white');
% 图窗保留在桌面上供查看（不自动关闭）
end

function lightax(ax)
% 深色主题下强制白底黑字（仅对单 Y 轴）
ax.Color='w';ax.XColor='k';ax.YColor='k';ax.GridColor=[0.15 0.15 0.15];ax.GridAlpha=0.15;
ax.Title.Color='k';ax.XLabel.Color='k';ax.YLabel.Color='k';
if isprop(ax,'ZColor'),ax.ZColor='k';end
if isprop(ax,'ZLabel'),ax.ZLabel.Color='k';end
end
