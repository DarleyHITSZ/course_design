function mdl=prepare_model(cfg,visualize)
% prepare_model  生成可注入 qd/qdd 的仿真模型 output/HL0905_2_writing.slx。
% 修补内容（与原实验一致）：
%   1) 关节 1 的 Simulink-PS Converter 允许外部提供一/二阶导数
%   2) 补 Joint1_derivative1/2 两个 From Workspace 块并连到转换器第 2/3 输入
%   3) 所有 To Workspace 块改为 Timeseries 输出
%   4) 7 个 Solid 块的几何文件改成工作区内的绝对路径（否则模型存到 output/ 后
%      Mechanics Explorer 按模型目录找 STL 会找不到；只影响显示，不影响动力学）
%
%   prepare_model(cfg)            % 批处理用：不打开 Mechanics Explorer
%   prepare_model(cfg,true)       % 回放用：打开 Mechanics Explorer 显示实体臂
if nargin<2||isempty(visualize), visualize=false; end
addpath(fullfile(cfg.root,'Matlab','HL-0905'));
src=fullfile(cfg.root,'Matlab','HL-0905','HL0905_2.slx');
dst=fullfile(cfg.root,'output','HL0905_2_writing.slx');
if bdIsLoaded('HL0905_2'), close_system('HL0905_2',0); end
if bdIsLoaded('HL0905_2_writing'), close_system('HL0905_2_writing',0); end
load_system(src); mdl='HL0905_2';
if visualize, set_param(mdl,'SimMechanicsOpenEditorOnUpdate','on');
else, set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off'); end

p=[mdl '/轨迹规划/Simulink-PS' newline 'Converterq'];
set_param(p,'UdotUserProvided','2');
for j=1:2
 name=['Joint1_derivative' num2str(j)];
 srcb=[mdl '/轨迹规划/' name];
 if getSimulinkBlockHandle(srcb)<=0
  add_block('simulink/Sources/From Workspace',srcb, ...
    'VariableName',sprintf('[t,q%s(:,1)]',repmat('d',1,j)), ...
    'SampleTime','0','Position',[330 210+j*45 430 230+j*45]);
 end
 add_line([mdl '/轨迹规划'],[name '/1'],['Simulink-PS' newline 'Converterq/' num2str(j+1)],'autorouting','on');
end

b=find_system(mdl,'BlockType','ToWorkspace');
for i=1:numel(b), set_param(b{i},'SaveFormat','Timeseries'); end

% 几何文件绝对路径：Solid 块的 ExtGeomFileName 存的是裸文件名，
% 模型另存到 output/ 后 Simscape 按模型所在目录解析，Mechanics Explorer 会读不到 STL。
all=find_system(mdl,'LookUnderMasks','all','FollowLinks','on');
geodir=fullfile(cfg.root,'Matlab','HL-0905');
ng=0;
for i=1:numel(all)
 try, name=get_param(all{i},'ExtGeomFileName'); catch, continue; end
 if isempty(name), continue; end
 if isfile(name), continue; end                       % 已经是可用绝对路径
 f=fullfile(geodir,name);
 if isfile(f)
  set_param(all{i},'ExtGeomFileName',f); ng=ng+1;
 end
end
fprintf('prepare_model：几何路径修正 %d 个 Solid 块，可视化 %s\n',ng,string(visualize));

save_system(mdl,dst);
mdl='HL0905_2_writing';
end
