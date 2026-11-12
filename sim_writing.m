% sim_writing  一键生成"工大"书写轨迹（规划 -> 仿真校验 -> 导出）。
% 直接在 MATLAB 中运行本脚本，或在命令行执行：
%   D:\MATLAB\R2026b\bin\matlab.exe -batch "run('sim_writing.m')"
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'Matlab'));

% 可选参数：仿真结束后是否打开 Mechanics Explorer 回放实体机械臂
%   true  = 跑完自动回放（需要在 MATLAB 桌面里运行，-batch 下无窗口）
%   false = 只做规划/校验/导出
doPlay=false;

run_design('工大',struct('play',doPlay));
