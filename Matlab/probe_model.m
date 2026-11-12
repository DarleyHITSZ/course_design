function probe_model
root='C:/Users/Darley/Downloads/exp4';cd(root);addpath(fullfile(root,'Matlab/HL-0905'));
diary(fullfile(root,'output/probe.log'));
try
mdl='HL0905_2';load_system(mdl);
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
p1=[mdl '/轨迹规划/Simulink-PS' newline 'Converterq'];set_param(p1,'UdotUserProvided','2');
for j=1:2
 src=[mdl '/轨迹规划/Joint1_derivative' num2str(j)];add_block('simulink/Sources/From Workspace',src,'VariableName',sprintf('[t,q%s(:,1)]',repmat('d',1,j)),'SampleTime','0');
 add_line([mdl '/轨迹规划'],['Joint1_derivative' num2str(j) '/1'],['Simulink-PS' newline 'Converterq/' num2str(j+1)],'autorouting','on');
end
b=find_system(mdl,'BlockType','ToWorkspace');for i=1:numel(b),set_param(b{i},'SaveFormat','Timeseries');end
for k=0:6
 t=[0;0.01];q=zeros(2,6); if k>0,q(:,k)=0.1;end;qd=zeros(2,6);qdd=qd;
 in=Simulink.SimulationInput(mdl);in=in.setVariable('t',t).setVariable('q',q).setVariable('qd',qd).setVariable('qdd',qdd);
 in=in.setModelParameter('StopTime','0.01','ReturnWorkspaceOutputs','on','Solver','ode15s','MaxStep','0.001');
 out=sim(in);disp(out);px=out.Px;py=out.Py;pz=out.Pz;rr=out.R;
 fprintf('PROBE %d\n',k);disp(px.Data(end));disp(py.Data(end));disp(pz.Data(end));disp(size(rr.Data));disp(rr.Data(:,:,end));
 probes{k+1}=out;
end
save(fullfile(root,'output/probes.mat'),'probes');
catch e,disp(getReport(e,'extended'));end
diary off;
end
