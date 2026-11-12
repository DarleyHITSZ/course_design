function theta=Ikine6s(T,config)
% Continuation IK using the POE geometric Jacobian, nearest seeded branch.
% config.seed selects a continuous solution; no Robotics Toolbox required.
if ~isfield(config,'seed'),config.seed=pi/180*[0 30 80 0 70 0];end
theta=config.seed(:)';
for k=1:80
 [A,J]=Fkine(config.Xi,theta,config.g0);
 D=T(1:3,1:3)*A(1:3,1:3)';
 eR=0.5*[D(3,2)-D(2,3);D(1,3)-D(3,1);D(2,1)-D(1,2)];
 e=[T(1:3,4)-A(1:3,4);eR];
 if norm(e)<2e-11 && trace(D)>2.999999,break;end
 dq=J' * ((J*J'+1e-10*eye(6))\e);
 if norm(dq)>0.15,dq=dq*0.15/norm(dq);end
 theta=theta+dq';
end
A=Fkine(config.Xi,theta,config.g0);
assert(norm(A-T,'fro')<1e-7,'IK did not converge; change target/seed.');
assert(all(theta>=config.qlim(:,1)'-1e-9 & theta<=config.qlim(:,2)'+1e-9),'Joint limit exceeded.');
end
