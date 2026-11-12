function [T,J] = Fkine(Xi,theta,g0)
% Space POE: Xi=[v;omega], translation in m, angles in rad.
T=eye(4);Js=zeros(6,6);
for i=1:numel(theta)
 w=Xi(4:6,i);v=Xi(1:3,i);W=skew(w);
 R=T(1:3,1:3);p=T(1:3,4);
 Js(:,i)=[R*v+cross(p,R*w);R*w];
 a=theta(i);E=eye(4);
 if norm(w)<1e-12,E(1:3,4)=v*a;
 else
 E(1:3,1:3)=eye(3)+sin(a)*W+(1-cos(a))*W*W;
 E(1:3,4)=(eye(3)*a+(1-cos(a))*W+(a-sin(a))*W*W)*v;
 end
 T=T*E;
end
T=T*g0;
J=[Js(1:3,:)-skew(T(1:3,4))*Js(4:6,:);Js(4:6,:)];
end
function W=skew(w)
W=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
end
