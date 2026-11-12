function m=MotionPlanning(TotalLength,Vmax,Accel,Decel,Ts)
% One normalized trapezoid synchronizes every moving axis.
% m.s/v/a: DOF x samples. Stage durations rounded UP to Ts.
L=abs(TotalLength(:));n=numel(L);
Vmax=expand(Vmax,n);Accel=expand(Accel,n);Decel=expand(Decel,n);
assert(Ts>0 && all(Vmax>0) && all(Accel>0) && all(Decel>0));
on=L>1e-14;
if ~any(on)
 m.t=[0;Ts];m.s=zeros(n,2);m.v=m.s;m.a=m.s;m.maxtime=Ts;m.progress=[0;0];m.speed=[0;0];m.acceleration=[0;0];m.stages=[0 Ts 0];m.Ts=Ts;return
end
v=min(Vmax(on)./L(on));ac=min(Accel(on)./L(on));dc=min(Decel(on)./L(on));
vp=min(v,sqrt(2/(1/ac+1/dc)));
ta=ceil((vp/ac)/Ts-1e-10)*Ts;td=ceil((vp/dc)/Ts-1e-10)*Ts;
tc=max(0,ceil(((1-0.5*vp*(vp/ac+vp/dc))/vp)/Ts-1e-10)*Ts);
vp=1/(tc+0.5*(ta+td));T=ta+tc+td;t=(0:round(T/Ts))'*Ts;
s=zeros(size(t));vel=s;acc=s;
i=t<ta;s(i)=0.5*vp/ta*t(i).^2;vel(i)=vp/ta*t(i);acc(i)=vp/ta;
i=t>=ta & t<ta+tc;s(i)=0.5*vp*ta+vp*(t(i)-ta);vel(i)=vp;
i=t>=ta+tc & t<T-1e-10;u=T-t(i);s(i)=1-0.5*vp/td*u.^2;vel(i)=vp/td*u;acc(i)=-vp/td;
s(end)=1;vel([1 end])=0;acc([1 end])=0;
m.t=t;m.s=L*s';m.v=L*vel';m.a=L*acc';m.maxtime=T;
m.progress=s;m.speed=vel;m.acceleration=acc;m.stages=[ta tc td];m.Ts=Ts;
end
function x=expand(x,n)
if isscalar(x),x=repmat(x,n,1);else,x=x(:);end
assert(numel(x)==n);
end
