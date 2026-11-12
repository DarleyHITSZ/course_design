function [q,qd,qdd]=TrajGena(time,MotionData,loc1,loc2)
% Evaluate synchronized profile, returning one row per requested time.
u=min(max(time(:),0),MotionData.t(end));
s=interp1(MotionData.t,MotionData.progress,u,'linear');
v=interp1(MotionData.t,MotionData.speed,u,'linear');
a=interp1(MotionData.t,MotionData.acceleration,u,'linear');
atStart=u<=0;atEnd=u>=MotionData.t(end)-10*eps(max(1,MotionData.t(end)));
s(atStart)=0;s(atEnd)=1;v(atStart|atEnd)=0;a(atStart|atEnd)=0;
d=loc2(:)'-loc1(:)';q=loc1(:)'+s*d;qd=v*d;qdd=a*d;
end
