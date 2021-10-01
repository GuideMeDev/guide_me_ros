function [m1,m2,tx]=correct_reg_trans(mpc1,mpc2,dx0,dxyedge,transx,transy)
m2=mpc2(dxyedge:end-dxyedge,dx0:end-dxyedge);s=[];
for j=1:length(transx)
for k=1:length(transy)
m1=mpc1(dxyedge+transy(k):end-dxyedge+transy(k),dx0+transx(j):end-dxyedge+transx(j));
s(k,j)=sum(sum((m2-m1).^2))/sum(sum(m2>0&m1>0));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);m1=mpc1(dxyedge+transy(fy):end-dxyedge+transy(fy),dx0+transx(fx):end-dxyedge+transx(fx));
tx=[transx(fx),transy(fy)];%yawt(2,i)=trans(fx);yawt(3,i)=transy(fy);
% S(i,2)=min(min(s));
