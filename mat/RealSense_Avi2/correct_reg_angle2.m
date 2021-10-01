function [mpc1,mpc2,tetaz,b1a,b2a]=correct_reg_angle2(b1,b2,thz0,thzh,dxmax,yaw1,show)
px2=b2(:,1);py2=b2(:,2)+200;pz2=b2(:,3);mpc2=zeros(400,300);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;
px=b1(:,1);py=b1(:,2);pz=b1(:,3); 
s=[];b2a=[px2';py2';pz2']';%  
for j=1:length(yaw1)  
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%imshow(mpc1)
% s(j)=sum(sum((mpc2-mpc1).^2))/sum(sum(mpc2>0&mpc1>0));
s(j)=sum(sum((mpc2.*mpc1).^4))/sum(sum(mpc2&mpc1)); 
end   
f=find(s==max(s));f=f(1);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;b1a=[px1';py1';pz']';
t1=round((Rz*[b1(:,1)';b1(:,2)'])');
if show==1
plot3(t1(:,1),t1(:,2),b1(:,3),'.');hold on;plot3(b2(:,1),b2(:,2),b2(:,3),'.')
end
% yawt(1,i)=yaw1(f);
