function [mpc1,mpc2,mpc2a,tetaz,b1a,b2a]=correct_reg_angle(b1,b2,thz0,thzh,dxmax,yaw1,show)
f=find(abs(b2(:,3))>thz0&b2(:,1)>40&b2(:,1)<dxmax);b2=round(b2(f,:));px2=b2(:,1);py2=b2(:,2)+200;pz2=round(b2(:,3)*1)+0;pz2(pz2>thzh)=0;pz2(pz2<0)=-1;pz2(pz2>3)=2;%pz2(pz2>k3)=k3;%pz=round(b1(:,3)*2);
mpc2=zeros(400,300);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find((b2(:,3))>3&b2(:,1)>40&b2(:,1)>0&b2(:,1)<dxmax);b2a=round(b2(f,:));px2a=b2a(:,1);py2a=b2a(:,2)+200;
mpc2a=zeros(400,300);mpc2a((px2a-1).*size(mpc2a,1)+py2a)=1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find(abs(b1(:,3))>thz0&b1(:,1)>40&b1(:,1)<dxmax);b1=round(b1(f,:));px=b1(:,1);py=b1(:,2)+0;pz=round(b1(:,3)*1)+0;pz(pz>thzh)=0;pz(pz<0)=-1;pz(pz>3)=2;%pz(pz>k4)=0;pz(pz>k3)=k3;
% for i=8:k3;mpc1=zeros(400,300);f=(pz==i);mpc1((px(f)-1).*size(mpc1,1)+py(f))=1;mpc1a=mpc1a+mpc1;mpc2=zeros(400,300);f=pz2==i;mpc2((px2(f)-1).*size(mpc2,1)+py2(f))=1;mpc2a=mpc2a+mpc2;end
% [py,px]=find(mpc1a>0);py=py-200;pz=mpc1a(mpc1a>0);[py2,px2]=find(mpc2a>0);pz2=mpc2a(mpc2a>0);mpc2=mpc2a;
% s=[];yaw1=yaw11(i+0)+[-1:0.5:1]*pi/180;for j=1:length(yaw1)
s=[];b2a=[px2';py2'-200;pz2']';% 
for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%imshow(mpc1)
% s(j)=sum(sum((mpc2-mpc1).^2))/sum(sum(mpc2>0&mpc1>0)); 
s(j)=sum(sum((mpc2.*mpc1).^4))/sum(sum(mpc2&mpc1)); 
end 
% plot(s)

f=find(s==max(s));f=f(1);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;b1a=[px1';py1'-200;pz']';
t1=round((Rz*[b1(:,1)';b1(:,2)'])');
if show==1
plot3(t1(:,1),t1(:,2),b1(:,3),'.');hold on;plot3(b2(:,1),b2(:,2),b2(:,3),'.')
end
% yawt(1,i)=yaw1(f);
