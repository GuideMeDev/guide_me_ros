%%
%1. find plane for the first frame
for i=1%190
% Xd=XYZ1{i+0}{1,1};i,range=1:5:length(Xd);Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
Xdr=XYZ{1,i};i,%range=1:5:length(Xd);Xdr=Xd;%Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
%Xdr=[Xdr(:,3)';-Xdr(:,1)';-Xdr(:,2)']';
% if i==15,h1(i)=1.363;eul=[1,16.5,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% h1(i)=1.30;eul(i,1:3)=[-1.289,5.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
h1(i)=1.30;eul(i,1:3)=[2.5289,26.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
% subplot(2,1,1);imshow(I1{i+0}{1,1})
subplot(2,1,1);imshow(I{1,i})
subplot(2,1,2);plot3(x1(:,1),x1(:,2),x1(:,3),'.')
plot(x1(:,2),x1(:,3),'.')
pause(1)
end
%%
%2. fit data to plane
pcloud=[];N=[];
for i=1:length(I)
% Xd=XYZ1{i+0}{1,1};i,range=1:5:length(Xd);Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
Xdr=XYZ{1,i};i,%range=1:5:length(Xd);Xdr=Xd;%[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
% if i==15,h1(i)=1.363;eul=[1,16.5,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% if i<=1,h1(i)=1.5439;eul(i,1:3)=[-2.5289,13.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
if i<=1,h1(i)=1.30;eul(i,1:3)=[2.5289,26.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% if i<=20,h1(i)=1.30;eul(i,1:3)=[-1.289,5.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% else h1(i)=h1(i-1);tetax=eul(i-1,1);tetay=eul(i-1,2);tetaz=eul(i-1,3);
else h1(i)=h1(i-1);tetax=roll(i);tetay=-(pitch(i)+pi/2);tetaz=eul(i-1,3);
% else h1(i)=h1(i-1);tetax=roll1(i);tetay=pitch1(i);tetaz=0;
end
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
x=x1;t=(abs(diff(x1(:,3))./diff(x1(:,2))));f=find(abs(x1(:,2))<1.0&abs(x1(:,1))<4&abs(x1(:,3))<0.20&[(abs(diff(x1(:,3))./diff(x1(:,2))));1]<0.22&[1;(abs(diff(x1(:,3))./diff(x1(:,2))))]<0.22);
f1=abs(x1(f,3)-prctile(x1(f,3),50))<0.05;f=f(f1);
%  hold off
% plot3(x1(:,1),x1(:,2),x1(:,3),'.')
%  pause
%  hold off
% plot(x1(:,2),x1(:,3),'.')
x=x1(f,:);
% n2=[-0.2:0.01:0.2];hz=hist(x(:,3),n2);hz=hz/sum(hz);n3=[-2:0.2:2];hy=hist(x(:,2),n3);hy=hy/sum(hy);
% if kurtosis(hz)>3,t=diff(h);tf=find(t(1:end-1)<0&t(2:end)>0);tf1=x(:,3)>n2(tf+1);tf2=x(:,3)<n2(tf+1);
% n1=polyfit(x(tf1,2),x(tf1,3),1);
n=polyfit(x(:,2),x(:,3),1);xn=x(:,2)*n(1)+n(2);f1=abs(x(:,3)-xn)<0.03;%plot(x(:,2),x(:,3),'.');hold on;plot(x(f1,2),x(f1,3),'.');
n=polyfit(x(f1,2),x(f1,3),1);xn=x(:,2)*n(1)+n(2);nx=n(1);f2=abs(x(:,3)-xn)<0.02;%plot(x((f2),2),x((f2),3),'o');hold on;plot(x(:,2),x(:,3),'.')
n=polyfit(x(f2,1),x(f2,3),1);xn=x(:,1)*n(1)+n(2);ny=n(1);f3=abs(x(:,3)-xn)<0.03;%plot(x((f3),1),x((f3),3),'o');hold on;plot(x(:,1),x(:,3),'.')
% pause
tetax=-atan(nx);tetay=atan(ny);tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R2=Rz*Ry*Rx;h1new=mean((R2*R1*Xdr(f(f3),:)')');h1new=h1new(3)+h1(i);h1(i)=h1(i)-h1new;
high=[0,0,h1(i)];x2=(R2*R1*Xdr')'+repmat(high,length(Xdr),1);
f=find(abs(x2(:,2))<1.0&abs(x2(:,1))<4&abs(x2(:,3))<0.10&[(abs(diff(x2(:,3))./diff(x2(:,2))));1]<0.22&[1;(abs(diff(x2(:,3))./diff(x2(:,2))))]<0.22);
% plot3(x2(f,1),x2(f,2),x2(f,3),'.');axis([1,3,-1,1,-0.1,0.1])

x=x2(f,:);
n=polyfit(x(:,2),x(:,3),1);xn=x(:,2)*n(1)+n(2);f1=abs(x(:,3)-xn)<0.03;%plot(x(:,2),x(:,3),'.');hold on;plot(x(f1,2),x(f1,3),'.');
n=polyfit(x(f1,2),x(f1,3),1);xn=x(:,2)*n(1)+n(2);nx=n(1);f2=abs(x(:,3)-xn)<0.02;%plot(x((f2),2),x((f2),3),'o');hold on;plot(x(:,2),x(:,3),'.')
n=polyfit(x(f2,1),x(f2,3),1);xn=x(:,1)*n(1)+n(2);ny=n(1);f3=abs(x(:,3)-xn)<0.03;%plot(x((f3),1),x((f3),3),'o');hold on;plot(x(:,1),x(:,3),'.')

tetax=-atan(nx);tetay=atan(ny);tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R3=Rz*Ry*Rx;h1new=mean((R3*R2*R1*Xdr(f(f3),:)')');h1new=h1new(3)+h1(i);h1(i)=h1(i)-h1new;
high=[0,0,h1(i)];x3=(R3*R2*R1*Xdr')'+repmat(high,length(Xdr),1);
f=find(abs(x3(:,2))<1.0&abs(x3(:,1))<3&abs(x3(:,3))<0.10&[(abs(diff(x3(:,3))./diff(x3(:,2))));1]<0.22&[1;(abs(diff(x3(:,3))./diff(x3(:,2))))]<0.22);
% plot3(x3(f,1),x3(f,2),x3(f,3),'.');axis([1,3,-1,1,-0.1,0.1])
plot3(x3(:,1),x3(:,2),x3(:,3),'.');axis([1,5,-1,1,-0.5,1]);view([280,5])

% subplot(2,1,1);plot(x2(f,2),x2(f,3),'.');axis([-1,1,-0.1,0.1]);subplot(2,1,2);plot(x2(f,1),x2(f,3),'.');axis([1,3,-0.1,0.1]);
% plot3(x2(:,1),x2(:,2),x2(:,3),'.');axis([1,3,-1,1,-0.1,0.1])%axis([-1,1,-0.1,0.1]);subplot(2,1,2);plot(x2(f,1),x2(f,3),'.');axis([1,3,-0.1,0.1]);
pause(.1)
pcloud{i}=x3;
end
%%
%3. find yaw and translation
%local map (slam) based on depth data
figure('units','normalized','outerposition',[0 0 1 1])
k=0;x=[];u=0;yawt=[];x=[];%pcloud1=pcloud;
for i=1:length(I)-2
% for i=41%210%length(I1)-2
a=rgb2gray(I{1,i});i,%a=fliplr(rgb2gray(I)')';
subplot(2,2,1),imshow(a),
sc=25;b1=pcloud{i}*1e3/sc;b2=pcloud{i+1}*1e3/sc;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
dx0=50;dxyedge=15;thzh=40;thz0=2;yaw1=yaw(i)-yaw(i+1);%+[2:0.2:5]*pi/180;dxmax=240;
[mpc1,mpc2,mpc2a,tetaz,b1a,b2a]=correct_reg_angle(b1,b2,thz0,thzh,dxmax,yaw1,0);m2a=mpc2a(dxyedge:end-dxyedge,dx0:end-dxyedge);
% m=mpc1;m(:,:,3)=mpc2;imshow(m),
k=30;m2=mpc2;m1b=mpc1(k:end-k,k:end-k);tt= xcorr2(m1b,m2);rangett=round(size(m1b)/2+size(m2)/2);tt1=tt*0;
[fy,fx]=find(tt==max(max(tt)));fx=fx(1);fy=fy(1);tx=([fy,fx]-rangett);
t1=b1a;t1(:,1)=t1(:,1)-tx(2);t1(:,2)=t1(:,2)-tx(1);

thzh=80;thz0=3;yaw1=+[-2:02:2]*pi/180;dxmax=220;
[mpc1,mpc2,tetaz1]=correct_reg_angle2(t1,b2a,thz0,thzh,dxmax,yaw1,0);
% m=mpc1;m(:,:,3)=mpc2;imshow(m),
% 
% transy=-8:1:10;transx=0:dxyedge;
% [m1,m2,tx]=correct_reg_trans(mpc1,mpc2,dx0,dxyedge,transx,transy);
% % m=m1;m(:,:,3)=m2;imshow(m),

% Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[b1a(:,1)';b1a(:,2)'])');
% t1=b1a;t1(:,1)=t1(:,1)-tx(1);t1(:,2)=t1(:,2)-tx(2);
% plot3(t1(:,1),t1(:,2),b1(:,3),'.');hold on;plot3(b2(:,1),b2(:,2),b2(:,3),'.')
% thzh=80;thz0=3;yaw1=0;+[-1:0.5:1]*pi/180;dxmax=220;
% [mpc1,mpc2,tetaz1]=correct_reg_angle2(t1,b2a,thz0,thzh,dxmax,yaw1,0);
% subplot(2,2,[1]),m=mpc1;m(:,:,3)=mpc2;imshow(m),
tetaz=tetaz+tetaz1;
yawt(i,1:3)=[tetaz,tx(2),tx(1)];
%m1=mpc1(k2:end-k2,k1:end-k2);m2=mpc2(k2:end-k2,k1:end-k2);
m1=double(mpc1&mpc2a);
m2=double(mpc2&mpc2a);m1(:,180:end)=0;m2(:,180:end)=0;
m=m1;m(:,:,3)=m2;%subplot(2,2,[2,4]),imshow(double(m))
subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),%title(num2str(S(i,:)))
% pause
t=((m1.*m2)>0);t2=mpc2*0;t2=t;%t2(dxyedge:end-dxyedge,dx0:end-dxyedge)=t;
c=conv2(t,ones(4,4)/16,'same');c=(c>0.6);t2=c;%t2=mpc2*0;t2(k2:end-k2,k1:end-k2)=c;
[py,px]=find(t2>0);px=px;py=py-200;px1=px+yawt(i,2);py1=py+yawt(i,3);
tetaz=-yawt(i,1);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=(Rz*[px1';py1'])';
x=[x;t1];tetaz=yawt(i,1);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x=[(Rz*[x]')'];
x(:,1)=x(:,1)-yawt(i,2);x(:,2)=x(:,2)-yawt(i,3);%t2=mpc2*0;t1(10-yawt(3,i):end-10-yawt(3,i),k1-yawt(2,i):end-10-yawt(2,i))=t;t2(10:end-10,k1:end-10)=t;
x=[x;[px';py']'];tetaz=90*pi/180;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x1=[(Rz*[x]')'];
subplot(2,2,[2,4]),plot(x1(:,1),x1(:,2),'.'),axis([-120,120,-80,300])
hold on
plot([-300,100,300,-300,-300]/sc,[0,0,2000,2000,0]/sc)
hold off

pause(.1)
end
%%
% Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=((Rz*[b1(:,1)';b1(:,2)'])');t1(:,1)=t1(:,1)-tx(1);t1(:,2)=t1(:,2)-tx(2);t1(:,3)=b1(:,3);
subplot(2,2,[2,4]),
hold off
s=sqrt(sum(sum((mpc2-mpc1).^2)))/sum(sum(mpc1~=0|mpc2~=0));
% plot3(t1(:,1),t1(:,2),t1(:,3),'.');hold on;plot3(b2(:,1),b2(:,2),b2(:,3),'.');view(220,35),axis([0,dxmax,-150,150,-0.5,thzh]/1),title(num2cell(s))
b2a=round(b2(b2(:,3)>6&b2(:,1)>dx0,:));px2=b2a(:,1);py2=b2a(:,2)+200;mpc2a=zeros(400,300);mpc2a((px2-1).*size(mpc2a,1)+py2)=1;
b1a=round(t1(t1(:,3)>6&t1(:,1)>dx0,:));px=b1a(:,1);py=b1a(:,2)+200;mpc1a=zeros(400,300);mpc1a((px-1).*size(mpc1a,1)+py)=1;
m=mpc1a;m(:,:,3)=mpc2a;imshow(m),

% pause
% end
%%


Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[b1(:,1)';b1(:,2)'])');t1(:,1)=t1(:,1)-tx(1);t1(:,2)=t1(:,2)-tx(2);
plot3(t1(:,1),t1(:,2),b1(:,3),'.');hold on;plot3(b2(:,1),b2(:,2),b2(:,3),'.')
% yawt(1,i)=yaw1(f);

plot3(x(:,1),x(:,2),x(:,3),'.'),view(280,5),axis([0,4,-2,2,-0.5,1]/1),


f=find(abs(b2(:,3))>5&b2(:,1)>0&b2(:,1)>0&b2(:,1)<280);b2=round(b2(f,:));px2=b2(:,1);py2=b2(:,2)+200;pz2=round(b2(:,3)*1)+0;pz2(pz2>k4)=0;pz2(pz2<0)=-1;pz2(pz2>3)=2;%pz2(pz2>k3)=k3;%pz=round(b1(:,3)*2);
mpc2=zeros(400,300);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find((b2(:,3))>5&b2(:,1)>0&b2(:,1)>0&b2(:,1)<260);b2=round(b2(f,:));px2a=b2(:,1);py2a=b2(:,2)+200;pz2a=round(b2(:,3)*1)+0;pz2a(pz2a>k4)=0;pz2a(pz2a>k3)=k3;%pz=round(b1(:,3)*2);
mpc2a=zeros(400,300);mpc2a((px2a-1).*size(mpc2a,1)+py2a)=pz2a;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find(abs(b1(:,3))>5&b1(:,1)>0&b1(:,1)>0&b1(:,1)<280);b1=round(b1(f,:));px=b1(:,1);py=b1(:,2)+0;pz=round(b1(:,3)*1)+0;pz(pz>k4)=0;pz(pz<0)=-1;pz(pz>3)=2;%pz(pz>k4)=0;pz(pz>k3)=k3;
% mpc1a=zeros(400,300);mpc2a=zeros(400,300);for i=8:k3;mpc1=zeros(400,300);f=(pz==i);mpc1((px(f)-1).*size(mpc1,1)+py(f))=1;mpc1a=mpc1a+mpc1;mpc2=zeros(400,300);f=pz2==i;mpc2((px2(f)-1).*size(mpc2,1)+py2(f))=1;mpc2a=mpc2a+mpc2;end
% [py,px]=find(mpc1a>0);py=py-200;pz=mpc1a(mpc1a>0);[py2,px2]=find(mpc2a>0);pz2=mpc2a(mpc2a>0);mpc2=mpc2a;
% s=[];yaw1=yaw11(i+0)+[-1:0.5:1]*pi/180;for j=1:length(yaw1)
s=[];yaw1=[-15:0.2:10]*pi/180;for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2))/sum(sum(mpc2>0&mpc1>0));
end
f=find(s==min(s));S(i,1)=min(s);f=f(1);yawt(1,i)=yaw1(f);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;
% plot(s)
% m=mpc1;m(:,:,3)=mpc2;imshow(m),pause(1)%imshow(mpc1)
k2=15;k1=50;transy=-8:1:10;trans=0:k2;m2=mpc2(k2:end-k2,k1:end-k2);s=[];
for j=1:length(trans)
for k=1:length(transy)
m1=mpc1(k2+transy(k):end-k2+transy(k),k1+trans(j):end-k2+trans(j));s(k,j)=sum(sum((m2-m1).^2))/sum(sum(m2>0&m1>0));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));S(i,2)=min(min(s));fx=fx(1);fy=fy(1);
m1=mpc1(k2+transy(fy):end-k2+transy(fy),k1+trans(fx):end-k2+trans(fx));
