load('euler.mat');
a=cell2table(pitch);pitch1=a.pitch;
a=cell2table(roll);roll1=a.roll;
a=cell2table(yaw);yaw1=a.yaw;
eul0=[roll1';pitch1';yaw1']';
lengthI=whos('-file','I.mat');
n0=1:3:lengthI.size(1)-400;
% for i=1:length(n1),I1{i}=I{n1(i)};end
% whos('-file','I.mat')
m1 = matfile('I.mat');
m2 = matfile('XYZ.mat');
for i=1:length(n0),I1{i}=m1.I(n0(i),:);XYZ1{i}=m2.XYZ(n0(i),:);end%imshow(a{1})
%%
plot(eul0*180/pi)
%%
%plane detection 
h1=[];eul=[];n1=round([1:length(eul0)/lengthI.size(1):length(eul0)]);n1=n1(1:3:end);
roll1=eul0(n1,1)-0*pi/180;pitch1=eul0(n1,2);yaw11=-(diff(eul0(n1,3)));plot(yaw11*180/pi)
%%
%find plane for the first frame
for i=20%190
Xd=XYZ1{i+0}{1,1};i,range=1:5:length(Xd);Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
% Xd=XYZ{1,i};i,range=1:5:length(Xd);Xdr=Xd;%Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
%Xdr=[Xdr(:,3)';-Xdr(:,1)';-Xdr(:,2)']';
% if i==15,h1(i)=1.363;eul=[1,16.5,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% h1(i)=1.30;eul(i,1:3)=[-1.289,5.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
h1(i)=1.10;eul(i,1:3)=[2.5289,16.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
subplot(2,1,1);imshow(I1{i+0}{1,1})
% subplot(2,1,1);imshow(I{1,i})
subplot(2,1,2);plot3(x1(:,1),x1(:,2),x1(:,3),'.')
pause(1)
end
%%
pcloud=[];N=[];
for i=20:length(I1)-1
Xd=XYZ1{i+0}{1,1};i,range=1:5:length(Xd);Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
% Xd=XYZ{1,i};i,range=1:5:length(Xd);Xdr=Xd;%[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
% if i==15,h1(i)=1.363;eul=[1,16.5,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% if i<=1,h1(i)=1.5439;eul(i,1:3)=[-2.5289,13.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
if i<=20,h1(i)=1.10;eul(i,1:3)=[2.5289,16.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% if i<=20,h1(i)=1.30;eul(i,1:3)=[-1.289,5.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
else h1(i)=h1(i-1);tetax=eul(i-1,1);tetay=eul(i-1,2);tetaz=eul(i-1,3);
% else h1(i)=h1(i-1);tetax=roll1(i);tetay=pitch1(i);tetaz=0;
end
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
% plot3(x1(:,1),x1(:,2),x1(:,3),'.')
%     pause
x=x1;t=(abs(diff(x1(:,3))./diff(x1(:,2))));f=find(abs(x1(:,2))<1.0&abs(x1(:,1))<4&abs(x1(:,3))<0.20&[(abs(diff(x1(:,3))./diff(x1(:,2))));1]<0.15&[1;(abs(diff(x1(:,3))./diff(x1(:,2))))]<0.15);
f1=abs(x1(f,3)-prctile(x1(f,3),50))<0.05;f=f(f1);
% plot(x1(f,2),x1(f,3),'.')
x=x1(f,:);
% n2=[-0.2:0.01:0.2];hz=hist(x(:,3),n2);hz=hz/sum(hz);n3=[-2:0.2:2];hy=hist(x(:,2),n3);hy=hy/sum(hy);
% if kurtosis(hz)>3,t=diff(h);tf=find(t(1:end-1)<0&t(2:end)>0);tf1=x(:,3)>n2(tf+1);tf2=x(:,3)<n2(tf+1);
% n1=polyfit(x(tf1,2),x(tf1,3),1);
n=polyfit(x(:,2),x(:,3),1);xn=x(:,2)*n(1)+n(2);f1=abs(x(:,3)-xn)<0.03;%plot(x(f1,2),x(f1,3),'.')
n=polyfit(x(f1,2),x(f1,3),1);xn=x(:,2)*n(1)+n(2);f2=abs(x(:,3)-xn)<0.02;%plot(x((f2),2),x((f2),3),'o');hold on;plot(x(:,2),x(:,3),'.')
n=polyfit(x(f2,1),x(f2,3),1);xn=x(:,1)*n(1)+n(2);f3=abs(x(:,3)-xn)<0.03;%
% plot(x((f3),1),x((f3),3),'o');hold on;plot(x(:,1),x(:,3),'.')
% pause

% n=polyfit(x(f2,2),x(f2,3),1);xn=x(:,2)*n(1)+n(2);f3=abs(x(:,3)-xn)<0.01;plot(x((f3),2),x((f3),3),'o');hold on;plot(x(:,2),x(:,3),'.')
%     tf=tf1;if abs(mean(x(tf1,2)))>abs(mean(x(tf2,2))),tf=tf2;end,end
%     plot(h);hold on;plot(tf+1,h(tf+1),'o');end
f=f(f3);

if length(f)<400,
    pause
h1(i)=h1(i-1);eul(i,:)=[roll1(i),pitch1(i),0];pcloud1{i}=x;
else
x=x1;x11=sum(x(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(f,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R2=Rz*Ry*Rx;x2=(R2*R1*Xdr')'+repmat(high,length(Xdr),1);h1(i)=h1(i)-mean(x2(f,3));x2(:,3)=x2(:,3)-mean(x2(f,3));
% ;h1(i)=h1(i)-prctile(x2(abs(x2(:,3))<0.1,3),50);R=R2*R1;%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
%plot(x(:,2),x(:,3),'.');hold on;plot(x(f,2),x(f,3),'.')
% pause
%   plot3(x2(:,1),x2(:,2),x2(:,3),'.')
%  pause
f=find(abs(x2(:,2))<1.0&(x2(:,1))<4&abs(x2(:,3))<0.05&[1;(abs(diff(x2(:,3))./diff(x2(:,2))))]<0.12);%plot(x2(f,2),x2(f,3),'.')
x=x2(f,:);
n=polyfit(x(:,2),x(:,3),1);xn=x(:,2)*n(1)+n(2);f1=abs(x(:,3)-xn)<0.015;%plot(x(f1,2),x(f1,3),'.')
n=polyfit(x(f1,2),x(f1,3),1);xn=x(:,2)*n(1)+n(2);f2=abs(x(:,3)-xn)<0.01;%plot(x((f2),2),x((f2),3),'o');hold on;plot(x(:,2),x(:,3),'.')
% n=polyfit(x(f2,1),x(f2,3),1);xn=x(:,1)*n(1)+n(2);f3=abs(x(:,3)-xn)<0.01;%plot(x((f3),1),x((f3),3),'o');hold on;plot(x(:,1),x(:,3),'.')
f=f(f2);

x=x2;x11=sum(x(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R3=Rz*Ry*Rx;high=[0,0,h1(i)];x3=(R3*R2*R1*Xdr')'+repmat(high,length(Xdr),1);
R=R3*R2*R1;eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
h1(i)=h1(i)-mean(x3(f,3));x3(:,3)=x3(:,3)-mean(x3(f,3));%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
% plot3(x3(:,1),x3(:,2),x3(:,3),'.')
% f=find(abs(diff(x(:,3))./diff(x(:,2)))<0.12);%plot(x(:,2),x(:,3),'.');hold on;plot(x(f,2),x(f,3),'.')
% x=x(f,:);
% plot3(x(:,1),x(:,2),x(:,3),'.')
% pause
% f=find(abs(x3(:,2))<1&abs(x3(:,1))<4&abs(x3(:,3))<0.04);
% % if i>15&&length(f)>600
% x=x3;x11=sum(x(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
% D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
% n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
% tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
% R4=Rz*Ry*Rx;x4=(R4*x3')';h1(i)=h1(i)-mean(x4(abs(x4(:,3))<0.06,3));R=R4*R3*R2*R1;eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
%  plot3(x4(:,1),x4(:,2),x4(:,3),'.')

% R3=Rz*Ry*Rx;x=(R3*x(f,:)')';R=R3*R2*R1;eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
x=x3(f,:);x11=sum(x(:,1).^2);x22=sum(x(:,2).^2);x33=sum(x(:,3).^2);x12=sum(x(:,1).*x(:,2));x13=sum(x(:,1).*x(:,3));x23=sum(x(:,2).*x(:,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);N(i,:)=n1;
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R4=Rz*Ry*Rx;R=R4*R3*R2*R1;x4=(R*Xdr')'+repmat(high,length(Xdr),1);h1(i)=h1(i)-mean(x4(f,3));x4(:,3)=x4(:,3)-mean(x4(f,3));
% ,n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
% plot3(x(:,1),x(:,2),x(:,3),'.')
% pause
% plot(x(:,2),x(:,3),'.')
x=(R*Xdr')'+repmat([0,0,h1(i)],length(Xdr),1);%x=x-repmat(high,length(Xdr),1);
% t=mean(x(abs(x(:,3))<0.06,3));h1(i)=h1(i)-t;high=[0,0,t];
% x=x-repmat(high,length(Xdr),1);
% plot3(x(:,1),x(:,2),x(:,3),'.'),view(280,5),axis([0,4,-2,2,-0.5,1]/1),
subplot(1,2,1);plot(x(:,2),x(:,3),'.'),axis([-3,3,-0.5,1]/1),
subplot(1,2,2);plot(x(:,1),x(:,3),'.'),axis([-0,4,-0.5,1]/1),

pause(.1)
pcloud{i}=x;
% pcloud1{i}=x;
end
end
% plot(eul*180/pi)
% roll=eul(:,1)*180/pi;pitch=eul(:,2)*180/pi;
% pitch(2)=14.5;
% pitch=13.9-(n2(1:end-1,4))-(n2(2:end,4));pitch=[13.9;pitch];plot(pitch,'.-')
% hold on
% roll=4.7+(n2(1:end-1,5))+(n2(2:end,5));roll=[4.7;roll];plot(roll,'.-')
% pcloud=x;
%%
plot(N(:,1:2))
figure
plot(pitch1(20:length(I1))*180/pi)
hold on
plot(eul(:,2)*180/pi)
plot(pitch1(20:length(I1))*180/pi-12)
figure
plot(roll1(20:length(I1))*180/pi)
hold on
plot(eul(:,1)*180/pi)
%%
%find yaw and translation
%local map (slam) based on depth data
figure('units','normalized','outerposition',[0 0 1 1])
k=0;x=[];u=0;yawt=[];x=[];%pcloud1=pcloud;
for i=20:210%length(I1)-2
% for i=41%210%length(I1)-2
a=rgb2gray(I1{i}{1,1});i,%a=fliplr(rgb2gray(I)')';
subplot(2,2,1),imshow(a),
sc=25;b1=pcloud{i}*1e3/sc;b2=pcloud{i+1}*1e3/sc;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
% if length(b2)==0,k=k+1;else,K(i+1)=k;k=0;
k3=30;k4=80;
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
yawt(2,i)=trans(fx);yawt(3,i)=transy(fy);

px1a=px1-trans(fx);py1a=py1-transy(fy)-200;
s=[];yaw1=[-1:0.2:1]*pi/180;for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px1a';py1a'])');px1b=t1(:,1);py1b=t1(:,2)+200;
mpc1a=zeros(400,300);mpc1a((px1b-1).*size(mpc1a,1)+py1b)=pz;%imshow(mpc1)
s(j)=sum(sum((mpc2a-mpc1a).^2))/sum(sum(mpc2a>0&mpc1a>0));
end
f=find(s==min(s));S(i,1)=min(s);f=f(1);yawt(1,i)=yawt(1,i)+yaw1(f);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px1a';py1a'])');px1b=t1(:,1);py1b=t1(:,2)+200;
mpc1a=zeros(400,300);mpc1a((px1b-1).*size(mpc1a,1)+py1b)=pz;

% subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),
% pause(.1)%imshow(mpc1)
% i,pause
% m1(abs(m1)<5)=0;m2(abs(m2)<5)=0;
m1=mpc1a(k2:end-k2,k1:end-k2);m2=mpc2a(k2:end-k2,k1:end-k2);m1(:,110:end)=0;m2(:,110:end)=0;
m=m1;m(:,:,3)=m2;%subplot(2,2,[2,4]),imshow(m);pause(1)
subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),title(num2str(S(i,:)))
t=double((m1.*m2)>1);t2=mpc2*0;t2(k2:end-k2,k1:end-k2)=t;
c=conv2(t,ones(4,4)/16,'same');c=(c>0.6);t2=mpc2*0;t2(k2:end-k2,k1:end-k2)=c;
[py,px]=find(t2>0);px=px;py=py-200;px1=px+yawt(2,i);py1=py+yawt(3,i);
tetaz=-yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=(Rz*[px1';py1'])';
x=[x;t1];tetaz=yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x=[(Rz*[x]')'];x(:,1)=x(:,1)-yawt(2,i);x(:,2)=x(:,2)-yawt(3,i);%t2=mpc2*0;t1(10-yawt(3,i):end-10-yawt(3,i),k1-yawt(2,i):end-10-yawt(2,i))=t;t2(10:end-10,k1:end-10)=t;
x=[x;[px';py']'];tetaz=90*pi/180;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x1=[(Rz*[x]')'];
subplot(2,2,[2,4]),plot(x1(:,1),x1(:,2),'.'),axis([-120,120,-80,300])
hold on
plot([-300,100,300,-300,-300]/sc,[0,0,2000,2000,0]/sc)
hold off
pause(.1)
end
% end
%%
%find yaw and translation
%local map (slam) based on depth data
figure('units','normalized','outerposition',[0 0 1 1])
k=0;x=[];u=0;yawt=[];x=[];%pcloud1=pcloud;
for i=30:length(I1)-2
a=rgb2gray(I1{i}{1,1});i,%a=fliplr(rgb2gray(I)')';
subplot(2,2,1),imshow(a),
sc=25;b1=pcloud{i}*1e3/sc;b2=pcloud{i+1}*1e3/sc;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
% if length(b2)==0,k=k+1;else,K(i+1)=k;k=0;
k3=30;f=find((b2(:,3))>6&b2(:,1)>0&b2(:,1)>0&b2(:,1)<200);b2=round(b2(f,:));px2=b2(:,1);py2=b2(:,2)+200;pz2=round(b2(:,3)*1)+0;pz2(pz2>k3)=k3;%pz=round(b1(:,3)*2);
mpc2=zeros(400,300);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find((b1(:,3))>6&b1(:,1)>0&b1(:,1)>0&b1(:,1)<200);b1=round(b1(f,:));px=b1(:,1);py=b1(:,2)+200;pz=round(b1(:,3)*1)+0;pz(pz>k3)=k3;
mpc1a=zeros(400,300);mpc2a=zeros(400,300);for i=8:k3;mpc1=zeros(400,300);f=(pz==i);mpc1((px(f)-1).*size(mpc1,1)+py(f))=1;mpc1a=mpc1a+mpc1;mpc2=zeros(400,300);f=pz2==i;mpc2((px2(f)-1).*size(mpc2,1)+py2(f))=1;mpc2a=mpc2a+mpc2;end
[py,px]=find(mpc1a>0);py=py-200;pz=mpc1a(mpc1a>0);[py2,px2]=find(mpc2a>0);pz2=mpc2a(mpc2a>0);mpc2=mpc2a;
% s=[];yaw1=yaw11(i+0)+[-1:0.5:1]*pi/180;for j=1:length(yaw1)
s=[];yaw1=[-15:0.5:10]*pi/180;for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);yawt(1,i)=yaw1(f);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;
% plot(s)
% m=mpc1;m(:,:,3)=mpc2;imshow(m),pause(1)%imshow(mpc1)
k2=13;k1=50;transy=-8:1:8;trans=2:k2;m2=mpc2(k2:end-k2,k1:end-k2);s=[];
for j=1:length(trans)
for k=1:length(transy)
m1=mpc1(k2+transy(k):end-k2+transy(k),k1+trans(j):end-k2+trans(j));s(k,j)=sum(sum((m2-m1).^2));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);
m1=mpc1(k2+transy(fy):end-k2+transy(fy),k1+trans(fx):end-k2+trans(fx));
yawt(2,i)=trans(fx);yawt(3,i)=transy(fy);
% subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),
% pause(.1)%imshow(mpc1)
% i,pause
% m1(abs(m1)<5)=0;m2(abs(m2)<5)=0;
m1(:,150:end)=0;m2(:,150:end)=0;
m=m1;m(:,:,3)=m2;%subplot(2,2,[2,4]),imshow(m);pause(1)
subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),
t=double((m1.*m2)>1);t2=mpc2*0;t2(k2:end-k2,k1:end-k2)=t;
c=conv2(t,ones(4,4)/16,'same');c=(c>0.3);t2=mpc2*0;t2(k2:end-k2,k1:end-k2)=c;
[py,px]=find(t2>0);px=px;py=py-200;px1=px+yawt(2,i);py1=py+yawt(3,i);
tetaz=-yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=(Rz*[px1';py1'])';
x=[x;t1];tetaz=yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x=[(Rz*[x]')'];x(:,1)=x(:,1)-yawt(2,i);x(:,2)=x(:,2)-yawt(3,i);%t2=mpc2*0;t1(10-yawt(3,i):end-10-yawt(3,i),k1-yawt(2,i):end-10-yawt(2,i))=t;t2(10:end-10,k1:end-10)=t;
x=[x;[px';py']'];tetaz=90*pi/180;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x1=[(Rz*[x]')'];
subplot(2,2,[2,4]),plot(x1(:,1),x1(:,2),'.'),axis([-120,120,-80,300])
hold on
plot([-300,100,300,-300,-300]/sc,[0,0,2000,2000,0]/sc)
hold off
pause
end
% end
