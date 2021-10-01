% load('recordParams.mat')
% load('obstacle_sim.mat')
load('obstacle_sim2.mat')
% load('zebra_sim.mat')

%%
%show raw data
% msgs2 = readMessages(bagselectCAM);
% bagselectCAMP = select(bag, 'Topic','/camera/color/image_raw/compressed/parameter_descriptions');
% msgsCAMP = readMessages(bagselectCAMP);
for i=1:length(I)%:10:size(msgs2,1)
% msgs1 = readMessages(bagselect1,[i]);Xd = readXYZ(msgs1{1,1});Xdr=[Xd(:,3)';-Xd(:,1)';-Xd(:,2)']';Xdr(Xdr(:,1)>5,:)=0;range=1:5:length(Xd);
subplot(2,1,1)
% I{i} = readImage(msgs2{i,1});
imshow(I{i})
subplot(2,1,2)
Xdr=XYZ{i};
plot3(Xdr(:,1),Xdr(:,2),Xdr(:,3),'.'),view(270,10),axis([-1,6,-5,5,-2,2]/1),
pause(0.1)
end
%%
%plane detection 
n1=round([1:length(pitch)/length(I):length(pitch)]);%n1=n1(1:3:end);
roll1=roll(n1)-2*pi/180;pitch1=-(pitch(n1)+pi/2);yaw11=-(diff(yaw(n1)));plot(yaw11*180/pi)
pcloud=[];pcloud1=[];%for i=1:3:length(XYZ),XYZ1{(i-1)/3+1}=XYZ{i};I1{(i-1)/3+1}=I{i};end
for i=1:length(I)
Xdr=XYZ{i};%Xdr=[Xdr(:,3)';-Xdr(:,1)';-Xdr(:,2)']';
% if i==15,h1(i)=1.363;eul=[1,16.5,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% if i<=5,h1(i)=1.3539;eul(i,1:3)=[-2.5289,13.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
if i<=1,h1(i)=1.1546;eul(i,1:3)=[roll1(i),pitch1(i),0];tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
else h1(i)=h1(i-1);tetax=eul(i-1,1);tetay=eul(i-1,2);tetaz=eul(i-1,3);
% else h1(i)=h1(i-1);tetax=roll1(i);tetay=pitch1(i);tetaz=0;
end
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
plot3(x1(:,1),x1(:,2),x1(:,3),'.')

% plot3(x(:,1),x(:,2),x(:,3),'.')
x=x1;f=find(abs(x(:,2))<1.5&abs(x(:,1))<5&abs(x(:,3))<0.12);f1=find(abs(diff(x(f,3))./diff(x(f,2)))<0.12);
if length(f1)<800,h1(i)=h1(i-1);eul(i,:)=[roll1(i),pitch1(i),0];pcloud1{i}=x;else
x11=sum(x(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(f,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R2=Rz*Ry*Rx;x=(R2*x(f,:)')';h1(i)-mean(x(:,3));R=R2*R1;%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
f=find(abs(diff(x(:,3))./diff(x(:,2)))<0.15);%plot(x(:,2),x(:,3),'.');hold on;plot(x(f,2),x(f,3),'.')
% pause
 plot3(x(:,1),x(:,2),x(:,3),'.')
%  pause
f=find(abs(x(:,2))<1&abs(x(:,1))<4&abs(x(:,3)-mean(x(:,3)))<0.06);
x11=sum(x(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R3=Rz*Ry*Rx;x=(R3*x(f,:)')';h1(i)=h1(i)-mean(x(:,3));R=R3*R2*R1;%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
plot3(x(:,1),x(:,2),x(:,3),'.')
f=find(abs(diff(x(:,3))./diff(x(:,2)))<0.12);%plot(x(:,2),x(:,3),'.');hold on;plot(x(f,2),x(f,3),'.')
x=x(f,:);
plot3(x(:,1),x(:,2),x(:,3),'.')

f=find(abs(x(:,2))<1&abs(x(:,1))<4&abs(x(:,3)-mean(x(:,3)))<0.04);
% if i>15&&length(f)>600
x11=sum(x(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R4=Rz*Ry*Rx;x=(R4*x(f,:)')';h1(i)=h1(i)-mean(x(:,3));R=R4*R3*R2*R1;eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
 plot3(x(:,1),x(:,2),x(:,3),'.')

% R3=Rz*Ry*Rx;x=(R3*x(f,:)')';R=R3*R2*R1;eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
x11=sum(x(:,1).^2);x22=sum(x(:,2).^2);x33=sum(x(:,3).^2);x12=sum(x(:,1).*x(:,2));x13=sum(x(:,1).*x(:,3));x23=sum(x(:,2).*x(:,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1)
% ,n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
plot3(x(:,1),x(:,2),x(:,3),'.')
% pause
% plot(x(:,2),x(:,3),'.')
high=[0,0,h1(i)];x=(R*Xdr')'+repmat(high,length(Xdr),1);
% plot3(x(:,1),x(:,2),x(:,3),'.'),view(280,5),axis([0,4,-2,2,-0.5,1]/1),
plot(x(:,2),x(:,3),'.'),axis([-3,3,-0.5,1]/1),

pause(.1)
pcloud{i}=x;
pcloud1{i}=x;
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
% cd('C:\Users\ùùù\Downloads\2021-06-23-18-39-50')
% %%
% bag= rosbag('street_sample1.bag');
% %%
% figure('units','normalized','outerposition',[0 0 1 1])
% sel= select(bag,'Topic','/camera/aligned_depth_to_color/image_raw');
% bagselectCAM = select(bag,'Time',[sel.StartTime+20,sel.StartTime+35],'Topic','/camera/color/image_raw');msgs2 = readMessages(bagselectCAM);
% for i=1:length(msgs2);
% I{i} = readImage(msgs2{i,1});
% a=I{i};
% imshow(a),pause(0.1)
% end
%%
% msgs2 = readMessages(bagselectCAM);
% bagselectCAMP = select(bag, 'Topic','/camera/color/image_raw/compressed/parameter_descriptions');
% msgsCAMP = readMessages(bagselectCAMP);
% for i=1:100%:10:size(msgs2,1)
% % msgs1 = readMessages(bagselect1,[i]);Xd = readXYZ(msgs1{1,1});Xdr=[Xd(:,3)';-Xd(:,1)';-Xd(:,2)']';Xdr(Xdr(:,1)>5,:)=0;range=1:5:length(Xd);
% subplot(2,1,1)
% % I{i} = readImage(msgs2{i,1});
% imshow(I{i})
% subplot(2,1,2)
% Xdr=XYZ{i};
% plot3(Xdr(:,1),Xdr(:,2),Xdr(:,3),'.'),view(270,10),axis([-1,6,-5,5,-1,2]/1),
% pause(0.1)
% end
%%
%find yaw and translation
%local map (slam) based on depth data
figure('units','normalized','outerposition',[0 0 1 1])
k=0;x=[];u=0;yawt=[];x=[];%pcloud1=pcloud;
for i=1:length(I)-1
a=rgb2gray(I{i});%a=fliplr(rgb2gray(I)')';
subplot(2,2,1),imshow(a),
sc=25;b1=pcloud1{i}*1e3/sc;b2=pcloud1{i+1}*1e3/sc;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
% if i>1%length(b2)==0,k=k+1;else,K(i+1)=k;k=0;
f=find(abs(b2(:,3))>12.5&b2(:,1)>0&b2(:,1)<300);
b2=round(b2(f,:));px2=b2(:,1);py2=b2(:,2)+200;pz2=round(b2(:,3)*2);
mpc2=zeros(400,300);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find(abs(b1(:,3))>5.5&b1(:,1)>0&b1(:,1)<300);b1=round(b1(f,:));px=b1(:,1);py=b1(:,2);pz=round(b1(:,3)*2);
s=[];yaw1=yaw11(i-0)+[-0.5,0,0.5]*pi/180;for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);yawt(1,i)=yaw1(f);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;
% m=mpc1;m(:,:,3)=mpc2;imshow(m),pause(1)%imshow(mpc1)
transy=-3:1:3;trans=-0:6;k1=70;m2=mpc2(10:end-10,k1:end-10);s=[];
for j=1:length(trans)
for k=1:length(transy)
m1=mpc1(10+transy(k):end-10+transy(k),k1+trans(j):end-10+trans(j));s(k,j)=sum(sum((m2-m1).^2));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);
m1=mpc1(10+transy(fy):end-10+transy(fy),k1+trans(fx):end-10+trans(fx));
yawt(2,i)=trans(fx);yawt(3,i)=transy(fy);
% subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),
% pause(.1)%imshow(mpc1)
% i,pause
m=m1;m(:,:,3)=m2;
subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),
t=double(m1.*m2>0);t2=mpc2*0;t2(10:end-10,k1:end-10)=t;
c=conv2(t,ones(4,4)/16,'same');c=(c>0.3);t2=mpc2*0;t2(10:end-10,k1:end-10)=c;
[py,px]=find(t2>0);px=px;py=py-200;px1=px+yawt(2,i);py1=py+yawt(3,i);
tetaz=-yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=(Rz*[px1';py1'])';
x=[x;t1];tetaz=yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x=[(Rz*[x]')'];x(:,1)=x(:,1)-yawt(2,i);x(:,2)=x(:,2)-yawt(3,i);%t2=mpc2*0;t1(10-yawt(3,i):end-10-yawt(3,i),k1-yawt(2,i):end-10-yawt(2,i))=t;t2(10:end-10,k1:end-10)=t;
x=[x;[px';py']'];tetaz=90*pi/180;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x1=[(Rz*[x]')'];
subplot(2,2,[2,4]),plot(x1(:,1),x1(:,2),'.'),axis([-120,120,-80,300])
hold on
plot([-300,100,300,-300,-300]/sc,[0,0,2000,2000,0]/sc)
hold off
pause(1)
end
%%
%comparing euler angles from depth data with IMU data
n=round([1:length(pitch)/length(XYZ):length(pitch)]);n1=120;
subplot(3,1,1)
plot(eul(1:n1,1)*180/pi)
hold on
plot([1:n1],(roll(n(1:n1))*180/pi))
subplot(3,1,2)
plot(eul(1:n1,2)*180/pi)
hold on
plot([1:n1],-(pitch(n(1:n1))*180/pi+90))
subplot(3,1,3)
plot(yawt(1,1:n1)*180/pi)
hold on
plot([1:n1-1],-(diff(yaw(n(1:n1)))*180/pi))
figure

subplot(2,1,1)
plot(round(smooth(yawt(2,1:n1),3)))
hold on
plot([1:n1-1],round(1.5-(diff(yaw(n(1:n1)))*180/pi)))
subplot(2,1,2)
plot([1:n1-1],round(smooth(yawt(2,1:n1-1),3)')-(round(1.5-(diff(yaw(n(1:n1)))*180/pi))))
eulrec=[roll(n(1:n1));-(pitch(n(1:n1)))-pi/2;-(diff(yaw(n(1:n1+1))))]';
%%
% cd('C:\Users\ùùù\Documents\Avi\Vis_Imp\Read sensors\RealSense')
%local map based on IMU and depth data
% figure('units','normalized','outerposition',[0 0 1 1])
Dx=[];dxrec=(round(1.5-(diff(yaw(n(1:n1)))*180/pi)));
X=[];n3=[];n3(1)=1.313;B=[];
for i=1:n1-1
Xdr=XYZ{i};if i>1,n3(i)=n3(i-1);end
tetax=eulrec(i,1)-1*pi/180;tetay=eulrec(i,2);tetaz=0;-eulrec(i+1,3);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,n3(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;f=abs(x1(:,2))<1&abs(x1(:,1))<4&abs(x(:,3))<0.1;x1=x(f,:);x1=x1(abs(diff(x1(:,3))./diff(x1(:,2)))<0.12,:);
% x1=x2;x1=x(abs(x1(:,2))<1&abs(x1(:,1))<4&abs(x1(:,3))<0.07,:);x1=x1(abs(diff(x1(:,3))./diff(x1(:,2)))<0.12,:);
x11=sum(x1(:,1).^2);x22=sum(x1(:,2).^2);x33=sum(x1(:,3).^2);x12=sum(x1(:,1).*x1(:,2));x13=sum(x1(:,1).*x1(:,3));x23=sum(x1(:,2).*x1(:,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
v1=[a,b,D];v1=v1/norm(v1);%n3(i)=n3(i)-mean(x1(:,3)),
n2(i,1:6)=[v1,atan2(v1(1),v1(3))*180/pi,atan2(v1(2),v1(3)),mean(x1(:,3))];
tetax=(atan2(v1(2),v1(3)));tetay=-atan2(v1(1),v1(3));tetaz=0;eulrec(i,3);%yawt(1,i);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R3=Rz*Ry*Rx;x2=(R3*x(:,:)')';x2(:,3)=x2(:,3)-mean(x2(f,3));n3(i)=n3(i)-mean(x2(f,3));%h1(i)=h1(i)-mean(x(:,3));%
R=R3*R1;eul2(i,1:4)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),eulrec(i,3),n3(i)];
% plot3(x2(:,1),x2(:,2),x2(:,3)-mean(x1(:,3)),'.'),plot(x2(:,2),x2(:,3),'.'),axis([-3,3,-0.5,2.5]),
% pause
x3{i}=x2;
sc=25;b=x2*1e3/sc;b=round(b((abs(b(:,3))>7&b(:,1)>50&b(:,1)<250),:));px=b(:,1);py=b(:,2)+200;%plot(px,py,'.')
mpc=zeros(400,300);mpc((px-1).*size(mpc,1)+py)=1;%imshow(mpc);%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
M{i}=mpc;B{i}=b(:,1:2);
if i==1,X=B{i};M12{i}=mpc;end
if i>1,m=M{i-1};m(:,:,3)=M{i};%imshow(m),
mpc2=M{i};[py,px]=find(M12{i-1}>0);py=py-200;[pya,pxa]=find(M{i-1}>0);pya=pya-200;%b1=B{i-1};px=b1(:,1);py=b1(:,2);
s=[];yaw1=eulrec(i,3)+[-0.005,0,0.005];for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;
t1=round((Rz*[pxa';pya'])');px1=t1(:,1);py1=t1(:,2)+200;mpc1a=zeros(400,300);mpc1a((px1-1).*size(mpc1a,1)+py1)=1;
% transy=-3:1:3;transx=-1:8;k1=70;m2=mpc2(10:end-10,k1:end-10);s=[];
% transy=-5:1:5;transx=dxrec(i)+[-2:1:2];k1=100;m2=mpc2(10:end-10,k1:end-10);s=[];
transy=-5:1:5;transx=-1:6;k1=70;m2=mpc2(10:end-10,k1:end-10);s=[];s1=[];
for j=1:length(transx)
for k=1:length(transy)
m1=mpc1(10+transy(k):end-10+transy(k),k1+transx(j):end-10+transx(j));s(k,j)=sum(sum((m2-m1).^2));
m1=mpc1a(10+transy(k):end-10+transy(k),k1+transx(j):end-10+transx(j));s1(k,j)=sum(sum((m2-m1).^2));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);[fya,fxa]=find(s1==min(min(s1)));fxa=fxa(1);fx=round((0.2*fx+0.8*fxa));%fy=round((0.8*fx+0.2*fya(1)));%fy=fy(1);
% m1=mpc1(10+transy(fy):end-10+transy(fy),k1+transx(fx):end-10+transx(fx));Dx(1,i)=transx(fx);Dx(2,i)=transy(fy);
m1=mpc1*0;m1(10:end-10,k1:end-10)=mpc1(10+transy(fy):end-10+transy(fy),k1+transx(fx):end-10+transx(fx));Dx(1,i)=transx(fx);Dx(2,i)=transy(fy);Dx(3,i)=tetaz;
m=m1;m(:,:,3)=mpc2;%
subplot(2,2,[1,3]),imshow(m),
% mpc12=double(mpc1&mpc2);M12{i}=mpc12;%imshow(mpc12),
b1=B{i-1};b2=B{i};X=(Rz*X')';X(:,1)=X(:,1)-Dx(1,i);X(:,2)=X(:,2)-Dx(2,i);X=[X;b2];%X(:,1)=X(:,1)-Dx(1,i);X(:,2)=X(:,2)-Dx(2,i);
x=round(X);x=x(x(:,1)>50&abs(x(:,2)<200),:);x(:,2)=x(:,2)+200;mpc1=zeros(400,300);mpc1((x(:,1)-1).*size(mpc1,1)+x(:,2))=1;M12{i}=mpc1;
% b1=B{i-1};b2=B{i};X=(Rz*X')';b2(:,1)=b2(:,1)-Dx(1,i);b2(:,2)=b2(:,2)-Dx(2,i);X=[X;b2];
subplot(2,2,[2,4]),plot(X(:,2),X(:,1),'.'),axis([-120,120,-80,300])
% m=mpc1;m(:,:,3)=mpc2;imshow(m),
pause(.1)

end
% pause(1)
end

%%

%Move to Read_Ros3!!!!

% cd('C:\Users\ùùù\Documents\Avi\Vis_Imp\Read sensors\RealSense')
%match frames based on CAM
figure('units','normalized','outerposition',[0 0 1 1])
% Dx=[];dxrec=(round(1.5-(diff(yaw(n(1:n1)))*180/pi)));
% X=[];n3=[];n3(1)=1.313;
x=[];sc=1;f=1.93;sopix=1*3e-3/sc;%a=rgb2gray(I{i});imshow(a)%a=fliplr(rgb2gray(I)')';
r1=200;c1=180;yaw1=[];
for i=1:n1-1
a=rgb2gray(I{i});px0=size(a,2)/2;py0=size(a,1)/2;%imshow(a)  
% a=(conv2(single(edge(a,'canny')),ones(3,3)/9,'same')>0.4)*255;px0=size(a,2)/2;py0=size(a,1)/2;%imshow(a)
% a=(conv2(single(edge(a)),ones(2,2)/4,'same')>0.1)*255;px0=size(a,2)/2;py0=size(a,1)/2;%imshow(a)
% a=(imresize(a,[size(a,1)*1,size(a,2)*1.0]));px0=size(a,2)/2;py0=size(a,1)/2;
% [SWpixl,repvfl,ll,m]=Image_SW_Projection_5(edge(a,'canny'),[eul2(i,1:4),0],50,f,sopix,px0,py0,[],1e3*x3{i},0);
[SWpixl,repvfl,ll,m]=Image_SW_Projection_5(imadjust(a,[0.1,0.8],[0,1])-1,[eul2(i,1:4),0],50,f,sopix,px0,py0,[],1e3*x3{i},0);
% imshow(m(:,:,1))
% m=(m(:,:,1).*(m(:,:,2)>0));m=m(101:300,51:50+c1);M{i}=m;imshow(M{i})m=fliplr(edge(a','canny'));
m=(m(:,:,1).*(m(:,:,2)>0));m=edge(m(size(m,1)/2-r1/2+1:size(m,1)/2+r1/2,121:120+c1),'canny');M{i}=255*m;%imshow(M{i})
% pause
if i==1,M12{i}=M{i};else%i>1%,[py,px]=find(M{i}>0);py=py-200;x=[px';py']';else 
% m1=M{i-1};m2=M{i};[py,px]=find(m1>0);py=py-200;[py2,px2]=find(m2>0);py2=py2-200;%b1=B{i-1};px=b1(:,1);py=b1(:,2);
m1=M{i-1};m1(1:10,:)=0;m1(:,1:10)=0;m1(:,end-10:end)=0;m1(end-10:end,:)=0;
m2=M{i};m2(1:10,:)=0;m2(:,1:10)=0;m2(:,end-10:end)=0;m2(end-10:end,:)=0;
% pause
[py,px]=find(m1>0);f_ind=find(m1>0);py=py-r1/2;[py2,px2]=find(m2>0);py2=py2-r1/2;%b1=B{i-1};px=b1(:,1);py=b1(:,2);
s=[];yaw1=Dx(3,i)+[-0.008,0,0.008];for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+r1/2;
m1a=zeros(r1,c1);m1a((px1-1).*size(m1,1)+py1)=m1(f_ind);%imshow(mpc1)
s(j)=sum(sum((m2-m1a).^2));
end
fs=find(s==min(s));fs=fs(1);tetaz=yaw1(fs);eulcam(i,1)=tetaz;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+r1/2;
m1a=zeros(r1,c1);m1a((px1-1).*size(m1,1)+py1)=m1(f_ind);Ma{i-1}=m1a;
% m=m1a;m(:,:,3)=m2;imshow(m),
s=[];transx=Dx(1,i-1)+[-2:2];transy=Dx(2,i-1)+[-2:2];k1=81;m2a=m2(10:end-10-k1,10:end-10);s=[];
for j=1:length(transx)
for k=1:length(transy)
m1b=m1a(10+transy(k):end-10+transy(k)-k1,10+transx(j):end-10+transx(j));s(k,j)=sum(sum((m2a-m1b).^2));
% m=m1b;m(:,:,3)=m2a;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);m1b=m1*0;m1b(10:end-10,10:end-10)=m1a(10+transy(fy):end-10+transy(fy),10+transx(fx):end-10+transx(fx));
Dxcam(1,i)=transx(fx);Dxcam(2,i)=transy(fy);
% m=m1b;m(:,:,3)=m2;imshow(m),pause
subplot(2,2,[1,3]),m=m1b;m(:,:,3)=m2;imshow(m)%subplot(2,2,[1,3]),

% transy=[-4:4];k1=0;m2a=m2(10:end-10,:);m2a(:,1:k1)=0;s=[];for k=1:length(transy)
% m1c=m1b(10+transy(k):end-10+transy(k),:);s(k)=sum(sum((m2a-m1c).^2));
% % m=m1b;m(:,:,3)=m2a;imshow(m),pause(1)
% end
% fy=find(s==min((s)));fy=fy(1);m1c=m1*0;m1c(10:end-10,:)=m1b(10+transy(fy):end-10+transy(fy),:);Dxcam(2,i)=transy(fy);
% m=m1c;m(:,:,3)=m2;%subplot(2,2,[1,3]),
%  imshow(m),
% pause

% transy=[-4:4];transx=[-1:8];k1=12;m2a=m2(10:end-10,k1:end-12);s=[];
% for j=1:length(transx)
% for k=1:length(transy)
% m1b=m1a(10+transy(k):end-10+transy(k),k1+transx(j):end-12+transx(j));s(k,j)=sum(sum((m2a-m1b).^2));
% % m=m1b;m(:,:,3)=m2a;imshow(m),
% % pause(1)
% end
% end
% [fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);
% % m1=mpc1(10+transy(fy):end-10+transy(fy),k1+transx(fx):end-10+transx(fx));Dx(1,i)=transx(fx);Dx(2,i)=transy(fy);
% m1b=m1a*0;m1b(10:end-10,k1:end-12)=m1a(10+transy(fy):end-10+transy(fy),k1+transx(fx):end-12+transx(fx));
% Dxcam(1,i)=transx(fx);Dxcam(2,i)=transy(fy);
% m=m1b;m(:,:,3)=m2;%subplot(2,2,[1,3]),
% subplot(2,2,[1,3]), 
% subplot(2,2,[1,3]),m=m1a;m(:,:,3)=m2;imshow(m),
% pause

% x=[x;[px';py']'];x=round((Rz*x')');x(:,1)=x(:,1)-Dx(1,i);x(:,2)=x(:,2)-Dx(2,i);x=([x;[px2';py2']']);
x=[[px';py']'];x=round((Rz*x')');x(:,1)=x(:,1)-Dxcam(1,i);x(:,2)=x(:,2)-Dxcam(2,i);
x=([x;[px2';py2']']);x=x(x(:,1)>0&abs(x(:,2))<200,:);
% x=x(x(:,1)>50&abs(x(:,2)<200),:);
subplot(2,2,[2,4]),plot(x(:,2),x(:,1),'.'),axis([-120,120,-80,300])
x(:,2)=x(:,2)+r1/2;mpc1=zeros(r1,c1);mpc1((x(:,1)-1).*size(mpc1,1)+x(:,2))=1;M12{i}=mpc1;

pause(.1)
    
end
end
%%
subplot(2,1,1),plot(eulrec(:,3)),hold on,plot(eulcam,'.-')
subplot(2,1,2),plot(Dx(1,:)'),hold on,plot(Dxcam(1,:)')


%%
    for i=74%1:50;
        x=1e3*XYZ{i};
tetax=eul2(i,1);tetay=eul2(i,2);tetaz=0;
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
% tetay=-90*pi/180;Ry90=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
x=(Rz*Ry*Rx*x')';
x(:,3)=x(:,3)+eul2(i,4)*1e3;
x2=x;plot3(x2(:,1),x2(:,2),x2(:,3),'.');%pause
        
        plot(x(:,2),x(:,3),'.');axis([-2000,2000,-60,60]),pause(1);
    end
%%
Xdr=XYZ{i};if i>1,n3(i)=n3(i-1);end
tetax=eulrec(i,1)-1*pi/180;tetay=eulrec(i,2);tetaz=0;-eulrec(i+1,3);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,n3(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;x1=x(abs(x1(:,2))<1&abs(x1(:,1))<4&abs(x(:,3))<0.06,:);x1=x1(abs(diff(x1(:,3))./diff(x1(:,2)))<0.12,:);
% x1=x2;x1=x(abs(x1(:,2))<1&abs(x1(:,1))<4&abs(x1(:,3))<0.07,:);x1=x1(abs(diff(x1(:,3))./diff(x1(:,2)))<0.12,:);
x11=sum(x1(:,1).^2);x22=sum(x1(:,2).^2);x33=sum(x1(:,3).^2);x12=sum(x1(:,1).*x1(:,2));x13=sum(x1(:,1).*x1(:,3));x23=sum(x1(:,2).*x1(:,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
v1=[a,b,D];v1=v1/norm(v1);n3(i)=n3(i)-mean(x1(:,3));n2(i,1:6)=[v1,atan2(v1(1),v1(3))*180/pi,atan2(v1(2),v1(3)),mean(x1(:,3))];
tetax=(atan2(v1(2),v1(3)));tetay=-atan2(v1(1),v1(3));tetaz=0;eulrec(i,3);%yawt(1,i);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R3=Rz*Ry*Rx;x2=(R3*x(:,:)')';%h1(i)=h1(i)-mean(x(:,3));%R=R3*R2*R1;%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
% plot3(x2(:,1),x2(:,2),x2(:,3)-mean(x1(:,3)),'.'),plot(x2(:,2),x2(:,3),'.'),axis([-3,3,-0.5,2.5]),
% pause(.1)
sc=25;b=x2*1e3/sc;b=round(b((abs(b(:,3))>7&b(:,1)>50&b(:,1)<250),:));px=b(:,1);py=b(:,2)+200;%plot(px,py,'.')
mpc=zeros(400,300);mpc((px-1).*size(mpc,1)+py)=1;%imshow(mpc);%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
M{i}=mpc;B{i}=b(:,1:2);
if i==1,X=B{i};M12{i}=mpc;end
if i>1,m=M{i-1};m(:,:,3)=M{i};%imshow(m),
mpc2=M{i};[py,px]=find(M12{i-1}>0);py=py-200;%b1=B{i-1};px=b1(:,1);py=b1(:,2);
s=[];yaw1=eulrec(i,3)+[-0.007,0,0.007];for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;

% transy=-3:1:3;transx=-1:8;k1=70;m2=mpc2(10:end-10,k1:end-10);s=[];
transy=-5:1:5;transx=dxrec(i)+[-2:1:2];k1=100;m2=mpc2(10:end-10,k1:end-10);s=[];
for j=1:length(transx)
for k=1:length(transy)
m1=mpc1(10+transy(k):end-10+transy(k),k1+transx(j):end-10+transx(j));s(k,j)=sum(sum((m2-m1).^2));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);
% m1=mpc1(10+transy(fy):end-10+transy(fy),k1+transx(fx):end-10+transx(fx));Dx(1,i)=transx(fx);Dx(2,i)=transy(fy);
m1=mpc1*0;m1(10:end-10,k1:end-10)=mpc1(10+transy(fy):end-10+transy(fy),k1+transx(fx):end-10+transx(fx));Dx(1,i)=transx(fx);Dx(2,i)=transy(fy);
m=m1;m(:,:,3)=mpc2;%
subplot(2,2,[1,3]),imshow(m),
% mpc12=double(mpc1&mpc2);M12{i}=mpc12;%imshow(mpc12),
b1=B{i-1};b2=B{i};X=(Rz*X')';X(:,1)=X(:,1)-Dx(1,i);X(:,2)=X(:,2)-Dx(2,i);X=[X;b2];%X(:,1)=X(:,1)-Dx(1,i);X(:,2)=X(:,2)-Dx(2,i);
x=round(X);x=x(x(:,1)>50&abs(x(:,2)<200),:);x(:,2)=x(:,2)+200;mpc1=zeros(400,300);mpc1((x(:,1)-1).*size(mpc1,1)+x(:,2))=1;M12{i}=mpc1;
% b1=B{i-1};b2=B{i};X=(Rz*X')';b2(:,1)=b2(:,1)-Dx(1,i);b2(:,2)=b2(:,2)-Dx(2,i);X=[X;b2];
subplot(2,2,[2,4]),plot(X(:,2),X(:,1),'.'),axis([-120,120,-80,300])
% m=mpc1;m(:,:,3)=mpc2;imshow(m),
pause(.2)

end
% pause(1)
end

%%
f=find(abs(diff(x(:,3))./diff(x(:,2)))<0.12);%plot(x(:,2),x(:,3),'.');hold on;plot(x(f,2),x(f,3),'.')
x=x(f,:);
plot3(x(:,1),x(:,2),x(:,3),'.')

pause(.1)
end

%%
cd('C:\Users\ùùù\Documents\Avi\Vis_Imp\þþDataYehudit2')
x=[];u=0;
for i=96%:59;
eul1=[[roll(i),pitch(i),-0]*pi/180,h1(i)*1e3];
sc=4;f=1.88;sopix=7*1.4e-3/sc;a=rgb2gray(I{i});imshow(a)%a=fliplr(rgb2gray(I)')';
a=(imresize(a,[size(a,1)*sc,size(a,2)*sc]));px0=size(a,2)/2;py0=size(a,1)/2;
a=(conv2(single(edge(a,'canny')),ones(3,3),'same')>0)*255;px0=size(a,2)/2;py0=size(a,1)/2;imshow(a)
b=pcloud{i};yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
[SWpixl,repvfl,ll,m_m3,mpc]=Image_SW_Projection_4(a,eul1,30,eul1(4),f,sopix,px0,py0,[],1e3*pcloud{i},0);
[SWpixl,repvfl,ll,m_m2,mpc]=Image_SW_Projection_4(a,eul1,30,eul1(4),f,sopix,px0,py0,[],1e3*pcloud{i},yaw(2));
[SWpixl,repvfl,ll,m_m1,mpc]=Image_SW_Projection_4(a,eul1,30,eul1(4),f,sopix,px0,py0,[],1e3*pcloud{i},yaw(3));
[SWpixl,repvfl,ll,m_0,mpc]=Image_SW_Projection_4(a,eul1,30,eul1(4),f,sopix,px0,py0,[],1e3*pcloud{i},yaw(4));
[SWpixl,repvfl,ll,m_1,mpc]=Image_SW_Projection_4(a,eul1,30,eul1(4),f,sopix,px0,py0,[],1e3*pcloud{i},yaw(5));
[SWpixl,repvfl,ll,m_2,mpc]=Image_SW_Projection_4(a,eul1,30,eul1(4),f,sopix,px0,py0,[],1e3*pcloud{i},yaw(6));
[SWpixl,repvfl,ll,m_3,mpc]=Image_SW_Projection_4(a,eul1,30,eul1(4),f,sopix,px0,py0,[],1e3*pcloud{i},yaw(7));
mpc=m_m3(:,:,3);mpc(:,:,2)=m_m2(:,:,3);mpc(:,:,3)=m_m1(:,:,3);mpc(:,:,4)=m_0(:,:,3);mpc(:,:,5)=m_1(:,:,3);mpc(:,:,6)=m_2(:,:,3);mpc(:,:,7)=m_3(:,:,3);% m=(m(size(m,1)/2-150:size(m,1)/2+150,1:250)');
% mim=m_m1(:,:,1);mim(:,:,2)=m_0(:,:,1);mim(:,:,3)=m_1(:,:,1);% m=(m(size(m,1)/2-150:size(m,1)/2+150,1:250)');
mim=m_m3(:,:,1);mim(:,:,2)=m_m2(:,:,1);mim(:,:,3)=m_m1(:,:,1);mim(:,:,4)=m_0(:,:,1);mim(:,:,5)=m_1(:,:,1);mim(:,:,6)=m_2(:,:,1);mim(:,:,7)=m_3(:,:,1);% m=(m(size(m,1)/2-150:size(m,1)/2+150,1:250)');
% mpc=(mpc(size(mpc,1)/2-150:size(mpc,1)/2+150,1:250)');
% c=double(mpc>0);c=conv2(mpc,ones(5,5)/25,'same');c1=c*0;c1(c>0.1)=1;imshow(c1)
% f=find(m<0.01);m(f(3:end-2))=m(f(3:end-2)+2);
% f=find(m<0.01);m(f(3:end-2))=m(f(3:end-2)+1);
% imshow(m_0)
% m(:,:,2)=m;m(:,:,3)=c1;
% imshow(m)
% pause(0.1)
if i>1&&size(mpc1,1)==size(mpc,1)&&u>0,
ra=100;sm=size(mpc)/2;t=mpc1(sm(1)-ra:sm(1)+ra,40:round(2.2*ra),:);mpc2=mpc(sm(1)-ra:sm(1)+ra,40:round(2.2*ra),:);
% s=[sqrt(sum(sum((t(:,:,2)-mpc2(:,:,1)).^2))),sqrt(sum(sum((t(:,:,2)-mpc2(:,:,2)).^2))),sqrt(sum(sum((t(:,:,2)-mpc2(:,:,3)).^2)))];
% s=[sqrt(sum(sum((t(:,:,1)-mpc2(:,:,2)).^2))),sqrt(sum(sum((t(:,:,2)-mpc2(:,:,2)).^2))),sqrt(sum(sum((t(:,:,3)-mpc2(:,:,2)).^2)))];
s=[sqrt(sum(sum((t(:,:,1)-mpc2(:,:,4)).^2))),sqrt(sum(sum((t(:,:,2)-mpc2(:,:,4)).^2))),sqrt(sum(sum((t(:,:,3)-mpc2(:,:,4)).^2))),sqrt(sum(sum((t(:,:,4)-mpc2(:,:,4)).^2))),sqrt(sum(sum((t(:,:,5)-mpc2(:,:,4)).^2))),sqrt(sum(sum((t(:,:,6)-mpc2(:,:,4)).^2))),sqrt(sum(sum((t(:,:,7)-mpc2(:,:,4)).^2)))];
fyaw=find(s==min(s)),s,%s1=1./(s-8000);s1=s1./sum(s1);sum(yaw.*s1)/1*(180/pi)
n(i)=fyaw;m=(mpc2(:,:,4));m(:,:,3)=(t(:,:,fyaw));mpct=m;
% subplot(1,2,1),imshow(m)
% pause

t=mim1(sm(1)-ra:sm(1)+ra,40:round(2.2*ra),:);mim2=mim(sm(1)-ra:sm(1)+ra,40:round(2.2*ra),:);
% m=(mim2(:,:,4));m(:,:,3)=(t(:,:,fyaw));
m=(mpct(:,:,1));m(:,:,3)=(mpct(:,:,3));
k1=10;k2=60;t2=m(k1:end-k1,k2:end-30,1);t1=m(k1:end-k1,:,3);%t11=t1;
k=2;t11=t1(:,k2+k*1:end-30+k*1);t11(:,:,2)=t1(:,k2+k*2:end-30+k*2);t11(:,:,3)=t1(:,k2+k*3:end-30+3*k);t11(:,:,4)=t1(:,k2+k*4:end-30+k*4);t11(:,:,5)=t1(:,k2+k*5:end-30+5*k);t11(:,:,6)=t1(:,k2+k*6:end-30+6*k);
s=[sqrt(sum(sum((t2-t11(:,:,1)).^2))),sqrt(sum(sum((t2-t11(:,:,2)).^2))),sqrt(sum(sum((t2-t11(:,:,3)).^2))),sqrt(sum(sum((t2-t11(:,:,4)).^2))),sqrt(sum(sum((t2-t11(:,:,5)).^2))),sqrt(sum(sum((t2-t11(:,:,6)).^2)))]/sum(sum(t2>0));
f=find(s==min(s));f=f(1),s,
m=t2;m(:,:,3)=t11(:,:,f);m(:,:,2)=mpct(k1:end-k1,k2:end-30,1);
subplot(2,2,2),imshow(m),%imshow(a)
% pause
t1=mpct(:,:,3);t2=mpct(:,:,1);t1=t1(:,1+(f-0)*k:end);t2=t2(:,1:end-((f-0)*k));m=t1;m(:,:,3)=t2;t12=(m(:,:,1)&m(:,:,3));
subplot(2,2,1),imshow(m),title(num2str(yaw(fyaw)*180/pi))
% pause
% [py1,px1]=find(mpct(:,:,3)>0);py1=py1-100;[py2,px2]=find(mpct(:,:,1)>0);py2=py2-100;px2=px2+((f-1)*k);
% tetaz=yaw(fyaw);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t=(Rz*[px2';py2'])';
% [py1,px1]=find(mpc1(sm(1)-ra:sm(1)+ra,40:2*ra,2)>0);py1=py1-100;[py2,px2]=find(mpct(:,:,1)>0);py2=py2-100;
t1=t12*0;t1=t12;%t1(:,1+(f-1)*k:end)=t12(:,1:end-((f-1)*k));
[py1,px1]=find(t1>0);py1=py1-100;px1=px1+40+k2-1;px2=px1;py2=py1;tetaz=-yaw(fyaw);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=(Rz*[px1';py1'])';x=[x;[t1(:,1)';t1(:,2)']'];
% [py1,px1]=find(mpc1(sm(1)-ra:sm(1)+ra,40:2*ra,2)>0);py1=py1-100;[py2,px2]=find(mpct(:,:,1)>0);py2=py2-100;
% x=[x;[px1';py1']'];
tetaz=yaw(fyaw);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x(:,1)=x(:,1)-((f-0)*k);x=[(Rz*[x]')'];x=[x;[px2';py2']'];
subplot(2,2,[3,4]),plot(x(:,1),-x(:,2),'.'),axis([-20,240,-120,120])

i
pause(.1)
end

mpc1=mpc;
mim1=mim;
t=mpc;u=1;
end
%%
%camera frame fit
x=[];u=0;sc=4;f=1.88;sopix=6*1.4e-3/sc;%imshow(a)%a=fliplr(rgb2gray(I)')';
for i=1%:10:99%:59;
eul1=[[roll(i),pitch(i),-0]*pi/180,h1(i)*1e3,-10];
% a=(conv2(single(edge(a,'canny')),ones(3,3),'same')>0)*255;px0=size(a,2)/2;py0=size(a,1)/2;imshow(a)
a=rgb2gray(I{i});a=(imresize(a,[size(a,1)*sc,size(a,2)*sc]));px0=size(a,2)/2;py0=size(a,1)/2;
% a=(conv2(single(edge(a,'canny')),ones(3,3),'same')>0)*255;
[SWpixl,repvfl,ll,m1,mpc]=Image_SW_Projection_4(a,eul1,-60,f,sopix,px0,py0,[],1e3*pcloud{i},0);
a=rgb2gray(I{i+1});a=(imresize(a,[size(a,1)*sc,size(a,2)*sc]));px0=size(a,2)/2;py0=size(a,1)/2;
% a=(conv2(single(edge(a,'canny')),ones(3,3),'same')>0)*255;
[SWpixl,repvfl,ll,m2,mpc]=Image_SW_Projection_4(a,eul1,-60,f,sopix,px0,py0,[],1e3*pcloud{i},0);

t=double((m1(:,:,2).*m2(:,:,2))>0);
mpc2=t.*m2(:,:,1);mpc1=t.*m1(:,:,1);mpc1(1:10,1:10)=0;
s=[];dx=[0:1:7];dy=[-4:1:4];t2=mpc2(10:end-10,10:end-10);for j=1:length(dx)
    for k=1:length(dy)
        t1=mpc1(10+dy(k):end-10+dy(k),10+dx(j):end-10+dx(j));f=t1>0&t2>0;
s(k,j)=sum(sum((t2(f)-t1(f)).^2));
    end
end

[fy,fx]=find(s==min(min(s)));dxy(1,i)=dx(fx);dxy(2,i)=dy(fy);
t1=mpc1(10+dy(fy):end-10+dy(fy),10+dx(fx):end-10+dx(fx));
mpc1a=mpc1*0;mpc1a(10+dy(fy):end-10+dy(fy),10+dx(fx):end-10+dx(fx))=t1;
m=mpc1a;m(:,:,3)=mpc2;imshow(m);%imshow(t1),figure,imshow(t2)

[py,px]=find(mpc1a>0);fp=mpc1a(mpc1a>0);px=round(px);py=round(py)-200;%px=px(f);py=py(f);
s=[];yaw=[-4:1:4]*pi/180;for j=6%1:length(yaw)
tetaz=yaw(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t=round((Rz*[px';py'])');px1=t(:,1);py1=t(:,2)+200;
mpc1a=zeros(400,400);mpc1a((px1-1).*size(mpc1a,1)+py1)=fp;s(j)=sum(sum((mpc2-mpc1a).^2));
end

f=find(s==min(s));f=f(1);f=2;yawtim(1,i)=yaw(f);tetaz=yaw(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t=round((Rz*[px';py'])');px1=t(:,1);py1=t(:,2)+200;
mpc1a=zeros(400,400);mpc1a((px1-1).*size(mpc1,1)+py1)=fp;m=mpc2;m(:,:,3)=mpc1a;imshow(m)

mpc1=zeros(400,400);mpc1((px1-1).*size(mpc1,1)+py1)=fp;m=mpc2;m(:,:,3)=mpc1;imshow(m)
% subplot(2,1,1),imshow(mpc2),subplot(2,1,2),imshow(mpc1)
pause(1)
end
%%
% pitch=[];pitch(1:20)=-75;t=[smooth(magF(:,1),20)';smooth(magF(:,2),20)';smooth(magF(:,3),20)']'/9.60;
bagselectIMU = select(bag,'Time',[sel.StartTime+30,sel.StartTime+302], 'Topic','/imu');
msgsIMU = readMessages(bagselectIMU);
eulZYX=zeros(size(msgsIMU,1),3);
% for i=round([1:size(msgsIMU,1)/size(msgs2,1):size(msgsIMU,1)]);
n=round([1:size(msgsIMU,1)]);
for i1=20:length(n)/1;
i=n(i1);quat(i,1:4) = [msgsIMU{i}.Orientation.W msgsIMU{i}.Orientation.Y msgsIMU{i}.Orientation.Y msgsIMU{i}.Orientation.Z];
magF(i1,1:3)=[-msgsIMU{i}.LinearAcceleration.Y,msgsIMU{i}.LinearAcceleration.X,-msgsIMU{i}.LinearAcceleration.Z];
magW(i1,1:3)=[msgsIMU{i}.AngularVelocity.X+0.083,-(msgsIMU{i}.AngularVelocity.Y+0.011),-(msgsIMU{i}.AngularVelocity.Z-0.011)];
eulZYX(i1,:) = quat2eul(quat(i,:),'ZYZ');
t=magF(i1,:);t1=sqrt(t(2).^2+t(3).^2);%norm(t);%sqrt(sum(t'.^2));plot(sqrt(sum(magF'.^2))/9.8)
% if abs(t1-1)<0.05,t=t/norm(t);pitch(i1)=atan(t(3)./sqrt(t(1).^2+t(2).^2))*180/pi;
% if abs(t(i1,3)+0.95)+abs(t(i1,1)-0.39)<0.05%,t=t/norm(t);
%     pitch(i1)=pitch(i1-3)/4+pitch(i1-2)/4+pitch(i1-1)/4+1/4*atan(t(i1,3)./sqrt(t(i1,1).^2+t(i1,2).^2))*180/pi;
% else
%     pitch(i1)=pitch(i1-1)-0.020*magW(i1,2)*180/pi;
% end
% plot(pitch)
% [rotationAng1 rotationAng2 rotationAng3] = q2e(quat(1),quat(2),quat(3),quat(4));
% [rotationAng1 rotationAng2 rotationAng3] = quat2angle(quat);
% eulZYX(i,:) = [rotationAng1 rotationAng2 rotationAng3];
end
%%
pitch=[];roll=[];yaw=[];csum=cumsum(magW);x=[1,0,0;0,0,0;0,1,0;0,0,0;0,0,1;0,0,0];
eul=[3.2,15,0]*pi/180;tetax=eul(1);tetay=eul(2);tetaz=eul(3);Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R=(Rz*Ry*Rx);
for i=1:10:1500%:10:2500%:10:length(magW)
eul=csum(i,:)*pi/180;tetax=eul(1);tetay=eul(2);tetaz=eul(3);Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
Rt=Rz*Ry*Rx*R;rx=(Rt*x')';
pitch(i)=-asin(Rt(3,1))*180/pi;
roll(i)=atan2(Rt(3,2),Rt(3,3))*180/pi;
yaw(i)=atan2(Rt(2,1),Rt(1,1))*180/pi;
% plot3(x(:,1),x(:,2),x(:,3),'.-')
% hold on
subplot(1,2,1),plot3(rx(:,1),rx(:,2),rx(:,3),'.-')
axis([-2,2,-2,2,-2,2]/2)
subplot(1,2,2),plot([pitch;roll;yaw]','.-')
pause(0.1)
end
%%
% nmagG=magG./repmat(sqrt(sum(magG'.^2)),3,1)';
magG1=[smooth(magF(:,1),20)';smooth(magF(:,2),20)';smooth(magF(:,3),20)']';
nmagG1=magG1./repmat(sqrt(sum(magG1'.^2)),3,1)';
pitch=atan(nmagG1(:,3)./sqrt(nmagG1(:,1).^2+nmagG1(:,2).^2))*180/pi;
roll=atan(nmagG1(:,1)./sqrt(nmagG1(:,3).^2+nmagG1(:,1).^2))*180/pi;
plot(pitch);hold on;plot(roll);
% interpeulZYX=[interp1([1:length(eulZYX)]',eulZYX(:,1),round([1:size(msgsIMU,1)/size(msgs2,1):size(msgsIMU,1)])')';...
% interp1([1:length(eulZYX)]',eulZYX(:,2),round([1:size(msgsIMU,1)/size(msgs2,1):size(msgsIMU,1)])')';...
% interp1([1:length(eulZYX)]',eulZYX(:,3),round([1:size(msgsIMU,1)/size(msgs2,1):size(msgsIMU,1)])')']';
%%
% for i=1:size(interpeulZYX,1)
% for i=1:size(eulZYX,1)
ind=round([1:size(msgsIMU,1)/size(msgs2,1):size(msgsIMU,1)]);
for i1=400:600,%1:length(ind),
    i=ind(i1);
tetax=eulZYX(i,1);tetay=eulZYX(i,2);tetaz=eulZYX(i,3);
% tetax=interpeulZYX(i,1);tetay=interpeulZYX(i,2);tetaz=interpeulZYX(i,3);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
Ry=[cos(tetay),0,-sin(tetay);0,1,0;sin(tetay),0,cos(tetay)];
Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
x=[0,1,0,0,0,0;0,0,0,1,0,0;0,0,0,0,0,1]';x=(Rz*Ry*Rx*x')';
plot3(x(:,1),x(:,2),x(:,3))
axis([-1,1,-1,1,-1,1])
pause(0.1)
end
%%
select3 = select(bag,'Topic','/camera/depth/color/points');
pcl2 = readMessages(select3,'DataFormat','struct');

bagImu = select(bag,'Topic','/imu');
imu = readMessages(bagImu,'DataFormat','sensor_msgs/Imu');

%%
msgStruct


s = readMessages(bagselect1, 'DataFormat', 'struct');
msg = copyImage(msgStructs{1});
I = readImage(msg);

a=msgs{1,1}.Data;
%%
filePath = fullfile(fileparts(which('ROSWorkingWithRosbagsExample')), 'data', 'ex_multiple_topics.bag');

bagselect = rosbag(filePath);
sel= select(bag,'Topic', '/odom');

bagselect2 = select(bagselect, 'Time',[bagselect.StartTime bagselect.StartTime + 1], 'Topic', '/odom');
firstMsgs = readMessages(bagselect2,1:10);