
%5 FPS
%%
%plane detection 
h1=[];eul=[];n1=round([1:length(pitch)/length(I):length(pitch)]);n1=n1(1:3:end);
roll1=roll(n1)-3*pi/180;pitch1=-(pitch(n1)+pi/2);yaw11=-(diff(yaw(n1)));plot(yaw11*180/pi)
pcloud=[];pcloud1=[];for i=1:3:length(XYZ),XYZ1{(i-1)/3+1}=XYZ{i};I1{(i-1)/3+1}=I{i};end
for i=1:length(I1)-1
Xdr=XYZ1{i};i,%Xdr=[Xdr(:,3)';-Xdr(:,1)';-Xdr(:,2)']';
% if i==15,h1(i)=1.363;eul=[1,16.5,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% if i<=1,h1(i)=1.5439;eul(i,1:3)=[-2.5289,13.1646,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
if i<=1,h1(i)=1.15439;eul(i,1:3)=[roll1(i),pitch1(i),0];tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
% else h1(i)=h1(i-1);tetax=eul(i-1,1);tetay=eul(i-1,2);tetaz=eul(i-1,3);
else h1(i)=h1(i-1);tetax=roll1(i);tetay=pitch1(i);tetaz=0;
end
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
plot3(x1(:,1),x1(:,2),x1(:,3),'.')
%     pause
% plot3(x(f,1),x(f,2),x(f,3),'.')
x=x1;t=(abs(diff(x1(:,3))./diff(x1(:,2))));f=find(abs(x1(:,2))<1.5&abs(x1(:,1))<5&abs(x1(:,3))<0.25&[(abs(diff(x1(:,3))./diff(x1(:,2))));1]<0.15&[1;(abs(diff(x1(:,3))./diff(x1(:,2))))]<0.15);
f1=abs(x1(f,3)-prctile(x1(f,3),50))<0.1;f=f(f1);
plot(x1(f,2),x1(f,3),'.')

if length(f)<400,
    pause
h1(i)=h1(i-1);eul(i,:)=[roll1(i),pitch1(i),0];pcloud1{i}=x;
else
x11=sum(x1(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(f,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
h1(i)=h1(i);high=[0,0,h1(i)];R2=Rz*Ry*Rx;x2=(R2*R1*Xdr')'+repmat(high,length(Xdr),1);h1(i)=h1(i)-mean(x2(f,3));x2(:,3)=x2(:,3)-mean(x2(f,3));
% ;h1(i)=h1(i)-prctile(x2(abs(x2(:,3))<0.1,3),50);R=R2*R1;%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
%plot(x(:,2),x(:,3),'.');hold on;plot(x(f,2),x(f,3),'.')
% pause
  plot3(x2(:,1),x2(:,2),x2(:,3),'.')
%  pause
f=find(abs(x2(:,2))<1.5&(x2(:,1))<4&abs(x2(:,3))<0.05&[1;(abs(diff(x2(:,3))./diff(x2(:,2))))]<0.2);plot(x2(f,2),x2(f,3),'.')

x=x2;x11=sum(x(f,1).^2);x22=sum(x(f,2).^2);x33=sum(x(f,3).^2);x12=sum(x(f,1).*x(f,2));x13=sum(x(f,1).*x(f,3));x23=sum(x(f,2).*x(f,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
n1=[a,b,D];n1=n1/norm(n1);n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
tetax=(atan2(n1(2),n1(3)));tetay=-atan2(n1(1),n1(3));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R3=Rz*Ry*Rx;high=[0,0,h1(i)];x3=(R3*R2*R1*Xdr')'+repmat(high,length(Xdr),1);
R=R3*R2*R1;eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
h1(i)=h1(i)-mean(x3(f,3));x3(:,3)=x3(:,3)-mean(x3(f,3));%eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
plot3(x3(:,1),x3(:,2),x3(:,3),'.')
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
n1=[a,b,D];n1=n1/norm(n1)
% ,n3(i)=mean(x(:,3));n2(i,1:6)=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];%n2(i,:)
plot3(x(:,1),x(:,2),x(:,3),'.')
% pause
% plot(x(:,2),x(:,3),'.')
high=[0,0,h1(i)];x=(R3*R2*R1*Xdr')'+repmat(high,length(Xdr),1);%x=x-repmat(high,length(Xdr),1);
% t=mean(x(abs(x(:,3))<0.06,3));h1(i)=h1(i)-t;high=[0,0,t];
% x=x-repmat(high,length(Xdr),1);
% plot3(x(:,1),x(:,2),x(:,3),'.'),view(280,5),axis([0,4,-2,2,-0.5,1]/1),
plot(x(:,2),x(:,3),'.'),axis([-3,3,-0.5,1]/1),

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
%find yaw and translation
%local map (slam) based on depth data
figure('units','normalized','outerposition',[0 0 1 1])
k=0;x=[];u=0;yawt=[];x=[];%pcloud1=pcloud;
for i=1:length(I1)-2
a=rgb2gray(I1{i});%a=fliplr(rgb2gray(I)')';
subplot(2,2,1),imshow(a),
sc=25;b1=pcloud{i}*1e3/sc;b2=pcloud{i+1}*1e3/sc;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
% if length(b2)==0,k=k+1;else,K(i+1)=k;k=0;
f=find(abs(b2(:,3))>5.5&b2(:,1)>0&b2(:,1)<200);b2=round(b2(f,:));px2=b2(:,1);py2=b2(:,2)+200;pz2=round(b2(:,3)*1);pz2(pz2>50)=50;%pz=round(b1(:,3)*2);
mpc2=zeros(400,300);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find(abs(b1(:,3))>5.5&b1(:,1)>0&b1(:,1)<200);b1=round(b1(f,:));px=b1(:,1);py=b1(:,2);pz=round(b1(:,3)*1);pz(pz>50)=50;
s=[];yaw1=yaw11(i+0)+[-1:0.5:1]*pi/180;for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);yawt(1,i)=yaw1(f);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=pz;
% m=mpc1;m(:,:,3)=mpc2;imshow(m),pause(1)%imshow(mpc1)
transy=-4:1:4;trans=1:9;k1=70;m2=mpc2(10:end-10,k1:end-10);s=[];
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
m=m1;m(:,:,3)=m2;%subplot(2,2,[2,4]),imshow(m);pause(1)
subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),
t=double((m1.*m2)>0);t2=mpc2*0;t2(10:end-10,k1:end-10)=t;
c=conv2(t,ones(4,4)/16,'same');c=(c>0.3);t2=mpc2*0;t2(10:end-10,k1:end-10)=c;
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
