
cd('C:\Users\ùùù\Downloads\2021-06-23-18-39-50')
%%
bag= rosbag('2021-06-23-18-39-50.bag');
sel= select(bag,'Topic','/camera/color/camera_info');
sel= select(bag,'Topic','/camera/color/image_raw/compressed');
sel.StartTime
bagselect1 = select(bag,'Time',[sel.StartTime+30,sel.StartTime+302], 'Topic','/camera/color/image_raw/compressed');
msgs = readMessages(bagselect1,[1:200]);
%%
figure
pcloud=[];bagselect1 = select(bag,'Time',[sel.StartTime+30,sel.StartTime+92], 'Topic','/camera/depth/color/points');
for i=1:100%:100
msgs1 = readMessages(bagselect1,[i]);
Xd = readXYZ(msgs1{1,1});
range=1:5:length(Xd);
% plot3(Xd(range,3),-Xd(range,1),Xd(range,2),'.')
% plot3(Xd(range,3),-Xd(range,1),-Xd(range,2),'.')
% axis([-10,10,-10,10,-10,10]/2)

Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
% plot3(Xdr(:,1),Xdr(:,2),Xdr(:,3),'.')
% eul=[roll(i),pitch(i),0]*pi/180;
% eul=[4.7+n2(i-1,5),13.9-n2(i-1,4),0]*pi/180;
if i==1,h1(i)=1.083;eul=[4.7,13.9,0]*pi/180;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
else h1(i)=h1(i-1);tetax=eul(i-1,1);tetay=eul(i-1,2);tetaz=eul(i-1,3);
end
% tetax=interpeulZYX(i,1);tetay=interpeulZYX(i,2);tetaz=interpeulZYX(i,3);
% Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
% Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
% Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
% high=[0,0,1.083-n3(i)];x=(Rz*Ry*Rx*Xdr')'+repmat(high,length(Xdr),1);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
plot3(x1(:,1),x1(:,2),x1(:,3),'.')
% plot3(x(:,1),x(:,2),x(:,3),'.')
x=x1;f=find(abs(x(:,2))<1&abs(x(:,1))<4&abs(x(:,3))<0.12);
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
plot3(x(:,1),x(:,2),x(:,3),'.'),view(280,5),axis([0,4,-2,2,-0.5,1]/1),
% plot(x(:,2),x(:,3),'.'),axis([-3,3,-0.5,1]/1),

pause(.1)
pcloud{i}=x;
end
plot(eul*180/pi)
roll=eul(:,1)*180/pi;pitch=eul(:,2)*180/pi;
pitch(2)=14.5;
% pitch=13.9-(n2(1:end-1,4))-(n2(2:end,4));pitch=[13.9;pitch];plot(pitch,'.-')
% hold on
% roll=4.7+(n2(1:end-1,5))+(n2(2:end,5));roll=[4.7;roll];plot(roll,'.-')
% pcloud=x;
%%
cd('C:\Users\ùùù\Downloads\2021-06-23-18-39-50')
%%
figure('units','normalized','outerposition',[0 0 1 1])
bagselectCAM = select(bag,'Time',[sel.StartTime+30,sel.StartTime+302], 'Topic','/camera/color/image_raw/compressed');
msgs2 = readMessages(bagselectCAM);
bagselectCAMP = select(bag, 'Topic','/camera/color/image_raw/compressed/parameter_descriptions');
msgsCAMP = readMessages(bagselectCAMP);
for i=1:100%:10:size(msgs2,1)
msgs1 = readMessages(bagselect1,[i]);Xd = readXYZ(msgs1{1,1});Xdr=[Xd(:,3)';-Xd(:,1)';-Xd(:,2)']';Xdr(Xdr(:,1)>5,:)=0;range=1:5:length(Xd);
subplot(2,1,1)
I{i} = readImage(msgs2{i,1});
imshow(I{i})
subplot(2,1,2)
plot3(Xdr(range,1),Xdr(range,2),Xdr(range,3),'.'),view(270,10),axis([-1,6,-5,5,-1,2]/1),
pause(0.1)
end
%%
%find yaw and translation
figure('units','normalized','outerposition',[0 0 1 1])
x=[];u=0;yawt=[];x=[];
for i=1:99%1:5
a=rgb2gray(I{i});%a=fliplr(rgb2gray(I)')';
subplot(2,2,1),imshow(a),
sc=25;b1=pcloud{i}*1e3/sc;b2=pcloud{i+1}*1e3/sc;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
f=find(b2(:,3)>5&b2(:,1)>0);b2=round(b2(f,:));px=b2(:,1);py=b2(:,2)+200;
mpc2=zeros(400,250);mpc2((px-1).*size(mpc2,1)+py)=1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find(b1(:,3)>5&b1(:,1)>0);b1=round(b1(f,:));px=b1(:,1);py=b1(:,2);
s=[];yaw=[-4:0.3:4]*pi/180;for j=1:length(yaw)
tetaz=yaw(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,250);mpc1((px1-1).*size(mpc1,1)+py1)=1;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);yawt(1,i)=yaw(f);tetaz=yaw(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,250);mpc1((px1-1).*size(mpc1,1)+py1)=1;
% m=mpc1;m(:,:,3)=mpc2;imshow(m),pause(1)%imshow(mpc1)
transy=-2:1:2;trans=-1:7;k1=60;m2=mpc2(10:end-10,k1:end-10);s=[];
for j=1:length(trans)
for k=1:length(transy)
m1=mpc1(10+transy(k):end-10+transy(k),k1+trans(j):end-10+trans(j));s(k,j)=sum(sum((m2-m1).^2));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);
m1=mpc1(10+transy(fy):end-10+transy(fy),k1+trans(fx):end-10+trans(fx));
yawt(2,i)=trans(fx);yawt(3,i)=transy(fy);
subplot(2,2,3),imshow(imrotate(fliplr(m),-90)),
% pause(.1)%imshow(mpc1)
% i,pause
m=m1;m(:,:,3)=m2;t=m1.*m2;t2=mpc2*0;t2(10:end-10,k1:end-10)=t;
c=conv2(t,ones(4,4)/16,'same');c=(c>0.3);t2=mpc2*0;t2(10:end-10,k1:end-10)=c;
[py,px]=find(t2>0);px=px;py=py-200;px1=px+yawt(2,i);py1=py+yawt(3,i);
tetaz=-yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=(Rz*[px1';py1'])';
x=[x;t1];tetaz=yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x=[(Rz*[x]')'];x(:,1)=x(:,1)-yawt(2,i);x(:,2)=x(:,2)-yawt(3,i);%t2=mpc2*0;t1(10-yawt(3,i):end-10-yawt(3,i),k1-yawt(2,i):end-10-yawt(2,i))=t;t2(10:end-10,k1:end-10)=t;
x=[x;[px';py']'];tetaz=90*pi/180;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x1=[(Rz*[x]')'];
subplot(2,2,[2,4]),plot(x1(:,1),x1(:,2),'.'),axis([-120,120,-80,240])
hold on
plot([-300,100,300,-300,-300]/sc,[0,0,2000,2000,0]/sc)
hold off
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