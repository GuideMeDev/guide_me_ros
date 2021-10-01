lmap=zeros(800,800);lmap(:,201:250)=1;lmap(:,end-250:end-200)=1;
c=conv2(lmap,ones(40)/1600,'same')>0;lmap=(lmap+0.1*c);
% lmap(200:250,400:550)=1;lmap(400:450,200:450)=1;imshow(lmap)
% lmap(200:250,200:350)=1;lmap(400:450,400:550)=1;imshow(lmap)
lmap1=lmap*0;lmap1(200:250,200:350)=1;lmap1(400:450,300:450)=1;lmap1(250:300,500:550)=1;
% c=conv2(lmap1,ones(40)/1600,'same')>0;lmap=(lmap+lmap1+0.1*c);
lmap=(lmap+lmap1);
imshow(lmap)
%%
tetazT=zeros(3,200);xy1=[0,2];v1=0;v0=pi/2-100*pi/180;xy0=[-40,30];dwi=15;dlen=80;M=[];sc1=10;sc2=1;sc3=5*pi/180;sc4=2;
writerObj = VideoWriter('myVideo.avi');writerObj.FrameRate = 5;open(writerObj);
for i=1:322
if i==1;M{i}=lmap;
m=M{i};[py,px]=find(m>0);f1=find(m>0);px=px-size(lmap,2)/2;px=px-xy0(1);py=py-xy0(2);%plot(px,py,'.')
tetazT(1,i)=v0;tetaz=v0;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
t1=round((Rz*[px';py'])');f=abs(t1(:,2)-size(lmap,2)/2)<size(lmap,2)/2&abs(t1(:,1))<size(lmap,1)/2;t1=t1(f,:);px1=t1(:,1)+size(lmap,2)/2;py1=t1(:,2);%plot(px1,py1,'.')
mpc1=zeros(size(lmap));mpc1((px1-1).*size(mpc1,1)+py1)=m(f1(f));M{i}=mpc1;
else
m=M{i-1};[py,px]=find(m>0);f1=find(m>0);px=px-size(lmap,2)/2;px=px-xy1(1);py=py-xy1(2);%plot(px,py,'.')
tetazT(1,i)=-v1;tetaz=-v1;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
t1=round((Rz*[px';py'])');f=abs(t1(:,2)-size(lmap,2)/2)<size(lmap,2)/2&abs(t1(:,1))<size(lmap,1)/2;t1=t1(f,:);px1=t1(:,1)+size(lmap,2)/2;py1=t1(:,2);plot(px1,py1,'.')
mpc1=zeros(size(lmap));mpc1((px1-1).*size(mpc1,1)+py1)=m(f1(f));
tt=mpc1*0;mpc2=tt;mpc2(1:dlen,size(lmap,1)/2-dwi:size(lmap,1)/2+dwi)=1;mpc1(:,:,3)=mpc2;M{i}=mpc1;
imshow(mpc1);v1=0;t1=mpc1(:,:,1).*mpc2;
pause(.05)
% if sum(sum(t1))/sum(sum(t1>0))==1,
if max(max(t1))>0.9&&sum(sum(t1))>5
    mpc2=tt;mpc2(1:dlen*sc2,size(lmap,1)/2-dwi*sc1:size(lmap,1)/2-dwi)=1;
    mpc2(1:dlen*sc2,size(lmap,1)/2+dwi:size(lmap,1)/2+dwi*sc1)=0.5;mpc1(:,:,2)=mpc2;
    imshow(mpc1);pause(.2)
    t=sum(((mpc1(:,:,1)>0.1).*(mpc1(:,:,2))))>0;mpc2(:,t)=0;mpc1(:,:,2)=mpc2;
    imshow(mpc1);pause(.2)
%     t=sum(mpc1(:,:,2))>0;f=find(t>0);f=abs(f-size(lmap,2)/2);t=find(f==max(f));t=f(t);
    t=sum(mpc1(:,:,2))>0;f=find(t>0);f1=(f-size(lmap,2)/2);f1a=f1(f1<0);f1b=f1(f1>0);F{1}=f1a+size(lmap,2)/2;F{2}=f1b+size(lmap,2)/2;
    j=[length(f1a),length(f1b)]==min([length(f1a),length(f1b)]);mpc1(:,F{sum(j.*[1,2])},2)=0;
    imshow(mpc1);pause(.2)
    t=sum(mpc1(:,:,2))>0;f=find(t>0);f1=(f-size(lmap,2)/2);
    f2=find(abs(f1)==min(abs(f1)));f2=f1(f2);
    if f2<0;f1(f1>0)=0;k=-1;else f1(f1<0)=0;k=1;end
    t=find(abs(f1)==max(abs(f1)));t=f(t);
    v2=pi/2-atan2(dlen*sc2,t);v1=-sc3;if k<0,v1=sc3;end,tetazT(2,i)=1;
    
else
    if i>5&&max(max(t1))==0.1&&sum(tetazT(2,i-5:i))==0
    mpc2=tt;mpc2(1:dlen*sc2,size(lmap,1)/2-dwi*sc4:size(lmap,1)/2-dwi)=1;
    mpc2(1:dlen*sc2,size(lmap,1)/2+dwi:size(lmap,1)/2+dwi*sc4)=0.5;
    mpc1(:,:,2)=mpc2;
    imshow(mpc1);pause(.1)
    t=sum(((mpc1(:,:,1)==0.1).*(mpc1(:,:,2)))>0);s1=sum(t(1:size(lmap,2)/2));s2=sum(t(1+size(lmap,2)/2:end));
    k=0;if s1<s2,k=size(lmap,2)/2;end,mpc2(:,1+k:size(lmap,2)/2+k)=0;
    mpc1(:,:,2)=mpc2;
    imshow(mpc1);pause(.1)
    
    t=sum(mpc1(:,:,2))>0;f=find(t>0)-size(lmap,2)/2;f1=length(f(f>0));f2=length(f(f<0));
    if abs(f1-f2)>10&&(s1/s2>1.5||s2/s1>1.5),
    t=sum(((mpc1(:,:,2))));s1=sum(t(1:size(lmap,2)/2));s2=sum(t(1+size(lmap,2)/2:end));
    k=0;k1=-1;if s1-s2>0,k=size(lmap,2)/2;k1=1;end,mpc2(:,1+k:size(lmap,2)/2+k)=0;
    mpc1(:,:,2)=mpc2;
    imshow(mpc1);pause(.1)
    v1=-sc3;if k1>0,v1=sc3;end
    end
   
else
    t=cumsum(tetazT(1,:));if i>35&&abs(t(i))>10*pi/180&&sum(tetazT(2,i-35:i))==0&&sum(tetazT(3,i-3:i))==0,
        v1=sign(t(i))*sc3;tetazT(3,i)=1;
%         pause
    end
    end
end 
f=getframe(gcf);
writeVideo(writerObj,f);
end
end
close(writerObj);

%%
%find yaw and translation
%local map (slam) based on depth data
figure('units','normalized','outerposition',[0 0 1 1])
writerObj = VideoWriter('myVideo.avi');writerObj.FrameRate = 5;open(writerObj);

k=0;x=[];u=0;yawt=[];dlen=57;dwi=10; sc2=1.0;sc1=4;%pcloud1=pcloud;
for i=15:length(I)-6
a=rgb2gray(I{i});%a=fliplr(rgb2gray(I)')';
subplot(2,2,1),imshow(a),
sc=25;b1=pcloud1{i}*1e3/sc;b2=pcloud1{i+1}*1e3/sc;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
% if length(b2)==0,k=k+1;else,K(i+1)=k;k=0;
f=find(abs(b2(:,3))>12.5&b2(:,1)>0&b2(:,1)<300);
b2=round(b2(f,:));px2=b2(:,1);py2=b2(:,2)+200;
mpc2=zeros(400,300);mpc2((px2-1).*size(mpc2,1)+py2)=1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
f=find(abs(b1(:,3))>5.5&b1(:,1)>0&b1(:,1)<300);b1=round(b1(f,:));px=b1(:,1);py=b1(:,2);
s=[];yaw1=yaw11(i-1)+[-0.5,0,0.5]*pi/180;for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);yawt(1,i)=yaw1(f);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;
% m=mpc1;m(:,:,3)=mpc2;imshow(m),pause(1)%imshow(mpc1)
transy=-3:1:3;trans=0:6;k1=70;m2=mpc2(10:end-10,k1:end-10);s=[];
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
t=m1.*m2;t2=mpc2*0;t2(10:end-10,k1:end-10)=t;
c=conv2(t,ones(4,4)/16,'same');c=(c>0.3);t2=mpc2*0;t2(10:end-10,k1:end-10)=c;
[py,px]=find(t2>0);px=px;py=py-200;px1=px+yawt(2,i);py1=py+yawt(3,i);
tetaz=-yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=(Rz*[px1';py1'])';
x=[x;t1];tetaz=yawt(1,i);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x=[(Rz*[x]')'];x(:,1)=x(:,1)-yawt(2,i);x(:,2)=x(:,2)-yawt(3,i);%t2=mpc2*0;t1(10-yawt(3,i):end-10-yawt(3,i),k1-yawt(2,i):end-10-yawt(2,i))=t;t2(10:end-10,k1:end-10)=t;
x=[x;[px';py']'];tetaz=87*pi/180;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x1=round([(Rz*[x]')']);
f=(x1(:,2)>0)&(abs(x1(:,1))<200);px=x1(f,1)+200;py=x1(f,2);mpc=zeros(300,400);mpc((px-1).*size(mpc,1)+py)=1;
mpc(1:dlen,size(mpc,2)/2-dwi:size(mpc,2)/2+dwi,3)=1;
subplot(2,2,[2,4]),imshow(imrotate(fliplr(mpc),180))
tt=mpc(:,:,1)*0;t1=mpc(:,:,1).*mpc(:,:,3);
pause(.05)
% if sum(sum(t1))/sum(sum(t1>0))==1,
if max(max(t1))>0.9&&sum(sum(t1))>50
    mpc2=tt;mpc2(1:dlen*sc2,size(mpc,2)/2-dwi*sc1:size(mpc,2)/2-dwi)=1;
    mpc2(1:dlen*sc2,size(mpc,2)/2+dwi:size(mpc,2)/2+dwi*sc1)=0.5;mpc(:,:,2)=mpc2;
%     imshow(mpc);pause(.2)
    t=sum(((mpc(:,:,1)>0.1).*(mpc(:,:,2))))>0;mpc2(:,t)=0;mpc(:,:,2)=mpc2;
    imshow(imrotate(fliplr(mpc),180))
pause(.2)
end
% subplot(2,2,[2,4]),plot(x1(:,1),x1(:,2),'.'),axis([-120,120,-80,300])
% hold on
% plot([-300,100,300,-300,-300]/sc,[0,0,2000,2000,0]/sc)
hold off
pause(.1)
f=getframe(gcf);
writeVideo(writerObj,f);
end
close(writerObj);

%%
% s=[];yaw1=eulrec2(i,3)+[-0.005,0,0.005];for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;