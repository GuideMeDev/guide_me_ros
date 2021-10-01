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
% s=[];yaw1=eulrec2(i,3)+[-0.005,0,0.005];for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;