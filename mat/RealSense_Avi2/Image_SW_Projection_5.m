function [SWpix,repvf,l,m]=Image_SW_Projection_5(a1,eul,transy,f,sopix,px0,py0,px,pcloud,yaw)
% function [SWpix,repvf,l,m,mpc]=Image_SW_Projection_5(a1,eul,transy,f,sopix,px0,py0,px,pcloud,yaw)
% f=2;sopix=2*3e-3; 
high=eul(4)*1e3;vfc=[0,0,high];vf=[0,0,-f;0,0,0;high*sopix,0,0];
if isempty(px),[fy,fx]=find(a1>0);indf=a1>0;indf=a1(indf);
    else fy=px(:,2);fx=px(:,1);indf=a1>=0;indf=a1(indf);
end    
% [fy,fx]=find(a1>=0);indf=a1>=0;indf=a1(indf);
% ind=find(fy<py0*1.1);fy=fy(ind);fx=fx(ind);%fy(end+1)=b3(i,2);fx(end+1)=b3(i,1);
fy=fy-(py0+0.5);fx=fx-(px0+0.5);
fy=-fy;%fx=-fx;
ind=find(fy<py0*0.2);fy=fy(ind);fx=fx(ind);%fy(end+1)=b3(i,2);fx(end+1)=b3(i,1);
indf=indf(ind); 
%pix=[-sopix*fy';sopix*fx';zeros(1,length(fx))]'; 
pix=[-sopix*fy';sopix*fx';zeros(1,length(fx))]';%plot(pix(:,1),pix(:,2),'.')
% tetax=eul(1);tetay=eul(2);tetaz=-0*pi/180;%eul(3);
tetax=eul(1);tetay=eul(2);tetaz=0;
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
tetay=-90*pi/180;Ry90=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
transx=eul(5);
vpix=(Rz*Ry*Rx*Ry90*pix')+repmat((vfc+[transx,transy,0])',1,size(pix,1));
rvf=(Rz*Ry*Rx*Ry90*vf')+[vfc+[transx,transy,0];vfc+[transx,transy,0];vfc+[transx,transy,0]]';repvf=repmat(rvf(:,1)',length(vpix),1)';
% plot3(vpix(1,:),vpix(2,:),vpix(3,:),'.');hold on;plot3(repvf(1,:),repvf(2,:),repvf(3,:),'.');
% plot3(rvf(1,:),rvf(2,:),rvf(3,:),'.-');
% x=[0,0,0;1,0,0;0,0,0;0,1,0;0,0,0;0,0,1];x=t*x'; 
t2=repmat([0,0,1],length(vpix),1);   
t3=dot(repvf,t2');l=repvf-vpix;      
l=l./repmat(sqrt(sum(l.^2)),3,1);dcoef=-t3./dot(l,t2');  
SWpix=repvf'+repmat(dcoef',1,3).*l';%SWpix=(Rz1*SWpix')'; 
%   plot3(SWpix(:,1),SWpix(:,2),SWpix(:,3),'.');axis([1000,8000,-3000,3000,0,1100])
% t=abs(pcloud(:,3));t(t<60)=0;t=5*t./max(t);t=round(t*255);t(t<60)=0;t(t>255)=255;
tetaz=(-0-0)*pi/180;
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];pcloud=(Rz*pcloud')';        
t=(pcloud(:,3));t(abs(t)<120)=0;t=1*t./max(t);t=round(t*255);t(t>255)=255;t(t==0)=-10;%t(t==100)=-10;
% t=(pcloud(:,3))-eul(4);t(t<40)=0;t=round(t*255);t(t<=100)=-10;t(t>255)=255;
% plot3(SWpix(:,1)*1.0,SWpix(:,2)*1.0,SWpix(:,3),'.');hold on;plot3(pcloud(:,1)-0,pcloud(:,2)-0,t,'.');axis([1000,8000,-3000,3000,-100,1100]),view(270,-90),
% hold off  
% x=pcloud;hold off,plot(x(:,1),x(:,3),'.')     
sc1=25; px=round(SWpix(:,1)/sc1);py=round(SWpix(:,2)/sc1);%plot(px,py,'.')%px=px-min(px)+1;py=py-round(mean(py))+1;plot(px,py,'.')
px=round(SWpix(:,1)/sc1);py=round(SWpix(:,2)/sc1);py(px>500)=1;px(px>500)=1;%plot(px,py,'.')
f1=find(px<1|abs(py)>=200|px>=300);py=py+200;px(f1)=1;py(f1)=1;%plot(px,py,'.')
m=zeros(400,300);m((px-1).*size(m,1)+py)=double(indf)/255;%imshow((m)/1)  
                                                    
px=round((pcloud(:,1))/sc1)-0;py=round((pcloud(:,2))/sc1);pz=round((pcloud(:,3))/sc1);pz(px>500)=1;py(px>500)=1;px(px>500)=1;%plot(px,py,'.')
f1=find(px<1|abs(py)>=200|px>=300);py=py+200;px(f1)=1;py(f1)=1;pz(f1)=1;t=pz>200/sc1;tfloor=abs(pz)<80/sc1;%plot(px,py,'.')
mpc=zeros(400,300);mpc((px-1).*size(mpc,1)+py)=double(t)/255;c=double(mpc>0);c=255*conv2(mpc,ones(3,3)/9,'same');%imshow(c>0)
mpc=zeros(400,300);mpc((px-1).*size(mpc,1)+py)=double(tfloor)/255;cfloor=double(mpc>0);cfloor=255*conv2(mpc,ones(3,3)/9,'same');%imshow(c>0)
% c1=c*0;c1(c<0.05&c>0.001)=0.1;c1(c>0.06)=1;imshow(c1*255)
m(:,:,2)=255*(double(cfloor>0.07));m(:,:,3)=255*(double(c>0.0));%imshow(m)


% f1=find(px<9|py>150|px>200);px(f1)=1;py(f1)=1;
% m=zeros(round([max(py),max(px)]));m((px-1).*size(m,1)+py)=double(indf)/255;
% m=zeros(round([max(py),max(px)]));m((px-1).*size(m,1)+py)=double(indf)/255;
% 1
 

% i1=ones(500,300);high=740;vfc=[0,0,high];[fy,fx]=find(i1==1);fy=fy-250+500;fx=fx-150;
% sopix=3e-3;%pix=[sopix*fy';sopix*fx';zeros(1,length(fx))]';
% tetax=0*pi/180;Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
% tetay=-77*pi/180;Ry=[cos(tetay),0,-sin(tetay);0,1,0;sin(tetay),0,cos(tetay)];
% % f=4.45;pix=[sopix*fx';sopix*fy';-f*ones(1,length(fx))]';%vpix1=(pix-repmat(pix0,size(pix,1),1))*sc;
% f=4.15;vf=[0,0,-f;0,0,0;high*sopix,0,0];%pix(end+1,:)=vf(3,:);
% %vpix1=(pix-repmat(pix0,size(pix,1),1))*sc;
% SWpix1=[];K=[];i1=60;i2=80;
% for i=i1:i2%n/2
% a1=Im(:,:,i)';[fy,fx]=find(a1>=0);indf=a1>=0;indf=a1(indf);
% ind=find(fy<450);fy=fy(ind);fx=fx(ind);fy(end+1)=b3(i,2);fx(end+1)=b3(i,1);
% indf=indf(ind);fy=fy-(size(a1,1)/2+0.5);fx=fx-(size(a1,2)/2+0.5);
% % pix=[-sopix*fy';sopix*fx';zeros(1,length(fx))]';
% pix=[sopix*fy';sopix*fx';zeros(1,length(fx))]';
% t=Ft(:,:,i+1);
% tetaz=atan(t(1,2)/t(1,1));Rz1=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
% tetay=-asin(t(1,3));Ry1=[cos(tetay),0,-sin(tetay);0,1,0;sin(tetay),0,cos(tetay)];
% tetax=atan(t(3,2)/t(3,3));Rx1=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
% t=Rz1*Ry1*Rx1;% Rz=[cos(tetaz),sin(tetaz),0;-sin(tetaz),cos(tetaz),0;0,0,1];%t=Rz*t;
% vpix=(t*Ry*Rx*pix')+repmat(vfc',1,size(pix,1));
% rvf=(t*Ry*Rx*vf')+[vfc;vfc;vfc]';repvf=repmat(rvf(:,1)',length(vpix),1);
% x=[0,0,0;1,0,0;0,0,0;0,1,0;0,0,0;0,0,1];x=t*x';
% t2=repmat([0,0,1],length(vpix),1);
% t3=dot(repvf',t2');l=vpix-repvf';
% l=l./repmat(sqrt(sum(l.^2)),3,1);dcoef=-t3./dot(l,t2');
% SWpix=repvf+repmat(dcoef',1,3).*l';SWpix(:,3)=0;
% SWpix1(1+size(SWpix1,1):size(SWpix1,1)+length(SWpix),:)=[SWpix];SWpix2=(Rz*SWpix1')';SWpix=(1*SWpix')';
% % plot3(x(1,:),x(2,:),x(3,:),'.-');axis([-2,2,-1,2,-2,2])
% hold off
% % plot3(vpix(1,:),vpix(2,:),vpix(3,:),'.-');%
% % hold on
% % plot3([rvf(1,:)],[rvf(2,:)],[rvf(3,:)],'o-r');%
% % % axis([-2,5,-2,2,720,760])
% % range=1:40:length(SWpix);
% % plot3(SWpix(end,1),SWpix(end,2),10,'om','markersize',23)
% K(i,1:2)=[SWpix(end,1),SWpix(end,2)];
% % plot3(SWpix2(range,1),SWpix2(range,2),SWpix2(range,3),'.')
