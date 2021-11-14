addpath('./utils')
figure('units','normalized','outerposition',[0 0 0.5 1])
figure('units','normalized','outerposition',[0.5 0 0.5 1])

sc=20;S=[];tx=[];tr=[];tc=[];weg_obst=5;weg_tex=3;thz0=80/sc;thz0_mean=100/sc;dxmin=1600/sc;dxmax=4000/sc;sizemx=5000/sc;sizemy=5200/sc;%k=12;d=[];for ki=-5:k;
dxIMU=round(dxinternal(4:end-1)*1e3/sc*1.0);dyIMU=round(dyinternal(4:end-1)*1e3/sc);
rangex_mpc2=dxmin+1:dxmax;rangex_mpc1=dxmin+400/sc+1:dxmax-400/sc;rangey_mpc1=600/sc+1:sizemy-600/sc;
xplus=[];xminus=[];yawt=[];x=[];dlen=round(2800/sc);dwi=round(250/sc);k3=5000/sc;kkx=6;kky=6;
% for j=2:2:10 

tic
for i=1:235 % number of frames
    i
%Improtant: pcloud = plane_fit(XYZ,eul) -> Activating plane_fit module with pcloud
% and euler angles
tx_prev=-1;if i>1;tx_prev=tx(i-1,:);end
% Calling for scan_match module
[yaw_t,tx_curr,minter_plus,minter_minus] = scan_match(pcloud{i},pcloud{i+1},pRGB1{1,i},pRGB1{1,i+1},yaw(i),yaw(i+1),dxIMU(i),dyIMU(i),tx_prev);
yawt(i,:)=yaw_t;tx(i,:)=tx_curr;
%show dframes above and below ground level
% minter_plus,minter_minus

figure(1)
subplot(2,2,1)
a=I{i};a(:,size(a,2)/2-5:size(a,2)/2+5,:)=0;imshow(a)
subplot(2,2,3)
%find convolved minter_plus (the filtered D-frame), by using low pass filter on minter_plus
m=minter_plus;m(:,:,3)=minter_minus;
imshow(m)
[xplus,xminus] = SLAM(yawt(i,:),minter_plus,minter_minus,xplus,xminus);

f = xplus(:,1) > -50;
xplus = xplus(f,:);
f = xminus(:,1) > -50;
xminus = xminus(f,:);
%only for illustration in figures, find x1 (x, rotated by 90 degrees and scaled) and x1curbe (xcurbe, rotated by 90 degrees and scaled)
tetaz=90*pi/180-1*pi/180;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
x1plus=[(Rz*[xplus]')']/1e3*sc;x1minus=[(Rz*[xminus]')']/1e3*sc;
%adding frames (i) plus and minus to the SLAM of xplus and xminus
toc
figure(1)
subplot(2,2,[2,4])
plot(x1plus(:,1),x1plus(:,2),'.b'),hold on,plot(x1minus(:,1),x1minus(:,2),'.','color',[1 .5 0]),axis([-120,120,-80,250]/1e3*25)
hold on
plot([0,0],[0,6])
% plot([-20,20,20,-20,-20]/1e3,[0,0,600,600,0]/1e3)
plot([-50,0,50]/1e3,[210,350,210]/1e3,'linewidth',5,'color','r')
% pause
% plot([-200,200,200,-200,-200]/sc,[0,0,200,200,0]/sc)
plot([0]/sc,[0]/sc,'o','markersize',8,'linewidth',9,'color','g')
hold off
title('Perception')
 
 Control(xplus,xminus)
 pause(0.1)
 
end
toc
diff_yaw=yawt(:,1)-yawt(:,4);
j=1;tr=tx(:,1)-dyIMU(1:i)';tc=tx(:,2)-dxIMU(1:i)';S(j,1:2)=[sum(abs(tr)==kky),sum(abs(tc)==kkx)];
figure
subplot(2,2,1)
plot(dxIMU);hold on;plot(tx,'o-');plot(dyIMU);
subplot(2,2,2)
plot(S)
subplot(2,2,3)
plot(yawt(:,1)-yawt(:,4),'.')
% Performance Metric
fit_IMU_img_y=[mean(tr),std(tr)]; % in pixels
fit_IMU_img_x=[mean(tc),std(tc)];
fit_IMU_img_yaw=[mean(diff_yaw),std(diff_yaw)];

% end
