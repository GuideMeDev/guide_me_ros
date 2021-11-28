
% output: yawt,tx,minter_plus,minter_minus
function [yawt,tx_curr,minter_plus,minter_minus] = scan_match(pcloud_curr,pcloud_next,pRGB1_curr,pRGB1_next,yaw_curr,yaw_next,dxIMU_i,dyIMU_i,tx_prev)
sc=20;weg_obst=5;weg_tex=3;thz0=80/sc;thz0_mean=100/sc;dxmin=1600/sc;dxmax=4000/sc;sizemx=5000/sc;sizemy=5200/sc;%k=12;d=[];for ki=-5:k;
rangex_mpc2=dxmin+1:dxmax;rangex_mpc1=dxmin+400/sc+1:dxmax-400/sc;rangey_mpc1=600/sc+1:sizemy-600/sc;
kkx = 6;kky = 6;
b1=pcloud_curr*1e3/sc;
b2=pcloud_next*1e3/sc;
trgb1=-weg_tex*pRGB1_curr;
trgb2=-weg_tex*pRGB1_next;%yaw=(-0+[-3.0,-2.0,-1.0,0,1.0,2.0,3.0])*pi/180;
%find d-frame and t-frame for sample (i+1) and same for frame (i) but after
%yaw rotation 
yaw1=yaw_curr-yaw_next;
tetaz0=yaw1;
[mpc1,mpc2,tmpc1,tmpc2,b1a,b1b]=find_dframe_tframe(b1,b2,trgb1,trgb2,dxmin,sizemx,sizemy,thz0,weg_obst,yaw1);
% imagesc(mpc2)
%find translation of frame (i) to match frame (i+1)
% tic
m2=mpc2(:,rangex_mpc2)+tmpc2(:,rangex_mpc2);m1=mpc1(rangey_mpc1,rangex_mpc1)+tmpc1(rangey_mpc1,rangex_mpc1);
tx1=xcross2_custom(m1,m2,dyIMU_i,dxIMU_i,kkx,kky);tx_curr=tx1;
% toc
if tx_prev==-1;tx_prev=tx_curr;end
tx_curr = 0.8*tx_curr+0.2*tx_prev;
%find no floor
%find segments below ground level (mpc2curbe) for frame (i+1)
[mpc2curbe,mpc2nofloor,mpc2floor]=choose_mean_range2(b2,thz0_mean,dxmin*1.5,dxmax,sizemx,sizemy);
%find a second rotation angle (tetaz1) to match frames (i) and (i+1)
rtb1a=b1a;rtb1a(:,1)=rtb1a(:,1)-tx_curr(2);rtb1a(:,2)=rtb1a(:,2)-tx_curr(1);b1b(:,1:2)=rtb1a(:,1:2);
yaw1=[-1.2:0.4:1.2]*pi/180;
[rtmpc1,tetaz1,rtb1b]=correct_reg_angle2(rtb1a,b1b,mpc2,yaw1,sizemx,sizemy);
%find segments below ground level (mpc1curbe) for frame (i) after rotation
%and translation (rt)
[mpc1curbe,mpc1nofloor,mpc1floor]=choose_mean_range2(rtb1b,thz0_mean,dxmin*1.5,dxmax,sizemx,sizemy);
% imagesc(mpc1curbe.*mpc2curbe)

%update tetaz and save translation and rotation values to the matrix yawt
tetaz=tetaz0+tetaz1+0;
yawt=[tetaz,tx_curr,tetaz0];
%find dframes (mpc2_plus and mpc1_plus) above ground level for frames (i)
%and (i+1) (interception)
x=b2;f=(x(:,3)>thz0*2 & x(:,1)>dxmin&x(:,1)<dxmax & abs(x(:,2))<sizemy/2);ba=round(x(f,:));px2a=ba(:,1);py2a=ba(:,2)+sizemy/2;
mpc=zeros(sizemy,sizemx);
mpc((px2a-1).*size(mpc,1)+py2a)=x(f,3);
c=conv2(mpc,ones(3,3)/9,'same');
mpc2_plus=(c>2);%
x=rtb1b;f=(x(:,3)>thz0*2&x(:,1)>dxmin&x(:,1)<dxmax&abs(x(:,2))<sizemy/2);ba=round(x(f,:));px2a=ba(:,1);py2a=ba(:,2)+sizemy/2;
mpc=zeros(sizemy,sizemx);mpc((px2a-1).*size(mpc,1)+py2a)=x(f,3);c=conv2(mpc,ones(3,3)/9,'same');mpc1_plus=(c>2);%
minter_plus=round((mpc2.*mpc1_plus)>0);minter_minus=round(mpc1curbe.*mpc2curbe);
