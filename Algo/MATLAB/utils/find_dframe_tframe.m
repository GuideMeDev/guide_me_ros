function [mpc1,mpc2,tmpc1,tmpc2,b1a,b1b]=find_dframe_tframe(b1,b2,trgb1,trgb2,dxmin,sizemx,sizemy,thz0,weg_obst,yaw1)
f=find(b2(:,1)>dxmin+1&b2(:,1)<sizemx-10&abs(b2(:,2))<sizemy-10);b2=round(b2(f,:));px2=b2(:,1);py2=b2(:,2)+sizemy/2;
pz2=(b2(:,3));
pz2(pz2<thz0)=-1;
pz2(pz2>thz0)=weg_obst;
pz2(abs(b2(:,3))<thz0/1)=0;%pz=round(b1(:,3)*2);
mpc2=zeros(sizemy,sizemx);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
mpc2(((px2+1)-1).*size(mpc2,1)+py2)=pz2;
mpc2(((px2-1)-1).*size(mpc2,1)+py2)=pz2;
mpc2((px2-1).*size(mpc2,1)+(py2+1))=pz2;
mpc2((px2-1).*size(mpc2,1)+(py2-1))=pz2;
%find t-frame
f=find(b2(:,1)>dxmin&b2(:,1)<sizemx/4*3-10&abs(b2(:,2))<sizemy-10&abs(b2(:,3))<thz0);tb2=round(b2(f,:));px2=tb2(:,1);py2=tb2(:,2)+sizemy/2;pz2=trgb2(f,2);
tmpc2=zeros(sizemy,sizemx);tmpc2((px2-1).*size(tmpc2,1)+py2)=pz2;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
%find d-frame
f=find(b1(:,1)>dxmin&b1(:,1)<sizemx-10&abs(b1(:,2))<sizemy-10);b1=(b1(f,:));px1=b1(:,1);py1=b1(:,2);
pz1=(b1(:,3));pz1(pz1<thz0)=-1;pz1(pz1>thz0)=weg_obst;pz1(abs(b1(:,3))<thz0/1)=0;%pz=round(b1(:,3)*2);
% yaw1=yaw(i)-yaw(i+1);
tetaz=yaw1(1);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px1';py1'])');
b1a=b1;b1a(:,1:2)=t1;b1a(:,3)=pz1;px1=t1(:,1);py1=t1(:,2)+sizemy/2;b1b=b1a;b1b(:,3)=b1(:,3);
mpc1=zeros(sizemy,sizemx);mpc1((px1-1).*size(mpc1,1)+py1)=pz1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
%find t-frame
f=find(b1(:,1)>dxmin&b1(:,1)<sizemx/4*3-10&abs(b1(:,2))<sizemy-10&abs(b1(:,3))<thz0);tb1=round(b1(f,:));px1=tb1(:,1);py1=tb1(:,2);pz1=trgb1(f,2);
t1=round((Rz*[px1';py1'])');px1=t1(:,1);py1=t1(:,2)+sizemy/2;
tmpc1=zeros(sizemy,sizemx);tmpc1((px1-1).*size(tmpc1,1)+py1)=pz1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%imshow(mpc2)
