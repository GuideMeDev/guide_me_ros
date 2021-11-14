function [mpc1,tetaz,b1b]=correct_reg_angle2(b1a,rtb1,mpc2,yaw1,sizemx,sizemy)
%this function is similar to the function correct_reg_angle2,
%except here we rotate the points of b1 by a range of angles 
%in search for best fit between mpc1 and mpc2
% b1=b1(b1(:,1)<dxmax&abs(b1(:,2))<180,:);b2=b2(b2(:,1)<dxmax&abs(b2(:,2))<180,:);
% px2=b2(:,1);py2=b2(:,2)+sizemy/2;pz2=b2(:,3);mpc2=zeros(sizemy,sizemx);mpc2((px2-1).*size(mpc2,1)+py2)=pz2;
px=b1a(:,1);py=b1a(:,2);pz=b1a(:,3);    
pxb=rtb1(:,1);pyb=rtb1(:,2);pzb=rtb1(:,3);    
s=[];
% b2a=[px2';py2';pz2']';%   
for j=1:length(yaw1)   
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+sizemy/2;
mpc1=zeros(sizemy,sizemx);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%imshow(mpc1)
% s(j)=sum(sum((mpc2-mpc1).^2))/sum(sum(mpc2>0&mpc1>0));
s(j)=sum(sum((mpc2.*mpc1)));%/sum(sum(mpc2&mpc1)); 
end   
f=find(s==max(s));f=f(1);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
t1=((Rz*[pxb';pyb'])');px1=t1(:,1);py1=t1(:,2)+sizemy/2;b1b=[px1';py1'-sizemy/2;pzb']';
t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+sizemy/2;
mpc1=zeros(sizemy,sizemx);mpc1((px1-1).*size(mpc1,1)+py1)=pz;%b1b=[px1';py1';pz']';
mpc1(((px1+1)-1).*size(mpc1,1)+py1)=pz;
mpc1(((px1-1)-1).*size(mpc1,1)+py1)=pz;
mpc1((px1-1).*size(mpc1,1)+(py1+1))=pz;
mpc1((px1-1).*size(mpc1,1)+(py1-1))=pz;

% t1=round((Rz*[b1(:,1)';b1(:,2)'])');
% if show==1
% plot3(t1(:,1),t1(:,2),b1(:,3),'.');hold on;plot3(b2(:,1),b2(:,2),b2(:,3),'.')
% end
% yawt(1,i)=yaw1(f);
