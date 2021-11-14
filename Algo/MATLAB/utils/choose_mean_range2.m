function [mpc2curbe,mpc2nofloor,mpc2floor]=choose_mean_range2(b,thz0,dxmin,dxmax,sizemx,sizemy)

%use only values with x>0
b1=b(b(:,1)>0,:);
%sort the P.C. vector based on ascending x-axis values and name the new variable x
[s1,s2]=sort(b1(:,1));x=b1(s2,:);
%sort the x vector based on ascending y-axis values and name the new variable x
[s1,s2]=sort(x(:,2));x=x(s2,:);
%smooth z-axis values of vector x 
x(:,3)=smooth(x(:,3),30);%plot((x(:,2)),(x(:,3)),'o');
%choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles 
%above the ground) 
f=(x(:,1)>dxmin & x(:,1)<dxmax & x(:,3)< -thz0 & abs(x(:,2))<sizemy/2);
%here we define the pixels for the x-y-axes
ba=round(x(f,:));px2a=ba(:,1);py2a=ba(:,2)+sizemy/2;
%here we create the image mpc2 
mpc2curbe=zeros(sizemy,sizemx);mpc2curbe((px2a-1).*size(mpc2curbe,1)+py2a)=1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
% imshow(mpc2)
%choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles 
%above and below the ground) 
% f=(abs(x(:,3))>thz0/1&abs(x(:,2))<sizemy/2&(x(:,1))<dxmax&x(:,1)>dxmin);ba=round(x(f,:));px2a=ba(:,1);py2a=ba(:,2)+sizemy/2;
% mpc2nofloor=zeros(sizemy,sizemx);mpc2nofloor((px2a-1).*size(mpc2nofloor,1)+py2a)=1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
% f=(abs(x(:,3)-thz0*3/2)<thz0/2&abs(x(:,2))<sizemy/2&(x(:,1))<dxmax&x(:,1)>dxmin);ba=round(x(f,:));px2a=ba(:,1);py2a=ba(:,2)+sizemy/2;
% mpc2nofloor=zeros(sizemy,sizemx);mpc2nofloor((px2a-1).*size(mpc2nofloor,1)+py2a)=x(f,3);%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
f=((x(:,3)-thz0*2)>0&abs(x(:,2))<sizemy/2&(x(:,1))<dxmax&x(:,1)>dxmin);ba=round(x(f,:));px2a=ba(:,1);py2a=ba(:,2)+sizemy/2;
mpc2nofloor=zeros(sizemy,sizemx);mpc2nofloor((px2a-1).*size(mpc2nofloor,1)+py2a)=x(f,3);%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
%choosing specific area withing x (next we transform coordinates to pixels to create an image of obstacles 
%below the ground)   
f=(abs(x(:,3))<thz0/1&abs(x(:,2))<sizemy/2&(x(:,1))<dxmax&x(:,1)>dxmin);ba=round(x(f,:));px2a=ba(:,1);py2a=ba(:,2)+sizemy/2;
mpc2floor=zeros(sizemy,sizemx);mpc2floor((px2a-1).*size(mpc2floor,1)+py2a)=1;%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
aa=6;
    