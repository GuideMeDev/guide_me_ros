function [pcloud] = plane_fit(XYZ,eul)
pcloud=[];N=[];h1=[];
for i=1:length(XYZ)
Xdr=XYZ{1,i};i,%range=1:5:length(Xd);Xdr=Xd;%[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
%using euler and translation from first frame
if i<=1,h1(i)=1.45;eul(i,1:3)=[roll(i)+2*pi/180,-(pitch(i)+pi/2),0]*1;tetax=eul(i,1);tetay=eul(i,2);tetaz=eul(i,3);
else
%using euler and translation from previous frame    
    h1(i)=h1(i-1);tetax=roll(i)+2*pi/180;tetay=-(pitch(i)+pi/2);tetaz=0;%eul(i-1,3);
end
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,h1(i)];R1=Rz*Ry*Rx;
%1st approximation of the s.w. points of Xdr to x-y plane
%rotating and translating p.c. to fit s.w. to x-y plane 
x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;%[round(x(:,1)'*90);round(x(:,2)'*90);round(x(:,3)'*90)]'/90;f=find(diff(x1(:,1))==0&diff(x1(:,2))==0);x1(f,3)=-0.5;
x=x1;t=(abs(diff(x1(:,3))./diff(x1(:,2))));f=find(abs(x1(:,2))<1.0&abs(x1(:,1))<4&abs(x1(:,3))<0.20&[(abs(diff(x1(:,3))./diff(x1(:,2))));1]<0.22&[1;(abs(diff(x1(:,3))./diff(x1(:,2))))]<0.22);
%choosing only points with height abs(z)<5cm
f1=abs(x1(f,3)-prctile(x1(f,3),50))<0.05;f=f(f1);
plot3(x1(:,1),x1(:,2),x1(:,3),'.')
x=x1(f,:);
%linear fit to points in y-z axes 
n=polyfit(x(:,2),x(:,3),1);xn=x(:,2)*n(1)+n(2);f1=abs(x(:,3)-xn)<0.03;%plot(x(:,2),x(:,3),'.');hold on;plot(x(f1,2),x(f1,3),'.');
n=polyfit(x(f1,2),x(f1,3),1);xn=x(:,2)*n(1)+n(2);nx=n(1);f2=abs(x(:,3)-xn)<0.02;%plot(x((f2),2),x((f2),3),'o');hold on;plot(x(:,2),x(:,3),'.')
%linear fit to points in x-z axes 
n=polyfit(x(f2,1),x(f2,3),1);xn=x(:,1)*n(1)+n(2);ny=n(1);f3=abs(x(:,3)-xn)<0.03;%plot(x((f3),1),x((f3),3),'o');hold on;plot(x(:,1),x(:,3),'.')
% pause
%to find rotation matrix R2 around x, y and z axes derived from the above linear fit
tetax=-atan(nx);tetay=atan(ny);tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R2=Rz*Ry*Rx;
%to find translation of the s.w. points in z axis
h1new=mean((R2*R1*Xdr(f(f3),:)')');h1new=h1new(3)+h1(i);h1(i)=h1(i)-h1new;
%2nd approximation of the s.w. points of Xdr to x-y plane
high=[0,0,h1(i)];x2=(R2*R1*Xdr')'+repmat(high,length(Xdr),1);R=R2*R1;eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
%choosing p.c. points within a volume in front of the user
f=find(abs(x2(:,2))<1.5&abs(x2(:,1)-3)<1&abs(x2(:,3))<0.08&[(abs(diff(x2(:,3))./diff(x2(:,2))));1]<0.22&[1;(abs(diff(x2(:,3))./diff(x2(:,2))))]<0.22);
%applying RANSAC. choosing 50 clusters of 50 p.c. points in search for the cluster with the best planar fit  
xr=[];S=[];yr=[];zr=[];nn=[];r=randi(round(length(f)/2),[50,50]);% pause
for j=1:size(r,1);xr(j,:)=x2(f(r(j,:)),1)';yr(j,:)=x2(f(r(j,:)),2)';zr(j,:)=x2(f(r(j,:)),3)';
%linear fit to each cluster in its x-z and y-z planes.  
nx=polyfit(xr(j,:),zr(j,:),1);xn=abs(x(:,1)*nx(1)+nx(2)-x(:,3));ny=polyfit(yr(j,:),zr(j,:),1);yn=abs(x(:,2)*ny(1)+ny(2)-x(:,3));
%measure the variation for between the samples and fit of each cluster 
S(j,:)=[std(xn),std(yn)];nn(j,:)=[nx,ny];
end
%to find the angles of the cluster with the best fit to a plane
S=sum(S')';fs=S==min(S);s1(i,:)=S(fs)';nx=nn(fs,1:2);ny=nn(fs,3:4);fr=[f(r(fs,:))'];
%to find the p.c. points of the cluster with the best fit to a plane
x=x2(fr,:);f=abs(x2(fr,3)-mean(x2(fr,3)))<0.02;fr=fr(f);x=x2(fr,:);
% x=x1(fr,:);f=x1(fr,3)<0.015;fr=fr(f);x=x1(fr,:);
% plot3(x(:,1),x(:,2),x(:,3),'o'),hold on%axis([1,9,-2,2,-0.5,1.5]);view(280,0)
%removing the mean of the points of the cluster and finding the plane with the best fit to cluster 
x(:,1)=x(:,1)-mean(x(:,1));x(:,2)=x(:,2)-mean(x(:,2));
x11=sum(x(:,1).^2);x22=sum(x(:,2).^2);x33=sum(x(:,3).^2);x12=sum(x(:,1).*x(:,2));x13=sum(x(:,1).*x(:,3));x23=sum(x(:,2).*x(:,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
%n1 represents the plane with the best fit to the cluster
n1=[a,b,D];n1=n1/norm(n1);
%to find the rotation angles required to rotate the n1 plane to become parallel to
%x-y plane
tetax=atan2(n1(2),sqrt(n1(3)^2+n1(1)^2));tetay=-atan2(n1(1),sqrt(n1(3)^2+n1(2)^2));tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
%3rd rotation and translation of Xdr
high=[0,0,h1(i)];R3=Rz*Ry*Rx;R=R3*R;x3=(R*Xdr')'+repmat(high,length(Xdr),1);h1(i)=h1(i)-mean(x3(fr,3));x3(:,3)=x3(:,3)-mean(x3(fr,3));
eul(i,1:3)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0]; 

hold off
plot3(x3(:,1),x3(:,2),x3(:,3),'.');axis([1,4,-2,2,-0.3,0.5]);view(280,1);%axis([-1,1,-0.1,0.1]);subplot(2,1,2);plot(x2(f,1),x2(f,3),'.');axis([1,3,-0.1,0.1]);
hold on
plot3(x3(f,1),x3(f,2),x3(f,3),'o');
plot3(x3(fr,1),x3(fr,2),x3(fr,3),'*');
pause(.1)
pcloud{i}=x3;

% Performance Metric
s(i)=plane_fit_param(x3);
end
pfp=[mean(s),std(s)];