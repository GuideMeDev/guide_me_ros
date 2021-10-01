%%
% cd('C:\Users\ששש\Documents\Avi\Vis_Imp\Read sensors\RealSense')
%match frames based on IMU
figure('units','normalized','outerposition',[0 0 1 1])
di=2;for i=1:n1/2;XYZ2{i}=XYZ{i*di};end,eulrec2=[roll(n(1:di:n1));-(pitch(n(1:di:n1)))-pi/2;-(diff(yaw(n(1:di:n1+1))))]';
Dx=[];dxrec=(round(1.5-(diff(yaw(n(1:di:n1)))*180/pi)));
X=[];n3=[];n3(1)=1.313;B=[];

for i=1:n1/2-1
Xdr=XYZ2{i};if i>1,n3(i)=n3(i-1);end
tetax=eulrec2(i,1)-1*pi/180;tetay=eulrec2(i,2);tetaz=0;-eulrec2(i+1,3);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
high=[0,0,n3(i)];R1=Rz*Ry*Rx;x=(R1*Xdr')'+repmat(high,length(Xdr),1);%x(:,3)=x(:,3)-n3(i);
x1=x;f=abs(x1(:,2))<1&abs(x1(:,1))<4&abs(x(:,3))<0.1;x1=x(f,:);x1=x1(abs(diff(x1(:,3))./diff(x1(:,2)))<0.12,:);
% x1=x2;x1=x(abs(x1(:,2))<1&abs(x1(:,1))<4&abs(x1(:,3))<0.07,:);x1=x1(abs(diff(x1(:,3))./diff(x1(:,2)))<0.12,:);
x11=sum(x1(:,1).^2);x22=sum(x1(:,2).^2);x33=sum(x1(:,3).^2);x12=sum(x1(:,1).*x1(:,2));x13=sum(x1(:,1).*x1(:,3));x23=sum(x1(:,2).*x1(:,3));
D=x11*x22-x12^2;a=x23*x12-x13*x22;b=x13*x12-x11*x23;
v1=[a,b,D];v1=v1/norm(v1);%n3(i)=n3(i)-mean(x1(:,3)),
n2(i,1:6)=[v1,atan2(v1(1),v1(3))*180/pi,atan2(v1(2),v1(3)),mean(x1(:,3))];
tetax=(atan2(v1(2),v1(3)));tetay=-atan2(v1(1),v1(3));tetaz=0;eulrec2(i,3);%yawt(1,i);
Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
R3=Rz*Ry*Rx;x2=(R3*x(:,:)')';x2(:,3)=x2(:,3)-mean(x2(f,3));n3(i)=n3(i)-mean(x2(f,3));%h1(i)=h1(i)-mean(x(:,3));%
R=R3*R1;eul2(i,1:4)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),eulrec2(i,3),n3(i)];
% plot3(x2(:,1),x2(:,2),x2(:,3)-mean(x1(:,3)),'.'),plot(x2(:,2),x2(:,3),'.'),axis([-3,3,-0.5,2.5]),
% pause
x3{i}=x2;
sc=25;b=x2*1e3/sc;b=round(b((abs(b(:,3))>7&b(:,1)>50&b(:,1)<250),:));px=b(:,1);py=b(:,2)+200;%plot(px,py,'.')
mpc=zeros(400,300);mpc((px-1).*size(mpc,1)+py)=1;%imshow(mpc);%c=conv2(mpc2,ones(4,4)/16,'same');mpc2=(c>0.2);%
M{i}=mpc;B{i}=b(:,1:2);
if i==1,X=B{i};M12{i}=mpc;end
if i>1,m=M{i-1};m(:,:,3)=M{i};%imshow(m),
mpc2=M{i};[py,px]=find(M12{i-1}>0);py=py-200;%b1=B{i-1};px=b1(:,1);py=b1(:,2);
s=[];yaw1=eulrec2(i,3)+[-0.005,0,0.005];for j=1:length(yaw1)
tetaz=yaw1(j);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;%imshow(mpc1)
s(j)=sum(sum((mpc2-mpc1).^2));
end
f=find(s==min(s));f=f(1);tetaz=yaw1(f);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+200;
mpc1=zeros(400,300);mpc1((px1-1).*size(mpc1,1)+py1)=1;

% transy=-3:1:3;transx=-1:8;k1=70;m2=mpc2(10:end-10,k1:end-10);s=[];
transy=-8:1:8;transx=dxrec(i)+[-2:1:4];k1=100;k2=12;m2=mpc2(k2:end-k2,k1:end-k2);s=[];
for j=1:length(transx)
for k=1:length(transy)
m1=mpc1(k2+transy(k):end-k2+transy(k),k1+transx(j):end-k2+transx(j));s(k,j)=sum(sum((m2-m1).^2));
% m=m1;m(:,:,3)=m2;imshow(m),pause(1)
end
end
[fy,fx]=find(s==min(min(s)));fx=fx(1);fy=fy(1);
% m1=mpc1(10+transy(fy):end-10+transy(fy),k1+transx(fx):end-10+transx(fx));Dx(1,i)=transx(fx);Dx(2,i)=transy(fy);
m1=mpc1*0;m1(k2:end-k2,k1:end-k2)=mpc1(k2+transy(fy):end-k2+transy(fy),k1+transx(fx):end-k2+transx(fx));Dx(1,i)=transx(fx);Dx(2,i)=transy(fy);Dx(3,i)=tetaz;
m=m1;m(:,:,3)=mpc2;%
subplot(2,2,[1]),imshow(fliplr(I{i})),
subplot(2,2,[3]),imshow(imrotate((m),90)),
% mpc12=double(mpc1&mpc2);M12{i}=mpc12;%imshow(mpc12),
b1=B{i-1};b2=B{i};X=(Rz*X')';X(:,1)=X(:,1)-Dx(1,i);X(:,2)=X(:,2)-Dx(2,i);X=[X;b2];%X(:,1)=X(:,1)-Dx(1,i);X(:,2)=X(:,2)-Dx(2,i);
x=round(X);x=x(x(:,1)>50&abs(x(:,2)<200),:);x(:,2)=x(:,2)+200;mpc1=zeros(400,300);mpc1((x(:,1)-1).*size(mpc1,1)+x(:,2))=1;M12{i}=mpc1;
% b1=B{i-1};b2=B{i};X=(Rz*X')';b2(:,1)=b2(:,1)-Dx(1,i);b2(:,2)=b2(:,2)-Dx(2,i);X=[X;b2];
subplot(2,2,[2,4]),plot(X(:,2),X(:,1),'.'),axis([-120,120,-80,300])
% m=mpc1;m(:,:,3)=mpc2;imshow(m),
pause(.1)

end
% pause(1)
end

