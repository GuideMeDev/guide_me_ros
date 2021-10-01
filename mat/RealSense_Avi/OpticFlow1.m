figure('units','normalized','outerposition',[0 0 1 1])
imshow(images{1})
figure('units','normalized','outerposition',[0 0 1 1])
imshow(images{2});
%%
figure('units','normalized','outerposition',[0 0 1 1])
x=[];sc=1;f=1.93;sopix=1*3e-3/sc;%a=rgb2gray(I{i});imshow(a)%a=fliplr(rgb2gray(I)')';
kr=100;kc=170;yaw1=[];dx1=[];

% 
for i=1:n1-1
a=rgb2gray(I{i});px0=size(a,2)/2;py0=size(a,1)/2;%imshow(a)  
[SWpixl,repvfl,ll,m]=Image_SW_Projection_5(imadjust(a,[0.2,0.8],[0,1])-1,[eul2(i,1:4),5],15,f,sopix,px0,py0,[],1e3*x3{i},0);
m=m(1+kr:end-kr,1+kc:end-10,:);%imshow(m)
m=(m(:,:,1).*(m(:,:,2)>0));m=50*edge(m,'canny');m(m==0)=-1;t=m*0;t(12:end-11,12:end-11)=m(12:end-11,12:end-11);M{i}=t;st=size(t);%imshow(M{i})
% m=(m(:,:,1).*(m(:,:,2)>0));M{i}=m;st=size(m);m=m-mean2(m);M{i}=m;%imshow(M{i})
if i>1
m1=M{i-1};m2=M{i};
[py,px]=find(m1~=0);f_ind=find(m1~=0);py=py-st(1)/2;%[py2,px2]=find(m2>0);py2=py2-st(1)/2;%b1=B{i-1};px=b1(:,1);py=b1(:,2);
s=[];yaw1=Dx(3,i)+[-0.008,0,0.008];
for j1=1:length(yaw1)
tetaz=yaw1(j1);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+st(1)/2;
m1a=zeros(st(1),st(2));m1a((px1-1).*size(m1,1)+py1)=m1(f_ind);%imshow(mpc1)
s(j1)=sum(sum((m2-m1a).^2));
end
fs=find(s==min(s));fs=fs(1);tetaz=yaw1(fs);eulcam(i,1)=tetaz;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+r1/2;
m1a=zeros(st(1),st(2));m1a((px1-1).*size(m1,1)+py1)=m1(f_ind);Ma{i-1}=m1a;
m=m2;m(:,:,3)=m1a;imshow(m)
% m1=Ma{i1-1};m2=M{i1};%m1=m1(1:2:end,1:2:end);m2=m2(1:2:end,1:2:end);
% k1=30;k2=30;
% images{1}=imresize(m1(k2+1:end-k2,k1+1:end-10),1);images{2}=imresize(m2(k2+1:end-k2,k1+1:end-10),1);
% % m1=double(edge(images{1},'canny'));m2=double(edge(images{2},'canny'));%tic
% m1=((images{1}));m2=((images{2}));%tic
% m1=m1a;
k=30;m1b=m1a(k:end-k,k:end-k);tt= xcorr2(m1b,m2);rangett=round(size(m1b)/2+size(m2)/2);tt1=tt*0;
k1=k-10;k2=10;tt1(rangett(1)-k2:rangett(1)+k1,rangett(2)-k2:rangett(2)+k1)=tt(rangett(1)-k2:rangett(1)+k1,rangett(2)-k2:rangett(2)+k1);
%imagesc(tt1)
[fy,fx]=find(tt1==max(max(tt1)));fx=fx(1);fy=fy(1);dx=([fy,fx]-rangett);
% m=m1;m(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2),3)=m2(k:end-k,k:end-k);
% m=m2;m(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2),3)=m1a();
m=m2;m(k:end-k,k:end-k,3)=m1a(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2));
subplot(1,2,1)
imagesc(tt(rangett(1)-k1:rangett(1)+k1,rangett(2)-k1:rangett(2)+k1))
subplot(1,2,2)
imshow(m)
pause(.1)
dx1(i,:)=dx;
end
end

%%
figure('units','normalized','outerposition',[0 0 1 1])
x=[];sc=1;f=1.93;sopix=1*3e-3/sc;%a=rgb2gray(I{i});imshow(a)%a=fliplr(rgb2gray(I)')';
kr=100;kc=180;yaw1=[];dx1=[];M=[];

% 
for i=1:n1-1
a=rgb2gray(I{i});px0=size(a,2)/2;py0=size(a,1)/2;%imshow(a)  
[SWpixl,repvfl,ll,m]=Image_SW_Projection_5(imadjust(a,[0.5,0.8],[0,1])-1,[eul2(i,1:4),5],15,f,sopix,px0,py0,[],1e3*x3{i},0);
m=m(1+kr:end-kr,1+kc:end-10,:);
m=(m(:,:,1).*(m(:,:,2)>0));t=mean2(m(m>0.2));m(m<=0.2)=t;m=m-t;m(m<0)=-10;m(m>0)=10;%imagesc(m);surf(m)
t=m*0;t(12:end-11,12:end-11)=m(12:end-11,12:end-11);M{i}=t;st=size(t);%imshow(M{i})
% m=(m(:,:,1).*(m(:,:,2)>0));M{i}=m;st=size(m);m=m-mean2(m);M{i}=m;%imshow(M{i})
if i>1
m1=M{i-1};m2=M{i};
[py,px]=find(m1~=0);f_ind=find(m1~=0);py=py-st(1)/2;%[py2,px2]=find(m2>0);py2=py2-st(1)/2;%b1=B{i-1};px=b1(:,1);py=b1(:,2);
s=[];yaw1=Dx(3,i)+[-0.008,0,0.008];
for j1=1:length(yaw1)
tetaz=yaw1(j1);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+st(1)/2;
m1a=zeros(st(1),st(2));m1a((px1-1).*size(m1,1)+py1)=m1(f_ind);%imshow(mpc1)
s(j1)=sum(sum((m2-m1a).^2));
end
fs=find(s==min(s));fs=fs(1);tetaz=yaw1(fs);eulcam(i,1)=tetaz;Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');px1=t1(:,1);py1=t1(:,2)+r1/2;
m1a=zeros(st(1),st(2));m1a((px1-1).*size(m1,1)+py1)=m1(f_ind);Ma{i-1}=m1a;
m=m2;m(:,:,3)=m1a;imshow(m)
% m1=Ma{i1-1};m2=M{i1};%m1=m1(1:2:end,1:2:end);m2=m2(1:2:end,1:2:end);
% k1=30;k2=30;
% images{1}=imresize(m1(k2+1:end-k2,k1+1:end-10),1);images{2}=imresize(m2(k2+1:end-k2,k1+1:end-10),1);
% % m1=double(edge(images{1},'canny'));m2=double(edge(images{2},'canny'));%tic
% m1=((images{1}));m2=((images{2}));%tic
% m1=m1a;
k=30;m1b=m1a(k:end-k,k:end-k);tt= xcorr2(m1b,m2);rangett=round(size(m1b)/2+size(m2)/2);tt1=tt*0;
k1=k-11;k2=10;tt1(rangett(1)-k2:rangett(1)+k2,rangett(2)-k2:rangett(2)+k1)=tt(rangett(1)-k2:rangett(1)+k2,rangett(2)-k2:rangett(2)+k1);
%imagesc(tt1)
[fy,fx]=find(tt1==max(max(tt1)));fx=fx(1);fy=fy(1);dx=([fy,fx]-rangett);
% m=m1;m(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2),3)=m2(k:end-k,k:end-k);
% m=m2;m(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2),3)=m1a();
m=m2;m(k:end-k,k:end-k,3)=m1a(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2));
subplot(1,2,1)
imagesc(tt(rangett(1)-k1:rangett(1)+k1,rangett(2)-k1:rangett(2)+k1))
subplot(1,2,2)
imshow(m)
pause(.1)
dx1(i,:)=dx;
end
end
%%
imshow(m2a)
k1=size(m2a)+size(m1)/2;
plot(tt()
%%
[fy,fx]=find(tt==max(max(tt)));dx=2*([fy,fx]-size(m1));





%%
% Calculate derivates
fx = conv2(images{1},[-1 1; -1 1]/4)-conv2(images{2},[-1 1; -1 1]/4);
fy = conv2(images{1},[-1 -1; 1 1]/4)-conv2(images{2},[-1 -1; 1 1]/4);
ft = conv2(images{1},  ones(5)/9) - conv2(images{2}, ones(5)/9);

% Calculate optical flow
window_size=30;
window_center = floor(window_size / 2);
image_size = size(images{1});
u = zeros(image_size);
v = zeros(image_size);
% for i = image_size(1)/2%image_size(1)/2-3:image_size(1)/2+3%window_center + 1:image_size(1) - window_center
%   for j = image_size(2)/2%image_size(2)/2-3:image_size(2)/2+3%window_center + 1:image_size(2) - window_center
for i = window_center + 1:image_size(1) - window_center
  for j = window_center + 1:image_size(2) - window_center
    % Get values for current window
    fx_window = fx(i - window_center:i + window_center, j - window_center:j + window_center);
    fy_window = fy(i - window_center:i + window_center, j - window_center:j + window_center);
    ft_window = ft(i - window_center:i + window_center, j - window_center:j + window_center);

    fx_window = fx_window';
    fy_window = fy_window';
    ft_window = ft_window';

    A = [fx_window(:) fy_window(:)];

    U = pinv(A' * A) * A' * -ft_window(:);

    u(i, j) = U(1);
    v(i, j) = U(2);
  end
end

% Define csv output format
format = [];
for i = 1:image_size
  if (i == image_size)
    format = [format '%g'];
  else
    format = [format '%g,'];
  end
end
format = [format '\n'];

% Write flow vectors to output file
file = fopen('output.csv', 'w');
fprintf(file, format, u);
fprintf(file, format, v);
fclose(file);

% Display the result
%  dx=round([U(1), U(2)]*30);
dx=round(-40*[u(size(u,1)/2,size(u,1)/2),v(size(u,2)/2,size(u,2)/2)]);
subplot(1,2,1)
axis equal
quiver(impyramid(impyramid(medfilt2(flipud(u), [5 5]), 'reduce'), 'reduce'), -impyramid(impyramid(medfilt2(flipud(v), [5 5]), 'reduce'), 'reduce'));
subplot(1,2,2)
k=20;tt= xcorr2(m1,m2(k:end-k,k:end-k));[fy,fx]=find(tt==max(max(tt)));dx=2*([fy,fx]-size(m1));
% k=11*2;m1a=m1*0;m1a(k+dx(2):end-k+dx(2),k+dx(1):end-k+dx(1))=m1(k:end-k,k:end-k);m=m2;m(:,:,3)=m1a;imshow(m)
k=20;m=m1;m(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2),3)=m2(k:end-k,k:end-k);imshow(m)
pause(1)
end

