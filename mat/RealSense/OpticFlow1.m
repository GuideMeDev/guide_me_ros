figure('units','normalized','outerposition',[0 0 1 1])
imshow(images{1})
figure('units','normalized','outerposition',[0 0 1 1])
imshow(images{2});
%%
figure('units','normalized','outerposition',[0 0 1 1])
% 
for i1=2:n1-1
m1=Ma{i1-1};m2=M{i1};%m1=m1(1:2:end,1:2:end);m2=m2(1:2:end,1:2:end);
k1=30;k2=30;
images{1}=imresize(m1(k2+1:end-k2,k1+1:end-10),1);images{2}=imresize(m2(k2+1:end-k2,k1+1:end-10),1);

% m1=double(edge(images{1},'canny'));m2=double(edge(images{2},'canny'));%tic
m1=((images{1}));m2=((images{2}));%tic

k=22;m2a=m2(k:end-k,k:end-k);tt= xcorr2(m1,m2a);rangett=round(size(m2a)/2+size(m1)/2);tt1=tt*0;
k1=k-1;tt1(rangett(1)-k1:rangett(1)+k1,rangett(2)-k1:rangett(2)+k1)=tt(rangett(1)-k1:rangett(1)+k1,rangett(2)-k1:rangett(2)+k1);
%imagesc(tt1)
[fy,fx]=find(tt1==max(max(tt1)));fx=fx(1);fy=fy(1);dx=([fy,fx]-rangett);
m=m1;m(k+dx(1):end-k+dx(1),k+dx(2):end-k+dx(2),3)=m2(k:end-k,k:end-k);
subplot(1,2,1)
imagesc(tt(rangett(1)-k1:rangett(1)+k1,rangett(2)-k1:rangett(2)+k1))
subplot(1,2,2)
imshow(m)
pause(.1)
dx1(i1,:)=dx;
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

