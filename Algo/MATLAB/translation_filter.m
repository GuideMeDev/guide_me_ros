%% Getting raw data from IMU accelerometer - in 200 hz, and creating the translation in x,y axes (dx,dy)

b = fir1(200,[0.01 0.015]);ax1= filtfilt(b,1,sqrt(acc_raw(:,2).^2+acc_raw(:,3).^2));%ax2= filtfilt(b,1,acc_raw(:,2));ax3= filtfilt(b,1,acc_raw(:,3));
ax1=ax1-mean(ax1);plot([ax1']')
%% Get velocity and translation from Accel
fs=200;
%xz = acc_raw(:,3).*acc_raw(:,2);
%b = fir1(200,[0.02 0.4]);ax= filtfilt(b,1,xz);
ax = ax1;
a=find(diff(diff(ax)>0));a=a+1;%a=a(2:end-1);
amin=find(ax(a)<0);
amax=find(ax(a)>0);
plot(ax,'.-')
hold on
plot(a(amin),ax(a(amin)),'o')
plot(a(amax),ax(a(amax)),'o')
amax=a(amax);amin=a(amin);
%%
t1=[];t=[];L=[];dt=[];vi=[];v0=[];dv=[];
for i=1:length(ax)
    f=find(i>=amax); fmin=find(i>=amin);
    if length(f)>0&length(fmin)>0
    f1=find((i-amax(f))==min((i-amax(f))));
    %f1=f(f1);
    amaxi(i)=ax(amax(f1));
    f1min=find((i-amin(fmin))==min((i-amin(fmin))));
    %f2=fmin(f1min);
    amini(i)=ax(amin(f1min));
    amaxmin = amaxi(i) - amini(i);
    if amaxmin < 1
        amaxmin = 0;
    end
    %k1=0.5; % Y axis, which is z generally
    k1=0.5;% Z axis, which is x generally
    L(i)=k1*(amaxmin).^0.25; % i'th step length
    dt(i)=(2*abs(amax(f1)-amin(f1min))/fs); % delta time of each step
    v0(i)=(L(i))/dt(i); % initial velocity
    k2=0.1; % for X axis - which is y axis generally
    dv(i)=k2*(ax(i)-(amaxi(i)+amini(i))/2); % Acceleration
    vi(i)=v0(i)+dv(i); % overall velocity
    %vi(i) = dv(i); % Y-AXIS velocity
    end
end
plot(vi(1:end-5))
%%
dx=[];
% Delta x - the translation between each 2 following accelerometer samples
dx=(vi((1:end-1))+vi((2:end)))/2*(1/200);
plot(cumsum(dx));
%% Sampling Vi from raw data according to indexes of 6fps with equal timestamps.
for i=1:length(imu_idx)-3
    vi_sampled(i) = vi(imu_idx{i});
end
plot(vi);hold on;plot(imu_idx,vi_sampled)
%%
dxexternal=[];
dxinternal=[];
dx_sampled = [];
%dyinternal=[];
for i=1:length(vi_sampled)-1
    vi1 = vi_sampled(i);vi2 = vi_sampled(i+1);
    %dx_sampled(i)=(vi1+vi2)/2*(1/200);
    tetai_1 = pitch(i+1);tetai = pitch(i);
    h=1.1;%tetai_1=8*pi/180;tetai=30*pi/180;
    xi_1=atan(pi/2-tetai_1)*h;
    xi=atan(pi/2-tetai)*h;
    dt=1/6;
    dxexternal(i)=(vi1+vi2)/2*dt;
    dxinternal(i)=(xi+dxexternal(i))-xi_1;
end
%% Mapping path
x=[];%yawc=yaw;yawc(yawc>0)=yawc(yawc>0)-2*pi;%plot(yawc)
for i=1:length(dxinternal)-1
px=0;py=0;px1=px+dxinternal(i);py1=py+dyinternal(i);
tetaz = yaw(i)-yaw(i+1);
    tetaz1=-tetaz;Rz=[cos(tetaz1),-sin(tetaz1);sin(tetaz1),cos(tetaz1)];t1=(Rz*[px1';py1'])';
x=[x;t1];Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];x=[(Rz*[x]')'];x(:,1)=x(:,1)-dxinternal(i);x(:,2)=x(:,2)-dyinternal(i);%t2=mpc2*0;t1(10-yawt(3,i):end-10-yawt(3,i),k1-yawt(2,i):end-10-yawt(2,i))=t;t2(10:end-10,k1:end-10)=t;
plot(x(:,1),x(:,2),'.-'),axis([-15,15,-15,15])
pause(0.1)
end