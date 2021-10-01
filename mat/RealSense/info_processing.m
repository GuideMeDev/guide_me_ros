%%bags and messages loading
bag_file = '/home/tal/ros_bag/new_data/2021-09-29/2021-09-29-15-58-47.bag';
bagselect = rosbag(bag_file);
rgb = select(bagselect,'Topic','/camera/color/image_raw');
rgbCAM = select(bagselect,'Time',[rgb.StartTime+1,rgb.EndTime-1],'Topic','/camera/color/image_raw');
msgs2 = readMessages(rgbCAM);
%%
points = select(bagselect,'Topic','/camera/depth/color/points');
pointsCAM = select(bagselect,'Time',[points.StartTime+1,points.EndTime-1],'Topic','/camera/depth/color/points');
%%
bagimu = rosbag(bag_file);
imu_info = select(bagimu,'Time',[bagimu.StartTime+1,bagimu.EndTime-1],'Topic','/imu/data');
msg_imu = readMessages(imu_info);
%%
% get rotation matrix for orientation, the accelerometer lin-acceleration,
% the RGB images(I), and the depth images (XYZ), as well as the rgb of the
% depth points.
% All frames, sampled with the closest timestamps to the depth camera frames.
XYZ = [];I=[];
XYZ_origin = [];
t_rgb=rgb.MessageList.Time-rgb.StartTime-1;
t_points = points.MessageList.Time-points.StartTime-1;
t_Imu = imu_info.MessageList.Time-imu_info.StartTime-1;
for i=1:length(t_points)
    t=t_points(i);t1=abs(t_rgb-t);frgb=find(t1==min(t1));t2=abs(t_Imu-t);fimu=find(t2==min(t2));
    msgsPoints =  readMessages(pointsCAM,[i]);
    I{i}=readImage(msgs2{frgb,1});
    Xd = readXYZ(msgsPoints{1,1});
    range=1:5:length(Xd);
    % plot3(Xd(range,3),-Xd(range,1),Xd(range,2),'.')
    % plot3(Xd(range,3),-Xd(range,1),-Xd(range,2),'.')
    % axis([-10,10,-10,10,-10,10]/2)
    Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
    XYZ_origin{i} = Xd;
    XYZ{i} = Xdr;
    pRGB{i} = readRGB(msgsPoints{1,1});
    acc_axes(i,1:3) = [msg_imu{fimu}.LinearAcceleration.X, msg_imu{fimu}.LinearAcceleration.Y, msg_imu{fimu}.LinearAcceleration.Z];
    quat(i,1:4) = [msg_imu{fimu}.Orientation.W,msg_imu{fimu}.Orientation.X,msg_imu{fimu}.Orientation.Y,msg_imu{fimu}.Orientation.Z];
    R = quat2rotm(quat(i,:));
    X(:,i) = R(:,1); Y(:,i)=R(:,2); Z(:,i)=R(:,3);
end
%%
% Get euler Angles (yaw,pitch,roll)
yaw=[];pitch=[];roll=[]
R0 = inv(quat2rotm(quat(1,:)));
for i=1:length(X)
    R=[X(:,i)';Y(:,i)';Z(:,i)']';
    x=[1,0,0;0,0,0;0,1,0;0,0,0;0,0,1;0,0,0];
    x=(R*x')';
    plot3(x(:,1),x(:,2),x(:,3),'-')
    axis([-1,1,-1,1,-1,1])
    pitch(i) = atan2(Y(3,i),Z(3,i));
    roll(i) = -asin(X(3,i));
    yaw(i) = atan2(X(2,i),X(1,i));

end