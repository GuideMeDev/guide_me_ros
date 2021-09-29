import robotics.*

global verbose
verbose = false;

main();
clear('sub_node', 'imu_sub', 'imu_pub', 'pcl_pub', 'pcl_sub', 'img_pub', 'img_sub')
rosshutdown


function imu_cb(imu_sub, msg)
    global imu
%     showdetails(imu_msg);
    imu = msg;
end


% function pcl_cb(pcl_sub, msg)
%     global pcl
%     pcl = msg;
% end


function img_cb(img_sub, msg)
    global img
    img = msg;
end


function [I, XYZ, pRGB, acc_axes, pitch, roll, yaw] = info_processing()
    global imu pcl img
    I = readImage(img);
    Xd = readXYZ(pcl);
    range=1:5:length(Xd);
    XYZ=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']'; % Xdr
    XYZ(sqrt(sum(XYZ'.^2))>6,:)=0; % Xdr
    pRGB = readRGB(pcl);
    acc_axes = [imu.LinearAcceleration.X, imu.LinearAcceleration.Y, imu.LinearAcceleration.Z];
    quat = [imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z];
    R = quat2rotm(quat);
    X = R(:,1); Y=R(:,2); Z=R(:,3);
%     R=[X';Y';Z']';
%     x=[1,0,0;0,0,0;0,1,0;0,0,0;0,0,1;0,0,0];
%     x=(R*x')';
    pitch = atan2(Y(3),Z(3));
    roll = -asin(X(3));
    yaw = atan2(X(2),X(1));
end


function main()
    global imu pcl img verbose
    if ~robotics.ros.internal.Global.isNodeActive
        rosinit;
    end

    sub_node = robotics.ros.Node('/example_sub_node');

    imu_pub = robotics.ros.Publisher(sub_node, '/imu/data', 'sensor_msgs/Imu');
    imu_sub = robotics.ros.Subscriber(sub_node, '/imu/data', 'sensor_msgs/Imu', @imu_cb);
    pcl_pub = robotics.ros.Publisher(sub_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2');
    pcl_sub = robotics.ros.Subscriber(sub_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2');
    img_pub = robotics.ros.Publisher(sub_node, 'camera/color/image_raw', 'sensor_msgs/Image');
    img_sub = robotics.ros.Subscriber(sub_node, 'camera/color/image_raw', 'sensor_msgs/Image', @img_cb);

    pcl = receive(pcl_sub);
    imu = receive(imu_sub);
    img = receive(img_sub);

%     r = robotics.ros.Rate(sub_node, 6);  % Hz
    while robotics.ros.internal.Global.isNodeActive
        pcl = receive(pcl_sub);
        [I, XYZ, pRGB, acc_axes, pitch, roll, yaw] = info_processing();
        if verbose
            subplot(1,2,1);
            imshow(I);
            subplot(2,2,2);
            Xdr=XYZ;
            plot3(Xdr(:,1),Xdr(:,2),Xdr(:,3),'.'),view(270,10),axis([-1,6,-5,5,-2,2]/1);
        end
%         waitfor(r);
    end
end
