import robotics.*

main();
clear('sub_node', 'imu_sub', 'imu_pub', 'pcl_pub', 'pcl_sub', 'img_pub', 'img_sub')
rosshutdown


function imu_cb(imu_sub, msg)
    global imu
%     showdetails(imu_msg);
    imu = msg;
end


function pcl_cb(pcl_sub, msg)
    global pcl
    pcl = msg;
end


function img_cb(img_sub, msg)
    global img
    img = msg;
end


function main()
    global imu pcl img
    if ~robotics.ros.internal.Global.isNodeActive
        rosinit;
    end

    sub_node = robotics.ros.Node('/example_sub_node');

    imu_pub = robotics.ros.Publisher(sub_node, '/imu/data', 'sensor_msgs/Imu');
    imu_sub = robotics.ros.Subscriber(sub_node, '/imu/data', 'sensor_msgs/Imu', @imu_cb);
    pcl_pub = robotics.ros.Publisher(sub_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2');
    pcl_sub = robotics.ros.Subscriber(sub_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2', @pcl_cb);
    img_pub = robotics.ros.Publisher(sub_node, 'camera/color/image_raw', 'sensor_msgs/Image');
    img_sub = robotics.ros.Subscriber(sub_node, 'camera/color/image_raw', 'sensor_msgs/Image', @img_cb);

    imu = receive(imu_sub);
    pcl = receive(pcl_sub);
    img = receive(img_sub);

    r = robotics.ros.Rate(sub_node, 5);  % Hz
    figure;
    while robotics.ros.internal.Global.isNodeActive
        quat = [imu.Orientation.W imu.Orientation.X imu.Orientation.Y imu.Orientation.Z];
        eulXYZ = quat2eul(quat,'XYZ');
        roll = eulXYZ(3);
        pitch = -eulXYZ(1);
        yaw = -eulXYZ(2);
        euler.roll = roll;
        euler.pitch = pitch;
        euler.yaw = yaw;
        disp(euler);
        subplot(1,2,1);
        imshow(readImage(img));
        subplot(1,2,2);
        scatter3(pcl);
        drawnow;
        waitfor(r);
    end
end
