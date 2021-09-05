import robotics.*

main();
clear('sub_node', 'imu_sub', 'imu_pub', 'pcl_pub', 'pcl_sub')
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


function main()
    global imu pcl
    if ~robotics.ros.internal.Global.isNodeActive
        rosinit;
    end
    sub_node = robotics.ros.Node('/example_sub_node');
    imu_pub = robotics.ros.Publisher(sub_node, '/imu/data', 'sensor_msgs/Imu');
    imu_sub = robotics.ros.Subscriber(sub_node, '/imu/data', 'sensor_msgs/Imu', @imu_cb);
    pcl_pub = robotics.ros.Publisher(sub_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2');
    pcl_sub = robotics.ros.Subscriber(sub_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2', @pcl_cb);
%     imu = receive(imu_sub);
    r = robotics.ros.Rate(sub_node, 5);  % Hz
    figure;
    while robotics.ros.internal.Global.isNodeActive
        waitfor(r);
        quat = [imu.Orientation.W imu.Orientation.X imu.Orientation.Y imu.Orientation.Z];
        eulXYZ = quat2eul(quat,'XYZ');
        roll = eulXYZ(3);
        pitch = -eulXYZ(1);
        yaw = -eulXYZ(2);
        euler.roll = roll;
        euler.pitch = pitch;
        euler.yaw = yaw;
        disp(euler);
        scatter3(pcl);
        drawnow;
    end
end
