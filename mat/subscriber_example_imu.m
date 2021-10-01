import robotics.*

main();
clear('sub_node', 'imu_sub', 'imu_pub', 'pcl_pub', 'pcl_sub', 'img_pub', 'img_sub', 'imu pcl', 'img', 'c', 'euler')
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
    global imu pcl img c euler
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
    c = 1;
    while robotics.ros.internal.Global.isNodeActive
        quat = [imu.Orientation.W imu.Orientation.X imu.Orientation.Y imu.Orientation.Z];
        eulXYZ = quat2eul(quat,'XYZ');
        roll = eulXYZ(3);
        pitch = -eulXYZ(1);
        yaw = -eulXYZ(2);
        euler.roll{c} = roll;
        euler.pitch{c} = pitch;
        euler.yaw{c} = yaw;
%         disp(euler);
%         subplot(1,2,1);
%         imshow(readImage(img));
%         subplot(1,2,2);
%         scatter3(pcl);
        if c>3
            hold on
            plot(cell2mat(euler.roll)*180/pi,'r')
            plot(cell2mat(euler.pitch)*180/pi,'g')
            plot(cell2mat(euler.yaw)*180/pi,'b')
            legend('roll','pitsh', 'yaw')
            hold off
            drawnow;
        end
        c = c+1;
        waitfor(r);
    end
end




