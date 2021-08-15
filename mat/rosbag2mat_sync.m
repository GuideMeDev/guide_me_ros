import robotics.*

global ii_imu ii_image ii_pcl roll pitch yaw XYZ I;
ii_imu = 1;
ii_image = 1;
ii_pcl = 1;

main();


function imu_cb(imu_sub, imu_msg)
    global ii_imu roll pitch yaw;
    quat{ii_imu} = [imu_msg{1}.Orientation.W imu_msg{1}.Orientation.X imu_msg{1}.Orientation.Y imu_msg{1}.Orientation.Z];
    eulXYZ = quat2eul(quat{ii_imu},'XYZ');
    roll{ii_imu} = eulXYZ(2);
    pitch{ii_imu} = -eulXYZ(1);
    yaw{ii_imu} = eulXYZ(3);
    ii_imu  = ii_imu +  1;
end


function image_cb(image_sub, image_msg)
    global ii_image I;
    I{ii_image} = readImage(image_msg{1,1});
    ii_image = ii_image +1;
end


function pcl_cb(pcl_sub, pcl_msg)
    global ii_pcl XYZ;
    XYZ{ii_imu} = readXYZ(pcl_msg{1,1});
    ii_pcl = ii_pcl +1;
end


function main()
    global roll pitch yaw XYZ I;
    if ~robotics.ros.internal.Global.isNodeActive
        rosinit;
    end
    rsbag2mat_node = robotics.ros.Node('/rsbag2mat_node');
    imu_pub = robotics.ros.Publisher(rsbag2mat_node, '/imu/madgwick', 'sensor_msgs/Imu');
    imu_sub = robotics.ros.Subscriber(rsbag2mat_node, '/imu/madgwick', 'sensor_msgs/Imu', @imu_cb);
    
    image_pub = robotics.ros.Publisher(rsbag2mat_node, '/camera/color/image_raw', 'sensor_msgs/Image');
    image_sub = robotics.ros.Subscriber(rsbag2mat_node, '/camera/color/image_raw', 'sensor_msgs/Image', @image_cb);
    
    pcl_pub = robotics.ros.Publisher(rsbag2mat_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2');
    pcl_sub = robotics.ros.Subscriber(rsbag2mat_node, '/camera/depth/color/points', 'sensor_msgs/PointCloud2', @pcl_cb);
%     imu = receive(imu_sub);
    r = robotics.ros.Rate(rsbag2mat_node, 5);  % Hz
    waitfor(r);
    
    quat = [];
    roll = [];
    pitch = [];
    yaw = [];
    I = [];
    XYZ = [];
    ii = 1;
    while robotics.ros.internal.Global.isNodeActive
        ii  = ii +  1;
        waitfor(r);
    end
end



% clear('rsbag2mat_node', 'imu_sub', 'imu_pub', 'image_pub', 'image_sub', 'pcl_pub', 'pcl_sub')
% rosshutdown