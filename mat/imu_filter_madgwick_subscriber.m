import robotics.*

main();
clear('imu_sub_node', 'imu_sub', 'imu_pub')
rosshutdown


function imu_cb(imu_sub, imu_msg)
    global imu
%     showdetails(imu_msg);
    imu = imu_msg;
end


function main()
    global imu
    if ~robotics.ros.internal.Global.isNodeActive
        rosinit;
    end
    imu_sub_node = robotics.ros.Node('/imu_sub_node');
    imu_pub = robotics.ros.Publisher(imu_sub_node, '/imu/data', 'sensor_msgs/Imu');
    imu_sub = robotics.ros.Subscriber(imu_sub_node, '/imu/data', 'sensor_msgs/Imu', @imu_cb);
%     imu = receive(imu_sub);
    r = robotics.ros.Rate(imu_sub_node, 1);  % Hz
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
    end
end
