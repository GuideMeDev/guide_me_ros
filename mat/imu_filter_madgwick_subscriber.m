import robotics.*

main();


function imu_cb(imu_sub, imu_msg)
    showdetails(imu_msg);
end



function main()
    if ~robotics.ros.internal.Global.isNodeActive
        rosinit;
    end
    imu_sub_node = robotics.ros.Node('/imu_sub_node');
    imu_pub = robotics.ros.Publisher(imu_sub_node, '/imu/madgwick', 'sensor_msgs/Imu');
    imu_sub = robotics.ros.Subscriber(imu_sub_node, '/imu/madgwick', 'sensor_msgs/Imu', @imu_cb);
%     imu = receive(imu_sub);
    r = robotics.ros.Rate(imu_sub_node, 1);  % Hz
    while robotics.ros.internal.Global.isNodeActive
        waitfor(r);
    end
end



% rosshutdown
% clear('imu_sub_node')