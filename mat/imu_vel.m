import robotics.*

main();


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
    imu_pub = robotics.ros.Publisher(imu_sub_node, '/imu/madgwick', 'sensor_msgs/Imu');
    imu_sub = robotics.ros.Subscriber(imu_sub_node, '/imu/madgwick', 'sensor_msgs/Imu', @imu_cb);
%     imu = receive(imu_sub);
    linear_vel = 0;
    linear_position = 0;
    angular_position = 0;
    old_imu = 0;
    rate = 10;
    dt = 0;
    r = robotics.ros.Rate(imu_sub_node, rate);  % Hz
    while robotics.ros.internal.Global.isNodeActive
%         disp(imu.Header.Seq);
        angular_vel = [imu.AngularVelocity.Y, -imu.AngularVelocity.X, imu.AngularVelocity.Z];
        linear_accel = [imu.LinearAcceleration.Y, -imu.LinearAcceleration.X, imu.LinearAcceleration.Z];
        linear_accel = (linear_accel/norm(linear_accel))*(norm(linear_accel)-9.8);
        if old_imu ~= 0
            if rate > 1
                dt = (imu.Header.Stamp.Nsec - old_imu.Header.Stamp.Nsec)*1e-9;
            else
                dt = (imu.Header.Stamp.Sec - old_imu.Header.Stamp.Sec);
            end
%             disp('dt : ')
%             disp(dt)
        end
        old_imu = imu;
        linear_vel = linear_accel*dt + linear_vel;
        linear_position = linear_vel*dt + linear_position;
        angular_position = angular_vel*dt + angular_position;
%         disp('linear_vel : ')
%         disp(linear_vel)
%         disp('angular_vel: ')
%         disp(angular_vel)
        waitfor(r);
    end
end


% rosshutdown
% clear('imu_sub_node')