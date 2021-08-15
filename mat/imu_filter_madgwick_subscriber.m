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
    r = robotics.ros.Rate(imu_sub_node, 0.5);  % Hz
%     while robotics.ros.internal.Global.isNodeActive
    for ii = 1:50
        waitfor(r);
%         disp(imu.Header.Seq);
        angular_vel = [imu.AngularVelocity.Y, -imu.AngularVelocity.X, imu.AngularVelocity.Z];
        accel = [imu.LinearAcceleration.Y, -imu.LinearAcceleration.X, imu.LinearAcceleration.Z];
        quat = [imu.Orientation.W imu.Orientation.X imu.Orientation.Y imu.Orientation.Z];
        eulXYZ = quat2eul(quat,'XYZ');
        roll = eulXYZ(2);
        pitch = -eulXYZ(1);
        yaw = eulXYZ(3);
        euler.roll{ii} = roll;
        euler.pitch{ii} = pitch;
        euler.yaw{ii} = yaw;
%         orient_cov = imu.OrientationCovariance;
        disp(euler.yaw{ii});
    end
    save('yaw.mat', '-struct', 'euler')
end


% rosshutdown
% clear('imu_sub_node')


%%
yaw_mat = cell2mat(yaw)*180/pi;
pitch_mat = cell2mat(pitch)*180/pi;
roll_mat = cell2mat(roll)*180/pi;

figure('Name','Around roll');
plot(roll_mat)

hold on
plot(pitch_mat)
plot(yaw_mat)
legend('roll','pitsh', 'yaw')
