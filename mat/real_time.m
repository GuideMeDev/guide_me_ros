import robotics.*

global verbose_raw_data init_flag
verbose_raw_data = false;

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


function [I, Xdr, pRGB, acc_axes, pitch, roll, yaw] = info_processing()
    global imu pcl img
    I = readImage(img);
    Xd = readXYZ(pcl);
    range=1:5:length(Xd);
    Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']'; % XYZ
    Xdr(sqrt(sum(Xdr'.^2))>6,:)=0; % XYZ
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
    global imu pcl img verbose_raw_data init_flag
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
        [I, Xdr, pRGB, acc_axes, pitch, roll, yaw] = info_processing();
        if verbose_raw_data
            subplot(1,2,1);
            imshow(I);
            subplot(2,2,2);
            plot3(Xdr(:,1),Xdr(:,2),Xdr(:,3),'.'),view(270,10),axis([-1,6,-5,5,-2,2]/1);
        end
        
%         ### Read_Ros2 ###

        if init_flag
            h1=1.1546;
            eul=[roll1,pitch1,0];
            tetax=eul(1);
            tetay=eul(2);
            tetaz=eul(3);
        else
            h1_old = h1;
            n3_old = n3;
        end
        Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];
        Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
        Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
        high=[0,0,h1];
        R1=Rz*Ry*Rx;
        x=(R1*Xdr')'+repmat(high,length(Xdr),1);
        x1=x;
%         plot3(x1(:,1),x1(:,2),x1(:,3),'.')
        f=find(abs(x(:,2))<1.5&abs(x(:,1))<5&abs(x(:,3))<0.12);
        f1=find(abs(diff(x(f,3))./diff(x(f,2)))<0.12);
        if length(f1)<800
            h1=h1_old;
            eul=[roll1,pitch1,0];
            pcloud1=x;
        else
            x11=sum(x(f,1).^2);
            x22=sum(x(f,2).^2);
            x33=sum(x(f,3).^2);
            x12=sum(x(f,1).*x(f,2));
            x13=sum(x(f,1).*x(f,3));
            x23=sum(x(f,2).*x(f,3));
            D=x11*x22-x12^2;
            a=x23*x12-x13*x22;
            b=x13*x12-x11*x23;
            n1=[a,b,D];
            n1=n1/norm(n1);
            n3=mean(x(f,3));
            n2=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];
            tetax=(atan2(n1(2),n1(3)));
            tetay=-atan2(n1(1),n1(3));
            tetaz=0;
            Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];
            Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
            Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
            R2=Rz*Ry*Rx;x=(R2*x(f,:)')';
%             h1-mean(x(:,3));
            h1=h1-mean(x(:,3)); %???
            R=R2*R1;
            f=find(abs(diff(x(:,3))./diff(x(:,2)))<0.15);
%             plot3(x(:,1),x(:,2),x(:,3),'.')
            f=find(abs(x(:,2))<1&abs(x(:,1))<4&abs(x(:,3)-mean(x(:,3)))<0.06);
            x11=sum(x(f,1).^2);
            x22=sum(x(f,2).^2);
            x33=sum(x(f,3).^2);
            x12=sum(x(f,1).*x(f,2));
            x13=sum(x(f,1).*x(f,3));
            x23=sum(x(f,2).*x(f,3));
            D=x11*x22-x12^2;
            a=x23*x12-x13*x22;
            b=x13*x12-x11*x23;
            n1=[a,b,D];
            n1=n1/norm(n1);
            n3=mean(x(:,3));
            n2=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];
            tetax=(atan2(n1(2),n1(3)));
            tetay=-atan2(n1(1),n1(3));
            tetaz=0;
            Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];
            Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
            Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
            R3=Rz*Ry*Rx;x=(R3*x(f,:)')';
            h1=h1-mean(x(:,3));
            R=R3*R2*R1;
%             plot3(x(:,1),x(:,2),x(:,3),'.')
            f=find(abs(diff(x(:,3))./diff(x(:,2)))<0.12);
            x=x(f,:);
            plot3(x(:,1),x(:,2),x(:,3),'.')

            f=find(abs(x(:,2))<1&abs(x(:,1))<4&abs(x(:,3)-mean(x(:,3)))<0.04);
            x11=sum(x(f,1).^2);
            x22=sum(x(f,2).^2);
            x33=sum(x(f,3).^2);
            x12=sum(x(f,1).*x(f,2));
            x13=sum(x(f,1).*x(f,3));
            x23=sum(x(f,2).*x(f,3));
            D=x11*x22-x12^2;
            a=x23*x12-x13*x22;
            b=x13*x12-x11*x23;
            n1=[a,b,D];n1=n1/norm(n1);
            n3=mean(x(:,3));
            n2=[n1,atan2(n1(1),n1(3))*180/pi,atan2(n1(2),n1(3))*180/pi,mean(x(f,3))];
            tetax=(atan2(n1(2),n1(3)));
            tetay=-atan2(n1(1),n1(3));
            tetaz=0;Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];
            Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
            Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
            R4=Rz*Ry*Rx;x=(R4*x(f,:)')';
            h1=h1-mean(x(:,3));
            R=R4*R3*R2*R1;
            eul=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),0];
%             plot3(x(:,1),x(:,2),x(:,3),'.')

            x11=sum(x(:,1).^2);
            x22=sum(x(:,2).^2);
            x33=sum(x(:,3).^2);
            x12=sum(x(:,1).*x(:,2));
            x13=sum(x(:,1).*x(:,3));
            x23=sum(x(:,2).*x(:,3));
            D=x11*x22-x12^2;
            a=x23*x12-x13*x22;
            b=x13*x12-x11*x23;
            n1=[a,b,D];
            n1=n1/norm(n1);
%             plot3(x(:,1),x(:,2),x(:,3),'.')
            high=[0,0,h1];
            x=(R*Xdr')'+repmat(high,length(Xdr),1);
%             plot(x(:,2),x(:,3),'.'),axis([-3,3,-0.5,1]/1),

            pcloud=x;
            pcloud1=x;
        end
        
%         ### Read_Ros3 ###
        if ~init_flag
            n3 = n3_old;
        end
        tetax = roll-1*pi/180;
        tetay = -pitch(i,2)-pi/2;
        tetaz = 0;
%         -eulrec2(i+1,3);
        Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];
        Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
        Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
        high=[0,0,n3];
        R1=Rz*Ry*Rx;
        x=(R1*Xdr')'+repmat(high,length(Xdr),1);
        x1=x;
        f=abs(x1(:,2))<1&abs(x1(:,1))<4&abs(x(:,3))<0.1;
        x1=x(f,:);
        x1=x1(abs(diff(x1(:,3))./diff(x1(:,2)))<0.12,:);
        x11=sum(x1(:,1).^2);
        x22=sum(x1(:,2).^2);
        x33=sum(x1(:,3).^2);
        x12=sum(x1(:,1).*x1(:,2));
        x13=sum(x1(:,1).*x1(:,3));
        x23=sum(x1(:,2).*x1(:,3));
        D=x11*x22-x12^2;
        a=x23*x12-x13*x22;
        b=x13*x12-x11*x23;
        v1=[a,b,D];v1=v1/norm(v1);
        n2=[v1,atan2(v1(1),v1(3))*180/pi,atan2(v1(2),v1(3)),mean(x1(:,3))];
        tetax=(atan2(v1(2),v1(3)));
        tetay=-atan2(v1(1),v1(3));
        tetaz=0;
%         eulrec2(i,3);
        Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];
        Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
        Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];

%         waitfor(r);
    end
end
