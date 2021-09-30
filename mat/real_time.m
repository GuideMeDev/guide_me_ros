import robotics.*

global show_raw_data init_flag show_read_ros
init_flag = true;
show_raw_data = false;
show_read_ros = false;

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
    global imu pcl img show_raw_data init_flag show_read_ros
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

    pcl = receive(pcl_sub);
    imu = receive(imu_sub);
    img = receive(img_sub);

    r = robotics.ros.Rate(sub_node, 6);  % Hz
    while robotics.ros.internal.Global.isNodeActive
        [I, Xdr, pRGB, acc_axes, pitch, roll, yaw] = info_processing();
        if show_raw_data
            subplot(1,2,1);
            imshow(I);
            subplot(2,2,2);
            plot3(Xdr(:,1),Xdr(:,2),Xdr(:,3),'.'),view(270,10),axis([-1,6,-5,5,-2,2]/1);
        end
        
%         ### Read_Ros2 ###

        if init_flag
            h1=1.1546;
            n3=1.313;
            h1_old = h1;
            eul=[roll-2*pi/180,-(pitch+pi/2),0];
            yaw_old = yaw;
        end
        tetax=eul(1);
        tetay=eul(2);
        tetaz=eul(3);
        
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
            eul=[roll-2*pi/180,-(pitch+pi/2),0];
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
%             plot3(x(:,1),x(:,2),x(:,3),'.')

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
        tetay = -pitch-pi/2;
        tetaz = 0;
%         -eulrec2(i+1,3);???
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
%         eulrec2(i,3);???
        Rz=[cos(tetaz),-sin(tetaz),0;sin(tetaz),cos(tetaz),0;0,0,1];
        Ry=[cos(tetay),0,sin(tetay);0,1,0;-sin(tetay),0,cos(tetay)];
        Rx=[1,0,0;0,cos(tetax),-sin(tetax);0,sin(tetax),cos(tetax)];
        R3=Rz*Ry*Rx;x2=(R3*x(:,:)')';
        x2(:,3)=x2(:,3)-mean(x2(f,3));
        n3=n3-mean(x2(f,3));
        R=R3*R1;
%         eul2(i,1:4)=[atan2(R(3,2),R(3,3)),-asin(R(3,1)),eulrec2(i,3),n3(i)];
        x3=x2;
        sc=25;
        b=x2*1e3/sc;
        b=round(b((abs(b(:,3))>7&b(:,1)>50&b(:,1)<250),:));
        px=b(:,1);
        py=b(:,2)+200;
        mpc=zeros(400,300);
        mpc((px-1).*size(mpc,1)+py)=1;
        M=mpc;
        B=b(:,1:2);
        if init_flag
            X=B;
            M12=mpc;
        else
            m=M_old;
            m(:,:,3)=M;
            mpc2=M;
            [py,px]=find(M12_old>0);
            py=py-200;
            s=[];
            yaw1=-(yaw - yaw_old)+[-0.005,0,0.005];

            for j=1:length(yaw1)
                tetaz=yaw1(j);
                Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];t1=round((Rz*[px';py'])');
                px1=t1(:,1);
                py1=t1(:,2)+200;
                mpc1=zeros(400,300);
                try
                    mpc1((px1-1).*size(mpc1,1)+py1)=1;
                catch
                    warning('Error in real_time>main \nline: "mpc1((px1-1).*size(mpc1,1)+py1)=1;" \nArray indices must be positive integers or logical values. \n','');
                end
                s(j)=sum(sum((mpc2-mpc1).^2));
            end

            f=find(s==min(s));
            f=f(1);
            tetaz=yaw1(f);
            Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
            t1=round((Rz*[px';py'])');
            px1=t1(:,1);
            py1=t1(:,2)+200;
            mpc1=zeros(400,300);
            try
                mpc1((px1-1).*size(mpc1,1)+py1)=1;
            catch
                warning('Error in real_time>main \nline: "mpc1((px1-1).*size(mpc1,1)+py1)=1;" \nArray indices must be positive integers or logical values. \n','');
            end
            transy=-8:1:8;
            transx=h1+[-2:1:4]; %h1 == round(1.5-((yaw-yaw_old)*180/pi)) ????
            k1=100;k2=12;
            m2=mpc2(k2:end-k2,k1:end-k2);
            s=[];
            
            for j=1:length(transx)
                for k=1:length(transy)
                    try
                        m1=mpc1(k2+transy(k):end-k2+transy(k),k1+transx(j):end-k2+transx(j));
                    catch
                        warning('Error in real_time>main \nline: " m1=mpc1(k2+transy(k):end-k2+transy(k),k1+transx(j):end-k2+transx(j));" \nNonfinite endpoints or increment for colon operator in index. \n','');
                    end
                    try
                        s(k,j)=sum(sum((m2-m1).^2));
                    catch
                        warning('Error in real_time>main \nline: "s(k,j)=sum(sum((m2-m1).^2));" \nMatrix dimensions must agree. \n','');
                    end
                end
            end

            [fy,fx]=find(s==min(min(s)));
            try
                fx=fx(1);
                fy=fy(1);
            catch
                warning('Error in real_time>main \nline: "fx=fx(1);" \nIndex exceeds array bounds. \n','');
            end
            m1=mpc1*0;
            try
                m1(k2:end-k2,k1:end-k2)=mpc1(k2+transy(fy):end-k2+transy(fy),k1+transx(fx):end-k2+transx(fx));
            catch
                warning('Error in real_time>main \nline: "m1(k2:end-k2,k1:end-k2)=mpc1(k2+transy(fy):end-k2+transy(fy),k1+transx(fx):end-k2+transx(fx));" \nUnable to perform assignment because the size of the left side is 377-by-189 and the size of the right side is 0-by-0. \n','');
            end
            try
                Dx(1)=transx(fx);
                Dx(2)=transy(fy);
                Dx(3)=tetaz;
            catch
                warning('Error in real_time>main \nline: "Dx(1)=transx(fx);" \nUnable to perform assignment because the left and right sides have a different number of elements. \n','');
            end
            m=m1;m(:,:,3)=mpc2;
%             subplot(2,2,[1]),imshow(fliplr(I{i})),
%             subplot(2,2,[3]),imshow(imrotate((m),90)),
            b1=B_old;
            b2=B;
            X=(Rz*X')';
            X(:,1)=X(:,1)-Dx(1);
            X(:,2)=X(:,2)-Dx(2);
            X=[X;b2];
            x=round(X);
            x=x(x(:,1)>50&abs(x(:,2)<200),:);
            x(:,2)=x(:,2)+200;
            mpc1=zeros(400,300);
            try
                mpc1((x(:,1)-1).*size(mpc1,1)+x(:,2))=1;
            catch
                warning('Error in real_time>main \nline: "mpc1((x(:,1)-1).*size(mpc1,1)+x(:,2))=1;" \nAttempt to grow array along ambiguous dimension. \n','');
            end
            M12=mpc1;
%             subplot(2,2,[2,4]),plot(X(:,2),X(:,1),'.'),axis([-120,120,-80,300])
        end

        if show_read_ros && ~init_flag
            subplot(2,2,1),imshow(fliplr(I)),
            subplot(2,2,3),imshow(imrotate((m),90)),
            subplot(2,2,[2,4]),plot(X(:,2),X(:,1),'.'),axis([-120,120,-80,300])
        end

%         ### bypass1 ###
        if init_flag
            tetazT=zeros(3,200);
            xy1=[0,2];
            v1=0;
            v0=pi/2-100*pi/180;
            xy0=[-40,30];
            dwi=15;
            dlen=80;
            sc1=10;
            sc2=1;
            sc3=5*pi/180;
            sc4=2;

            m=M;
            lmap = M;
            [py,px]=find(m>0);
            f1=find(m>0);
            px=px-size(lmap,2)/2;
            px=px-xy0(1);
            py=py-xy0(2);
            tetazT(1)=v0;
            tetaz=v0;
            Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
            t1=round((Rz*[px';py'])');
            f=abs(t1(:,2)-size(lmap,2)/2)<size(lmap,2)/2&abs(t1(:,1))<size(lmap,1)/2;
            t1=t1(f,:);
            px1=t1(:,1)+size(lmap,2)/2;py1=t1(:,2);
            mpc1=zeros(size(lmap));
            mpc1((px1-1).*size(mpc1,1)+py1)=m(f1(f));
            M=mpc1;
        else
            m=M_old;
            [py,px]=find(m>0);
            f1=find(m>0);
            px=px-size(lmap,2)/2;px=px-xy1(1);
            py=py-xy1(2);
            try
                tetazT(1)=-v1;
            catch
                warning('Error in real_time>main \nline: "tetazT(1)=-v1;" \nUnable to perform assignment because the left and right sides have a different number of elements. \n','');
            end
            tetaz=-v1;
            Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
            t1=round((Rz*[px';py'])');
            f=abs(t1(:,2)-size(lmap,2)/2)<size(lmap,2)/2&abs(t1(:,1))<size(lmap,1)/2;
            t1=t1(f,:);
            px1=t1(:,1)+size(lmap,2)/2;py1=t1(:,2);
%             plot(px1,py1,'.');
            mpc1=zeros(size(lmap));
            mpc1((px1-1).*size(mpc1,1)+py1)=m(f1(f));
            tt=mpc1*0;mpc2=tt;mpc2(1:dlen,size(lmap,1)/2-dwi:size(lmap,1)/2+dwi)=1;
            mpc1(:,:,3)=mpc2;M=mpc1;
%             imshow(mpc1);
            v1=0;
            t1=mpc1(:,:,1).*mpc2;

            if max(max(t1))>0.9 && sum(sum(t1))>5
                mpc2=tt;
                mpc2(1:dlen*sc2,size(lmap,1)/2-dwi*sc1:size(lmap,1)/2-dwi)=1;
                mpc2(1:dlen*sc2,size(lmap,1)/2+dwi:size(lmap,1)/2+dwi*sc1)=0.5;
                mpc1(:,:,2)=mpc2;
%                 imshow(mpc1);
                t=sum(((mpc1(:,:,1)>0.1).*(mpc1(:,:,2))))>0;
                mpc2(:,t)=0;
                mpc1(:,:,2)=mpc2;
%                 imshow(mpc1);
                t=sum(mpc1(:,:,2))>0;
                f=find(t>0);
                f1=(f-size(lmap,2)/2);
                f1a=f1(f1<0);
                f1b=f1(f1>0);
                F{1}=f1a+size(lmap,2)/2;
                F{2}=f1b+size(lmap,2)/2;
                j=[length(f1a),length(f1b)]==min([length(f1a),length(f1b)]);
                mpc1(:,F{sum(j.*[1,2])},2)=0;
%                 imshow(mpc1);
                t=sum(mpc1(:,:,2))>0;
                f=find(t>0);
                f1=(f-size(lmap,2)/2);
                f2=find(abs(f1)==min(abs(f1)));
                f2=f1(f2);
                if f2<0
                    f1(f1>0)=0;
                    k=-1;
                else
                    f1(f1<0)=0;
                    k=1;
                end
                t=find(abs(f1)==max(abs(f1)));
                t=f(t);
                v2=pi/2-atan2(dlen*sc2,t);
                v1=-sc3;
                if k<0
                    v1=sc3
                end
                tetazT(2)=1;
            else
                if ~init_flag && max(max(t1))==0.1 && sum(tetazT(2))==0
                    mpc2=tt;
                    mpc2(1:dlen*sc2,size(lmap,1)/2-dwi*sc4:size(lmap,1)/2-dwi)=1;
                    mpc2(1:dlen*sc2,size(lmap,1)/2+dwi:size(lmap,1)/2+dwi*sc4)=0.5;
                    mpc1(:,:,2)=mpc2;
%                     imshow(mpc1);
                    t=sum(((mpc1(:,:,1)==0.1).*(mpc1(:,:,2)))>0);
                    s1=sum(t(1:size(lmap,2)/2));
                    s2=sum(t(1+size(lmap,2)/2:end));
                    k=0;
                    if s1<s2
                        k=size(lmap,2)/2
                    end
                    mpc2(:,1+k:size(lmap,2)/2+k)=0;
                    mpc1(:,:,2)=mpc2;
%                     imshow(mpc1);
                    t=sum(mpc1(:,:,2))>0;
                    f=find(t>0)-size(lmap,2)/2;
                    f1=length(f(f>0));
                    f2=length(f(f<0));
                    if abs(f1-f2)>10 && (s1/s2>1.5 || s2/s1>1.5)
                        t=sum(((mpc1(:,:,2))));
                        s1=sum(t(1:size(lmap,2)/2));
                        s2=sum(t(1+size(lmap,2)/2:end));
                        k=0;
                        k1=-1;
                        if s1-s2>0
                            k=size(lmap,2)/2;
                            k1=1;
                        end
                        mpc2(:,1+k:size(lmap,2)/2+k)=0;
                        mpc1(:,:,2)=mpc2;
%                         imshow(mpc1);
                        v1=-sc3;
                        if k1>0
                            v1=sc3;
                        end
                    end
                else
                    t=cumsum(tetazT(1,:));
                    if ~init_flag && abs(t)>10*pi/180 && sum(tetazT(2))==0 && sum(tetazT(3))==0
                        v1=sign(t)*sc3;tetazT(3)=1;
                    end
                end
            end
            
        end

        B_old = B;
        M_old=M;
        yaw_old = yaw;
        h1_old = h1;
        n3_old = n3;
        M12_old = M12;
        if init_flag
            init_flag = false;
        end
        waitfor(r);
        imshow(I);
    end
end
