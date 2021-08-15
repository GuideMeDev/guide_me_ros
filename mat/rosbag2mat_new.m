import robotics.*
bag_file = '2021-08-12-14-20-12.bag';
bag = rosbag(bag_file);

bagImu_madgwick = select(bag,'Topic','/imu/madgwick');
bagImage = select(bag,'Topic','/camera/color/image_raw');
bagPcl2 = select(bag,'Topic','/camera/depth/color/points');

% roll = [];
% pitch = [];
% yaw = [];
% quat = [];
% I = [];
% XYZ = [];
jj_imu_old = 0;
jj_image_old = 0;
jj_pcl_old = 0;
kk = 0;
d = 3;
min_ii = min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1));
roll = cell(floor(min_ii/d), 1);
pitch = cell(floor(min_ii/d), 1);
yaw = cell(floor(min_ii/d), 1);
quat = cell(floor(min_ii/d), 1);
I = cell(floor(min_ii/d), 1);
XYZ = cell(floor(min_ii/d), 1);
for ii=d:d:min_ii
    if size(bagPcl2.MessageList,1) == min_ii
        pcl2_msg = readMessages(bagPcl2, [ii], 'DataFormat','sensor_msgs/PointCloud2');
        imu_madgwick = readMessages(bagImu_madgwick,[ii], 'DataFormat','sensor_msgs/Imu');
        Image_msg = readMessages(bagImage, [ii], 'DataFormat', 'sensor_msgs/Image');
        
        t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
        t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
        t_image = Image_msg{1}.Header.Stamp.Nsec;        
        
        jj_imu_positive = ii;
        t_imu_old_positive = t_imu;
        in_while = false;
        while abs(t_imu - t_pcl) < abs(t_imu_old_positive - t_pcl) && jj_imu_positive < ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*2+ii && jj_imu_positive <= size(bagImu_madgwick.MessageList,1)
            in_while = true;
            jj_imu_positive = jj_imu_positive + 1;
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_positive], 'DataFormat','sensor_msgs/Imu');
            t_imu_old_positive = t_imu;
            t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_imu_positive = jj_imu_positive - 1;
        end
        
        jj_imu_negative = ii;
        t_imu_old_negative = t_imu;
        in_while = false;
        while abs(t_imu - t_pcl) < abs(t_imu_old_negative - t_pcl) && jj_imu_negative > ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*(-2)+ii && jj_imu_negative >= 1
            in_while = true;
            jj_imu_negative = jj_imu_negative - 1;
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_negative], 'DataFormat','sensor_msgs/Imu');
            t_imu_old_negative = t_imu;
            t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_imu_negative = jj_imu_negative + 1;
        end
        
        if abs(t_imu_old_positive - t_pcl) <= abs(t_imu_old_negative - t_pcl)
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_positive], 'DataFormat','sensor_msgs/Imu');
        else
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_negative], 'DataFormat','sensor_msgs/Imu');
        end
        
        jj_image_positive = ii;
        t_image_old_positive = t_image;
        in_while = false;
        while abs(t_image - t_pcl) < abs(t_image_old_positive - t_pcl) && jj_image_positive < ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*2+ii && jj_image_positive <= size(bagImage.MessageList,1)
            in_while = true;
            jj_image_positive = jj_image_positive + 1;
            Image_msg = readMessages(bagImage,[jj_image_positive], 'DataFormat','sensor_msgs/Image');
            t_image_old_positive = t_image;
            t_image = Image_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_image_positive = jj_image_positive - 1;
        end
        
        jj_image_negative = ii;
        t_image_old_negative = t_image;
        in_while = false;
        while abs(t_image - t_pcl) < abs(t_image_old_negative - t_pcl) && jj_image_negative > ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*(-2)+ii && jj_image_negative >= 1
            in_while = true;
            jj_image_negative = jj_image_negative - 1;
            Image_msg = readMessages(bagImage,[jj_image_negative], 'DataFormat','sensor_msgs/Image');
            t_image_old_negative = t_image;
            t_image = Image_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_image_negative = jj_image_negative + 1;
        end
        
        if abs(t_image_old_positive - t_pcl) <= abs(t_image_old_negative - t_pcl)
            Image_msg = readMessages(bagImage,[jj_image_positive], 'DataFormat','sensor_msgs/Image');
        else
            Image_msg = readMessages(bagImage,[jj_image_negative], 'DataFormat','sensor_msgs/Image');
        end
        
        quat{ii/d} = [imu_madgwick{1}.Orientation.W imu_madgwick{1}.Orientation.X imu_madgwick{1}.Orientation.Y imu_madgwick{1}.Orientation.Z];
        eulXYZ = quat2eul(quat{ii/d},'XYZ');
        roll{ii/d} = eulXYZ(2);
        pitch{ii/d} = -eulXYZ(1);
        yaw{ii/d} = eulXYZ(3);
        
        I{ii/d-kk} = readImage(Image_msg{1,1});
        
        XYZ{ii/d-kk} = readXYZ(pcl2_msg{1,1});
        
    elseif size(bagImage.MessageList,1) == min_ii
        pcl2_msg = readMessages(bagPcl2, [ii], 'DataFormat','sensor_msgs/PointCloud2');
        imu_madgwick = readMessages(bagImu_madgwick,[ii], 'DataFormat','sensor_msgs/Imu');
        Image_msg = readMessages(bagImage, [ii], 'DataFormat', 'sensor_msgs/Image');
        
        t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
        t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
        t_image = Image_msg{1}.Header.Stamp.Nsec;        
        
        jj_imu_positive = ii;
        t_imu_old_positive = t_imu;
        in_while = false;
        while abs(t_imu - t_image) < abs(t_imu_old_positive - t_image) && jj_imu_positive < ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*2+ii && jj_imu_positive <= size(bagImu_madgwick.MessageList,1)
            in_while = true;
            jj_imu_positive = jj_imu_positive + 1;
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_positive], 'DataFormat','sensor_msgs/Imu');
            t_imu_old_positive = t_imu;
            t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_imu_positive = jj_imu_positive - 1;
        end
        
        jj_imu_negative = ii;
        t_imu_old_negative = t_imu;
        in_while = false;
        while abs(t_imu - t_image) < abs(t_imu_old_negative - t_image) && jj_imu_negative > ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*(-2)+ii && jj_imu_negative >= 1
            in_while = true;
            jj_imu_negative = jj_imu_negative - 1;
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_negative], 'DataFormat','sensor_msgs/Imu');
            t_imu_old_negative = t_imu;
            t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_imu_negative = jj_imu_negative + 1;
        end
        
        if abs(t_imu_old_positive - t_image) <= abs(t_imu_old_negative - t_image)
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_positive], 'DataFormat','sensor_msgs/Imu');
        else
            imu_madgwick = readMessages(bagImu_madgwick,[jj_imu_negative], 'DataFormat','sensor_msgs/Imu');
        end
        
        jj_pcl_positive = ii;
        t_pcl_old_positive = t_pcl;
        in_while = false;
        while abs(t_pcl - t_image) < abs(t_pcl_old_positive - t_image) && jj_pcl_positive < ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*2+ii && jj_pcl_positive <= size(bagPcl2.MessageList,1)
            in_while = true;
            jj_pcl_positive = jj_pcl_positive + 1;
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_positive], 'DataFormat','sensor_msgs/PointCloud2');
            t_pcl_old_positive = t_pcl;
            t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_pcl_positive = jj_pcl_positive - 1;
        end
        
        jj_pcl_negative = ii;
        t_pcl_old_negative = t_pcl;
        in_while = false;
        while abs(t_pcl - t_image) < abs(t_pcl_old_negative - t_image) && jj_pcl_negative > ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*(-2)+ii && jj_pcl_negative >= 1
            in_while = true;
            jj_pcl_negative = jj_pcl_negative - 1;
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_negative], 'DataFormat','sensor_msgs/PointCloud2');
            t_pcl_old_negative = t_pcl;
            t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_pcl_negative = jj_pcl_negative + 1;
        end
        
        if abs(t_pcl_old_positive - t_image) <= abs(t_pcl_old_negative - t_image)
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_positive], 'DataFormat','sensor_msgs/PointCloud2');
        else
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_negative], 'DataFormat','sensor_msgs/PointCloud2');
        end
        
        quat{ii/d} = [imu_madgwick{1}.Orientation.W imu_madgwick{1}.Orientation.X imu_madgwick{1}.Orientation.Y imu_madgwick{1}.Orientation.Z];
        eulXYZ = quat2eul(quat{ii/d},'XYZ');
        roll{ii/d} = eulXYZ(2);
        pitch{ii/d} = -eulXYZ(1);
        yaw{ii/d} = eulXYZ(3);
        
        I{ii/d-kk} = readImage(Image_msg{1,1});
        
        XYZ{ii/d-kk} = readXYZ(pcl2_msg{1,1});
        
    elseif size(bagImu_madgwick.MessageList,1) == min_ii
        pcl2_msg = readMessages(bagPcl2, [ii], 'DataFormat','sensor_msgs/PointCloud2');
        imu_madgwick = readMessages(bagImu_madgwick,[ii], 'DataFormat','sensor_msgs/Imu');
        Image_msg = readMessages(bagImage, [ii], 'DataFormat', 'sensor_msgs/Image');
        
        t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
        t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
        t_image = Image_msg{1}.Header.Stamp.Nsec;        
        
        jj_image_positive = ii;
        t_image_old_positive = t_image;
        in_while = false;
        while abs(t_image - t_pcl) < abs(t_image_old_positive - t_pcl) && jj_image_positive < ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*2+ii && jj_image_positive >= size(bagImage.MessageList,1)
            in_while = true;
            jj_image_positive = jj_image_positive + 1;
            Image_msg = readMessages(bagImage,[jj_image_positive], 'DataFormat','sensor_msgs/Image');
            t_image_old_positive = t_image;
            t_image = Image_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_image_positive = jj_image_positive - 1;
        end
        
        jj_image_negative = ii;
        t_image_old_negative = t_image;
        in_while = false;
        while abs(t_image - t_pcl) < abs(t_image_old_negative - t_pcl) && jj_image_negative > ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*(-2)+ii && jj_image_negative >= 1
            in_while = true;
            jj_image_negative = jj_image_negative - 1;
            Image_msg = readMessages(bagImage,[jj_image_negative], 'DataFormat','sensor_msgs/Image');
            t_image_old_negative = t_image;
            t_image = Image_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_image_negative = jj_image_negative + 1;
        end
        
        if abs(t_image_old_positive - t_pcl) <= abs(t_image_old_negative - t_pcl)
            Image_msg = readMessages(bagImage,[jj_image_positive], 'DataFormat','sensor_msgs/Image');
        else
            Image_msg = readMessages(bagImage,[jj_image_negative], 'DataFormat','sensor_msgs/Image');
        end
        
        jj_pcl_positive = ii;
        t_pcl_old_positive = t_pcl;
        in_while = false;
        while abs(t_pcl - t_image) < abs(t_pcl_old_positive - t_image) && jj_pcl_positive < ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*2+ii && jj_pcl_positive <= size(bagPcl2.MessageList,1)
            in_while = true;
            jj_pcl_positive = jj_pcl_positive + 1;
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_positive], 'DataFormat','sensor_msgs/PointCloud2');
            t_pcl_old_positive = t_pcl;
            t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_pcl_positive = jj_pcl_positive - 1;
        end
        
        jj_pcl_negative = ii;
        t_pcl_old_negative = t_pcl;
        in_while = false;
        while abs(t_pcl - t_image) < abs(t_pcl_old_negative - t_image) && jj_pcl_negative > ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*(-2)+ii && jj_pcl_negative >= 1
            in_while = true;
            jj_pcl_negative = jj_pcl_negative - 1;
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_negative], 'DataFormat','sensor_msgs/PointCloud2');
            t_pcl_old_negative = t_pcl;
            t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
        end
        if in_while
            jj_pcl_negative = jj_pcl_negative + 1;
        end
        
        if abs(t_pcl_old_positive - t_image) <= abs(t_pcl_old_negative - t_image)
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_positive], 'DataFormat','sensor_msgs/PointCloud2');
        else
            pcl2_msg = readMessages(bagPcl2,[jj_pcl_negative], 'DataFormat','sensor_msgs/PointCloud2');
        end
        
        quat{ii/d} = [imu_madgwick{1}.Orientation.W imu_madgwick{1}.Orientation.X imu_madgwick{1}.Orientation.Y imu_madgwick{1}.Orientation.Z];
        eulXYZ = quat2eul(quat{ii/d},'XYZ');
        roll{ii/d} = eulXYZ(2);
        pitch{ii/d} = -eulXYZ(1);
        yaw{ii/d} = eulXYZ(3);
        
        I{ii/d-kk} = readImage(Image_msg{1,1});
        
        XYZ{ii/d-kk} = readXYZ(pcl2_msg{1,1});
        
    end    
end

%%
% for ii=1:size(I,1)
%     imshow(I{ii});
%     pause(0.5);
% end


% while (abs(t_imu - t_Image)*1e-9' > 1/15) || (abs(t_imu - t_pcl)*1e-9' > 1/15) || (abs(t_pcl - t_Image)*1e-9' > 1/15)
%         max_s = ceil(max(max(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1)))*2+ii;
%         if jj_imu > max_s || jj_image > max_s || jj_pcl > max_s
%             break
%         end
%         t_min = min(min(t_imu, t_Image), t_pcl);
% 
%         if t_imu == t_min
%             jj_imu = jj_imu + 1;
%             if jj_imu > size(bagImu_madgwick.MessageList,1)
%                 break
%             end
%             imu_madgwick = readMessages(bagImu_madgwick,[jj_imu], 'DataFormat','sensor_msgs/Imu');
%             t_imu = imu_madgwick{1}.Header.Stamp.Nsec;
%         end
%         if t_Image == t_min
%             jj_image = jj_image + 1;
%             if jj_image > size(bagImage.MessageList,1)
%                 break
%             end
%             Image_msg = readMessages(bagImage, [jj_image], 'DataFormat', 'sensor_msgs/Image');
%             t_Image = Image_msg{1}.Header.Stamp.Nsec;
%         end
%         if t_pcl == t_min
%             jj_pcl = jj_pcl + 1;
%             if jj_pcl > size(bagPcl2.MessageList,1)
%                 break
%             end
%             pcl2_msg = readMessages(bagPcl2, [jj_pcl], 'DataFormat','sensor_msgs/PointCloud2');
%             t_pcl = pcl2_msg{1}.Header.Stamp.Nsec;
%         end
%     end
%     
%     if jj_imu_old >= jj_imu || jj_image_old >= jj_image ||jj_pcl_old >= jj_pcl
%         kk = kk + 1;
%         continue
%     end
%     
%     if jj_imu > max_s || jj_image > max_s || jj_pcl > max_s
%         kk = kk + 1;
%         continue
%     end
% 
%     if jj_imu > size(bagImu_madgwick.MessageList,1)
%         kk = kk + 1;
%         continue
%     end
%     if jj_image > size(bagImage.MessageList,1)
%         kk = kk + 1;
%         continue
%     end
%     if jj_pcl > size(bagPcl2.MessageList,1)
%         kk = kk + 1;
%         continue
%     end
% 
%     if ii <= min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))
%         quat{ii/d-kk} = [imu_madgwick{1}.Orientation.W imu_madgwick{1}.Orientation.X imu_madgwick{1}.Orientation.Y imu_madgwick{1}.Orientation.Z];
%         eulXYZ = quat2eul(quat{ii/d-kk},'XYZ');
%         roll{ii/d-kk} = eulXYZ(2);
%         pitch{ii-kk} = -eulXYZ(1);
%         yaw{ii/d-kk} = eulXYZ(3);
% 
%         I{ii/d-kk} = readImage(Image_msg{1,1});
% 
%         XYZ{ii/d-kk} = readXYZ(pcl2_msg{1,1});
%         
%         jj_imu_old = jj_imu;
%         jj_image_old = jj_image;
%         jj_pcl_old = jj_pcl;
%     else
%         break
%     end
%     disp('ii/d: ');
%     disp(ii/d);
%     disp('out of: ');
%     disp(min(min(size(bagImu_madgwick.MessageList,1), size(bagImage.MessageList,1)), size(bagPcl2.MessageList,1))/d-kk);
