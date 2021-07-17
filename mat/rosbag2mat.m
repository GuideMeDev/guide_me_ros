
exampleHelperROSLoadMessages;
bag = rosbag(bag_file);

bagCameraInfo = select(bag,'Topic','/camera/color/camera_info');
cameraInfo = readMessages(bagCameraInfo,'DataFormat','sensor_msgs/CameraInfo');

bagImu = select(bag,'Topic','/imu');
imu = readMessages(bagImu,'DataFormat','sensor_msgs/Imu');

bagImage = select(bag,'Topic','camera/color/image_raw');
Image_msg = readMessages(bagImage,'DataFormat','sensor_msgs/Image');
Image = cell(size(Image_msg,1),1);
for ii = 1:size(Image_msg,1)
    Image(ii) = {readImage(Image_msg{ii})};
end

bagCompressedImage = select(bag,'Topic','camera/color/image_raw/compressed');
compressedImage_msg = readMessages(bagCompressedImage,'DataFormat','sensor_msgs/CompressedImage');
compressedImage = cell(size(compressedImage_msg,1),1);
for ii = 1:size(compressedImage_msg,1)
    compressedImage(ii) = {readImage(compressedImage_msg{ii})};
end

bagCameraDepthInfo = select(bag,'Topic','/camera/depth/camera_info');
cameraDepthInfo = readMessages(bagCameraDepthInfo,'DataFormat','sensor_msgs/CameraInfo');

bagDepthImage = select(bag,'Topic','camera/depth/image_rect_raw');
depthImage_msg = readMessages(bagDepthImage,'DataFormat','struct');
depthImage = cell(size(depthImage_msg,1),1);
for ii = 1:size(depthImage_msg,1)
     depthImage_msg{ii}.Data = reshape(typecast(depthImage_msg{ii}.Data, 'uint16'), depthImage_msg{ii}.Width, depthImage_msg{ii}.Height);
     depthImage(ii) = {depthImage_msg{ii}.Data};
end

bagPcl2 = select(bag,'Topic','/camera/depth/color/points');
pcl2_msg = readMessages(bagPcl2,'DataFormat','struct');
pcl2 = cell(size(pcl2_msg,1),1);
for ii = 1:100  % size(pcl2,1)
    pcl2{ii} = rosmessage('sensor_msgs/PointCloud2');
    pcl2{ii}.Header.Seq = pcl2_msg{ii}.Header.Seq;
    pcl2{ii}.Header.Stamp.Nsec = pcl2_msg{ii}.Header.Stamp.Nsec;
    pcl2{ii}.Header.Stamp.Sec = pcl2_msg{ii}.Header.Stamp.Sec;
    pcl2{ii}.Header.FrameId = pcl2_msg{ii}.Header.FrameId;
    
    pcl2{ii}.Height = pcl2_msg{ii}.Height;
    pcl2{ii}.Width = pcl2_msg{ii}.Width;
    
    pf = cell(size(pcl2_msg{ii}.Fields,2),1);
    Name = {pcl2_msg{ii}.Fields.Name};
    Offset = {pcl2_msg{ii}.Fields.Offset};
    Datatype = {pcl2_msg{ii}.Fields.Datatype};
    Count = {pcl2_msg{ii}.Fields.Count};
    for jj = 1:size(pcl2_msg{ii}.Fields,2)
        pf{jj} = rosmessage('sensor_msgs/PointField');       
        pf{jj}.Name = Name{jj};
        pf{jj}.Offset = Offset{jj};
        pf{jj}.Datatype = Datatype{jj};
        pf{jj}.Count = Count{jj};
        pcl2{ii}.Fields(end+1) = pf{jj};
    end
    
    pcl2{ii}.IsBigendian = pcl2_msg{ii}.IsBigendian;
    pcl2{ii}.PointStep = pcl2_msg{ii}.PointStep;
    pcl2{ii}.RowStep = pcl2_msg{ii}.RowStep;
    pcl2{ii}.Data = pcl2_msg{ii}.Data;
    pcl2{ii}.IsDense = pcl2_msg{ii}.IsDense;
end

% imshow(Image)
% 
% imshow(compressedImage)
% 
% imshow(depthImage)
%
% figure
% scatter3(pcl2{50})
% 
% quat = [imu{1}.Orientation.W imu{1}.Orientation.Y imu{1}.Orientation.Y imu{1}.Orientation.Z];
% eulZYX = quat2eul(quat,'ZYX');
% eulZYX

