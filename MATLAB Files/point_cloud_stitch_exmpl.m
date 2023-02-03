projectdir = '.../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_pointClouds';
dinfo = dir(fullfile(projectdir));
dinfo([dinfo.isdir]) = [];     %get rid of all directories including . and ..
nfiles = length(dinfo);

sensor_dir = '.../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_sensorCSV';
sensor_info = dir(fullfile(sensor_dir));
sensor_info([sensor_info.isdir]) = [];     %get rid of all directories including . and ..
sensorfiles = length(sensor_info);

image_dir = '.../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_image_dataset';

%%%%poses = get_absolute_poses_functions.get_abs_poses_from_sensor_fusion(sensor_dir);%%get_absolute_poses_functions.get_abs_pose(image_dir);
%%%%tforms = cell2table(poses);
%%%%tforms = tforms.poses;
quaternions = get_absolute_poses_functions.get_quaternions(sensor_dir);
quats = quaternions.quat_list;

%%%%%Get point clouds converted if necessary.
ptCloudScans = point_cloud_functions.get_point_cloud_array(projectdir);
ptClouds = ptCloudScans.Views.PointCloud;


[waypointTrajectory, coordinate_with_time_of_arrival] = sensor_functions.return_waypoints_and_coordinate_with_time(sensor_dir);
position = waypointTrajectory.Waypoints;
orientation = quats;%waypointTrajectory.Orientation;%

degree = [0,270,0];
perp_quat = eul2quat(deg2rad(degree), "ZYX");
aff_rot_obj = affine3d(quat2tform(perp_quat));

for i = 1:length(ptClouds)
    [row_count, col_count] = size(ptClouds(i).Location);
    new_cloud = zeros([row_count, col_count]);
    qparts = compact(orientation(i));
    ptClouds(i) = pctransform(ptClouds(i),aff_rot_obj);
    ptClouds(i) = pctransform(ptClouds(i),affine3d(quat2tform(orientation(i))));
    for j = 1:length(ptClouds(i).Location)
        new_cloud(j,1) = position(i,1)/2 + ptClouds(i).Location(j,1);
        new_cloud(j,2) = position(i,2)/2 + ptClouds(i).Location(j,2);
        new_cloud(j,3) = position(i,3)/2 + ptClouds(i).Location(j,3);
    end
    ptClouds(i) = pointCloud(new_cloud); 
end

% i = 9
% qparts = compact(orientation(i));
% ptClouds(i) = quatrotate(qparts, ptClouds(i).Location);
% pcshow(ptClouds(i))
 
% hold on
% for i = 1:length(ptClouds)
%     pcshow(ptClouds(i))
% end

new_clouds = pccat(ptClouds);
pcshow(new_clouds)


% grid_size = 100;
% grid_res = 0.1;
% merge_size = 0.001;
% dw_rate= 0.8;
% 
% cloud_list = ptClouds;
% 
% for i = 1:10
%     disp("started")
%     cloud_list = point_cloud_functions.get_incremental_map(cloud_list,merge_size,grid_size,grid_res,quats)
%     disp("done")
% end
% 
% pcshow(cat(cloud_list.Views.PointCloud))

% hold on
% for i = 1:length(cloud_list.Views.PointCloud)
%     pcshow(cloud_list.Views.PointCloud(i))
% end


