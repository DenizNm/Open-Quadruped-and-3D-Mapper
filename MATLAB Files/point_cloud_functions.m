classdef point_cloud_functions
    %Point Cloud Backend
    %   Used for preprocessing and stitching point clouds

    methods(Static)
        %%Converting unorganized point cloud to meters and organized point
        %%cloud.
        function converted_point_cloud = convert_point_cloud(point_cloud)
            %%Possible to convert with just using converted_point_cloud but it
            %%still works so no problem. Below parts used when point cloud data
            %%created with linear mapping of angles but if angle mapping achieved
            %%with trigonometric functions they are not necessary
            point_cloud(:,3) = -0.01.*point_cloud(:,3);%-0.01.*point_cloud(:,3);
            point_cloud(:,1) = -0.01.*point_cloud(:,1);%-0.01.*point_cloud(:,1);
            point_cloud(:,2) = 0.01.*point_cloud(:,2);
            %point_cloud = 0.01.*point_cloud;
            %Create organized point cloud with resahping the 307200 piece.

            %point_cloud = reshape(point_cloud, [480, 640, 3]);

            %disp(length(point_cloud(:,2)))

            %%%Clearing points att the point cloud origin
            diff_array = zeros(length(point_cloud(:,2)),3);
            for i = 1:length(point_cloud(:,2))
                net_dist = sqrt(sum(point_cloud(i,:).^2));
                if (net_dist < 0.3)
                    diff_array(i,:) = point_cloud(i,:);
                end
            end
            point_cloud = setdiff(point_cloud, diff_array, "rows");

            point_cloud = pointCloud(point_cloud);
            point_cloud = pcdenoise(point_cloud, "PreserveStructure",true,"NumNeighbors",32,"Threshold",0.001);
            converted_point_cloud = point_cloud;
        end

        %%%%%%%%BU FONKSİYONUN 2X'Lİ HALİNİ DENE%%%%%%%%
        function rotated_point_cloud = rotate_pc(pointcloud, est_quaternion)
            angles = 2.*(eulerd(est_quaternion,"ZXY", "point"));
            rad_deg = deg2rad(angles);
            db_quat = eul2quat(rad_deg, "ZYX");%quaternion(angles,"eulerd","ZXY","point");
            %[w,x,y,z] = parts(db_quat);
            qparts = compact(quaternion(db_quat));
            inverted  = quatinv(db_quat);%quatinv([w, x, y, z]);
            %inverted_quat = quaternion(inverted(:,1),inverted(:,2),inverted(:,3),inverted(:,4));
            %                         inv_rot_mtx = quat2tform(inverted_quat(length(inverted_quat)));
            %                         tform = affine3d(inv_rot_mtx);
            %                         rotated_point_cloud = pctransform(pointcloud,tform);
            %
            for i = 1:size(pointcloud,1)
                pointcloud(i,:) = quatrotate(inverted, pointcloud.Location);
            end

            %pointcloud = point_cloud_functions.convert_point_cloud(pointcloud);

            %mtx = eul2tform([pi/4 pi/2 pi/4]);
            %%inv_rot_mtx = quat2tform(est_quaternion(length(inverted_quat)));
            %%tform = affine3d(inv_rot_mtx);%quat2tform(est_quaternion));
            %%pointcloud = pctransform(pointcloud, tform);
            rotated_point_cloud = pointcloud;
        end

        function point_clouds = get_point_cloud_array(project_dir)
            dinfo = dir(fullfile(project_dir));
            dinfo([dinfo.isdir]) = [];
            nfiles = length(dinfo);
            %pc_size = size(dinfo);

            pcSet = pcviewset;
            for i = 1:nfiles
                pc_buffer = readmatrix(fullfile(project_dir, dinfo(i).name));
                pc_buffer = point_cloud_functions.convert_point_cloud(pc_buffer);
                pcSet = addView(pcSet,i, "PointCloud", pc_buffer);
            end
            point_clouds = pcSet;
        end

        function created_pcSet = get_incremental_map(input_ptCld_set, merge_size, gridSize, gridStep, quatlist)
            set_class = class(input_ptCld_set);
            if set_class == "pointCloud"
                pcSet = pcviewset;
                merge_size = 0.001;
                for i = 1:(length(input_ptCld_set)-1)
                    fixed = input_ptCld_set(i);%point_cloud_functions.rotate_pc(input_ptCld_set(i),quatlist(i,:));%
                    moving = input_ptCld_set(i+1);%point_cloud_functions.rotate_pc(input_ptCld_set(i+1),quatlist(i+1,:));%

                    fixed = pcdownsample(fixed,'gridAverage',0.01);
                    moving = pcdownsample(moving,'gridAverage',0.01);

                    tform = pcregistercorr(moving, fixed, gridSize, gridStep);%pcregistercpd(moving,fixed);%pcregistericp(moving, fixed, "Metric","pointToPoint","Extrapolate",false,"MaxIterations",32);%

                    ptCloudAligned = pctransform(moving, tform);

                    ptCloudScene = pcmerge(fixed, ptCloudAligned, merge_size);

                    pcSet = addView(pcSet, i, "PointCloud",ptCloudScene);
                end
                created_pcSet = pcSet;
                disp("Done")

            elseif set_class == "pcviewset"
                newSet = pcviewset;
                for i = 1:(length(input_ptCld_set.Views.PointCloud)-1)
                    fixed = input_ptCld_set.Views.PointCloud(i,:);
                    moving = input_ptCld_set.Views.PointCloud(i+1,:);

                    fixed = pcdownsample(fixed,'gridAverage',0.005);
                    moving = pcdownsample(moving,'gridAverage',0.005);

                    %%TODO: Try different registiration algo.
                    tform = pcregistercorr(moving, fixed, gridSize,gridStep);%pcregistercpd(moving,fixed);%pcregistericp(moving, fixed, "Metric","pointToPoint","Extrapolate",false,"MaxIterations",32);%

                    ptCloudAligned = pctransform(moving, tform);

                    ptCloudScene = pcmerge(fixed, ptCloudAligned, merge_size);

                    newSet = addView(newSet, i, "PointCloud",ptCloudScene);
                end
                created_pcSet = newSet;
                disp("Done")
            end
        end

        function registered_point_cloud = get_map_with_appending(point_list, downsampling_rate, merge_size, quat_list)
            ptCloudRef = point_list(1,:);
            %ptCloudRef = point_cloud_functions.rotate_pc(ptCloudRef, quat_list(1,:));
            ptCloudCurrent = point_list(2,:);
            %ptCloudCurrent = point_cloud_functions.rotate_pc(ptCloudCurrent, quat_list(2,:));

            fixed = pcdownsample(ptCloudRef,'gridAverage',downsampling_rate);
            moving = pcdownsample(ptCloudCurrent,'gridAverage',downsampling_rate);

            tform = pcregistericp(moving,fixed,'Metric','pointToPlane','Extrapolate', true);
            movingReg = pctransform(moving,tform);
            ptCloudScene = pcmerge(ptCloudRef, movingReg, merge_size);

            accumTform = tform;

            figure
            hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
            title('Updated world scene')
            % Set the axes property for faster rendering
            hAxes.CameraViewAngleMode = 'auto';
            hScatter = hAxes.Children;
            for i = 3:length(point_list)
                curent_cloud = point_list(i,:);
                fixed = moving;
                moving = pcdownsample(curent_cloud, 'gridAverage', downsampling_rate);
                %moving = point_cloud_functions.rotate_pc(moving, quat_list(i,:));

                tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

                accumTform = affine3d(tform.T * accumTform.T);
                ptCloudAligned = pctransform(ptCloudCurrent, accumTform);

                ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, merge_size);

                hScatter.XData = ptCloudScene.Location(:,1);
                hScatter.YData = ptCloudScene.Location(:,2);
                hScatter.ZData = ptCloudScene.Location(:,3);
                drawnow('limitrate')
            end

            registered_point_cloud = ptCloudScene;
        end

        function registered_point_cloud = get_map_with_correlation(point_list, downsampling_rate, merge_size, quat_list, grid_size, grid_res)
            ptCloudRef = point_list(1,:);
            %ptCloudRef = point_cloud_functions.rotate_pc(ptCloudRef, quat_list(1,:));
            ptCloudCurrent = point_list(2,:);
            %ptCloudCurrent = point_cloud_functions.rotate_pc(ptCloudCurrent, quat_list(2,:));

            fixed = pcdownsample(ptCloudRef,'gridAverage',downsampling_rate);
            moving = pcdownsample(ptCloudCurrent,'gridAverage',downsampling_rate);

            tform = pcregistercorr(moving,fixed, grid_size, grid_res);
            movingReg = pctransform(moving,tform);
            ptCloudScene = pcmerge(ptCloudRef, movingReg, merge_size);

            accumTform = tform;

            figure
            hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
            title('Updated world scene')
            % Set the axes property for faster rendering
            hAxes.CameraViewAngleMode = 'auto';
            hScatter = hAxes.Children;
            for i = 3:length(point_list)
                curent_cloud = point_list(i,:);
                fixed = moving;
                moving = pcdownsample(curent_cloud, 'gridAverage', downsampling_rate);
                %moving = point_cloud_functions.rotate_pc(moving, quat_list(i,:));

                tform = pcregistercorr(moving, fixed, grid_size, grid_res);

                accumTform = affine3d(tform.T * accumTform.T);
                ptCloudAligned = pctransform(ptCloudCurrent, accumTform);

                ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, merge_size);

                hScatter.XData = ptCloudScene.Location(:,1);
                hScatter.YData = ptCloudScene.Location(:,2);
                hScatter.ZData = ptCloudScene.Location(:,3);
                drawnow('limitrate')
            end

            registered_point_cloud = ptCloudScene;
        end
    end
end

