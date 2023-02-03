classdef get_absolute_poses_functions
    methods (Static)
        function absolute_poses = get_abs_pose(image_dir) %#codegen
            dinfo = dir(fullfile(image_dir));
            dinfo([dinfo.isdir]) = [];
            nfiles = length(dinfo);

            I = im2gray(imread(fullfile(image_dir, dinfo(1).name)));
            pointsPrev = detectSURFFeatures(I);
            [featuresPrev,pointsPrev] = extractFeatures(I,pointsPrev);

            vSet = imageviewset;
            vSet = addView(vSet,1,'Points',pointsPrev);

            for i = 2:nfiles
                I = im2gray(imread(fullfile(image_dir, dinfo(i).name)));
                points = detectSURFFeatures(I);
                [features,points] = extractFeatures(I,points);
                vSet = addView(vSet,i,'Features',features,'Points',points);
                pairsIdx = matchFeatures(featuresPrev,features);
                vSet = addConnection(vSet,i-1,i,'Matches',pairsIdx);
                featuresPrev = features;
            end

            absolute_poses = poses(vSet);
        end

        function poses = get_abs_poses_from_sensor_fusion(sensor_dir) %#codegen
            sensor_info = dir(fullfile(sensor_dir));
            sensor_info([sensor_info.isdir]) = [];
            sensorfiles = length(sensor_info);
            tform_list = cell(sensorfiles,1);

            first_element = sensor_functions.get_orientation_data(fullfile(sensor_dir, sensor_info(2).name));
            tform_list{1,1} = affine3d(quat2tform(first_element(1)));%affine3d(quat2tform(get_absolute_poses_functions.inverted_quaternion(first_element(1))));

            for i = 2:(sensorfiles)
                q_list = sensor_functions.get_orientation_data(fullfile(sensor_dir, sensor_info(i).name));
                inverted = get_absolute_poses_functions.inverted_quaternion(q_list(end));
                %tform_of_last_q = quat2tform(q_list(end));
                tform_of_last_q = affine3d(quat2tform(inverted));%affine3d(tform_of_last_q);
                tform_list{i,1} = tform_of_last_q;
            end
            poses = tform_list;
        end

        function quaternion_table = get_quaternions(sensor_dir) %#codegen
            sensor_info = dir(fullfile(sensor_dir));
            sensor_info([sensor_info.isdir]) = [];
            sensorfiles = length(sensor_info);
            quat_list = cell(sensorfiles,1);

            first_element = sensor_functions.get_orientation_data(fullfile(sensor_dir, sensor_info(2).name));
            quat_list{1,1} = get_absolute_poses_functions.inverted_quaternion(first_element(1));%first_element(end-3);%

            for i = 2:(sensorfiles)
                q_list = sensor_functions.get_orientation_data(fullfile(sensor_dir, sensor_info(i).name));
                inverted = get_absolute_poses_functions.inverted_quaternion(q_list(end));
                %tform_of_last_q = quat2tform(q_list(end));
                %tform_of_last_q = affine3d(quat2tform(inverted));%affine3d(tform_of_last_q);
                quat_list{i,1} = inverted;
            end
            quaternion_table = cell2table(quat_list);
        end

        function inv_quaternion = inverted_quaternion(quat) %#codegen
            qparts = compact(quat);
            inverted  = quatinv(qparts);
            inv_quaternion = quaternion(inverted(:,1),inverted(:,2),inverted(:,3),inverted(:,4));
        end
    end
end












