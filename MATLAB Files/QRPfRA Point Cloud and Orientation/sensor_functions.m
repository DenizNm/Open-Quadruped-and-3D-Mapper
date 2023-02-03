classdef sensor_functions
    methods (Static)
        function quaternion_list = get_orientation_data(sensor_csv_path)
            SampleRate = 100;
            sensor_matrix = readmatrix(sensor_csv_path);
            Accelerometer = -9.8.*[sensor_matrix(:,8), sensor_matrix(:,7), -sensor_matrix(:,9)];
            Gyroscope = [sensor_matrix(:,5), sensor_matrix(:,4), -sensor_matrix(:,6)];
            Magnetometer = [sensor_matrix(:,15), sensor_matrix(:,14), -sensor_matrix(:,16)];

            Accelerometer = sensor_functions.filter_data(Accelerometer);
            Gyroscope = sensor_functions.filter_data(Gyroscope);
            Magnetometer = sensor_functions.filter_data(Magnetometer);

            orientFilt = ahrsfilter('SampleRate', SampleRate,'ReferenceFrame','NED');

            reset(orientFilt);
            qEst = orientFilt(Accelerometer,Gyroscope,Magnetometer);

            quaternion_list = qEst;
        end

        function filtered_data = filter_data(data)
            %For savgol filter
            order = 3;
            framelen = 55;
            data = smoothdata(data, "rloess");
            data = sgolayfilt(data,order,framelen);
            filtered_data = smoothdata(data, "rlowess");

        end

        function rotated_acc_data = invert_qest_list(qest, normalized_acc_data)
            %syms tetha phi
            qparts = compact(qest);
            inverted = quatinv(qparts); %Returns inverted quaternion
            %           inverse_angles = quat2eul(quaternion(inverted), "ZYX");
            %           rX = normalized_acc_data(1,1);
            %           rY = normalized_acc_data(1,2);
            %           rZ = normalized_acc_data(1,3);
            %           tetha = inverse_angles(1,1);
            %           phi = inverse_angles(1,3);
            %           rotated_acc_data = [((rX*cos(tetha)) + (rY*sin(tetha)*sin(phi)) + (rZ*sin(tetha)*cos(phi))) ((rZ*cos(tetha)) + (rX*sin(tetha)*sin(phi)) + (rY*sin(tetha)*cos(phi))) ((rY*cos(tetha)) + (rZ*sin(tetha)*sin(phi)) + (rX*sin(tetha)*cos(phi)))];
            rotated_acc_data = quatrotate(inverted, normalized_acc_data);
        end

        function [loc_change, time] = location_change(sensor_CSV, file_num)
            projectdir = sensor_CSV;
            dinfo = dir(fullfile(projectdir));
            dinfo([dinfo.isdir]) = [];
            nfiles = length(dinfo); %Lenght of the directory

            SampleRate = 100;
            sensor_matrix = readmatrix(fullfile(projectdir, dinfo(file_num).name));%readmatrix("/Users/deniz/Desktop/sensor_data_(8).csv");%

            %%%Just find the right orientation and make magic
            Accelerometer = -9.8.*[sensor_matrix(:,8), sensor_matrix(:,7), -sensor_matrix(:,9)];
            Gyroscope = [sensor_matrix(:,5), sensor_matrix(:,4), -sensor_matrix(:,6)];
            Magnetometer = [sensor_matrix(:,15), sensor_matrix(:,14), -sensor_matrix(:,16)];
            UserAcceleration = -9.8.*[sensor_matrix(:,18), sensor_matrix(:,17), -sensor_matrix(:,19)];

            filtered_Accelerometer = sensor_functions.filter_data(Accelerometer);
            filtered_Magnetometer = sensor_functions.filter_data(Magnetometer);
            filtered_Gyroscope = sensor_functions.filter_data(Gyroscope);
            %filtered_UserAcceleration = sensor_functions.filter_data(UserAcceleration);

            %Filtered Data
            Accelerometer = filtered_Accelerometer;
            Magnetometer = filtered_Magnetometer;
            Gyroscope = filtered_Gyroscope;
            %UserAcceleration = filtered_UserAcceleration;

            q = ecompass(Accelerometer, Magnetometer, "quaternion","ReferenceFrame","NED");
            orientFilt = ahrsfilter('SampleRate', SampleRate,'ReferenceFrame','NED');
            %           tc = tunerconfig('ahrsfilter', "MaxIterations", 64, ...
            %               'ObjectiveLimit', 0.001, 'Display', 'none');
            reset(orientFilt);

            %Estimated Quaternion list for given set stating orientation of the device
            qEst = orientFilt(Accelerometer,Gyroscope,Magnetometer);

            normalized_acc_data = sensor_functions.invert_qest_list(qEst, UserAcceleration);
            sz = size(sensor_matrix(:,1),1);
            dt = transpose(linspace(0, sz/100, sz));

            velocity_x = cumtrapz(normalized_acc_data(:,2), dt);
            velocity_x_new = detrend(velocity_x);
            location_array_x = cumtrapz(velocity_x_new, dt);
            loc_X = sum(location_array_x);

            velocity_y = cumtrapz(normalized_acc_data(:,1), dt);
            velocity_y_new = detrend(velocity_y);
            location_array_y = cumtrapz(velocity_y_new, dt);
            loc_Y = sum(location_array_y);

            velocity_z = cumtrapz(normalized_acc_data(:,3), dt);
            velocity_z_new = detrend(velocity_z);
            location_array_z = cumtrapz(velocity_z_new, dt);
            loc_Z = sum(location_array_z);

            loc_change = [-loc_X, loc_Y, loc_Z];
            time = dt(end);
        end

        function [change_between_takes_list, time_of_arrival] = return_change_list(projectdir)
            dinfo = dir(fullfile(projectdir));
            dinfo([dinfo.isdir]) = [];     %get rid of all directories including . and ..
            nfiles = length(dinfo);
            %Defining Origin
            intial_point = [0,0,0];
            %In terms of meter
            loc_change = zeros([nfiles,3]);
            time = zeros([nfiles,1]);
            loc_change(1,:) = intial_point;
            for i = 2:nfiles
                [loc_change(i,:), time(i,:)] = sensor_functions.location_change(projectdir,i);
            end
            change_between_takes_list = loc_change;
            time_of_arrival = time;
        end

        function summed_list = return_sum_list(location_change_list,time_list)
            sum_list = zeros([size(location_change_list)]);
            buffer_time_list = zeros([length(time_list),1]);
            for i = 1:length(location_change_list)
                sum_list(i,:) = sum(location_change_list(1:i,:),1);
                buffer_time_list(i,:) = sum(time_list(1:i,1),1);
            end
            %In here I decided to not sum Z-axis because particularly
            %Z-axis is much error prone compared to X and Y axes. In the
            %future when accelerometer data is better handled it can be
            %summed again.
            sum_list(:,3) = location_change_list(:,3);
            summed_list = cat(2,sum_list,buffer_time_list);
        end

        function [waypoint_trajectory, coordinate_with_time_of_arrival] = return_waypoints_and_coordinate_with_time(projectdir)
            [delta_loc_list,time_list] = sensor_functions.return_change_list(projectdir);
            summed = sum(delta_loc_list)
            sprintf("Summed value for net change = X: %d (m), Y: %d (m), Z:%d (m)", summed(1,1),summed(1,2),summed(1,3))
            coordinate_with_time_of_arrival = sensor_functions.return_sum_list(delta_loc_list,time_list);
            waypoint_trajectory = waypointTrajectory(coordinate_with_time_of_arrival(:,1:3), "TimeOfArrival",coordinate_with_time_of_arrival(:,4))
        end

        function kinematic_trajectory = kinematic_trajectory_deneme(sensor_CSV, file_num)
            %Uses MATLAB kinematic trajectory function to determine both
            %trajectory and orientation
            projectdir = sensor_CSV;
            dinfo = dir(fullfile(projectdir));
            dinfo([dinfo.isdir]) = [];
            nfiles = length(dinfo); %Lenght of the directory

            SampleRate = 100;
            sensor_matrix = readmatrix(fullfile(projectdir, dinfo(file_num).name));%readmatrix("/Users/deniz/Desktop/sensor_data_(8).csv");%

            %%%Just find the right orientation and make magic
            Accelerometer = -9.8.*[sensor_matrix(:,8), sensor_matrix(:,7), -sensor_matrix(:,9)];
            Gyroscope = [sensor_matrix(:,5), sensor_matrix(:,4), -sensor_matrix(:,6)];
            Magnetometer = [sensor_matrix(:,15), sensor_matrix(:,14), -sensor_matrix(:,16)];
            UserAcceleration = -9.8.*[sensor_matrix(:,18), sensor_matrix(:,17), -sensor_matrix(:,19)];

            filtered_Accelerometer = sensor_functions.filter_data(Accelerometer);
            filtered_Magnetometer = sensor_functions.filter_data(Magnetometer);
            filtered_Gyroscope = sensor_functions.filter_data(Gyroscope);
            filtered_UserAcceleration = sensor_functions.filter_data(UserAcceleration);

            %Filtered Data
            Accelerometer = filtered_Accelerometer;
            Magnetometer = filtered_Magnetometer;
            Gyroscope = filtered_Gyroscope;
            UserAcceleration = filtered_UserAcceleration;

            q = ecompass(Accelerometer, Magnetometer, "quaternion","ReferenceFrame","NED");
            orientFilt = ahrsfilter('SampleRate', SampleRate,'ReferenceFrame','NED');
            %           tc = tunerconfig('ahrsfilter', "MaxIterations", 64, ...
            %               'ObjectiveLimit', 0.001, 'Display', 'none');
            reset(orientFilt);

            %Estimated Quaternion list for given set stating orientation of the device
            qEst = orientFilt(Accelerometer,Gyroscope,Magnetometer);

            normalized_acc_data = sensor_functions.invert_qest_list(qEst, UserAcceleration);

            kin_trajectory = kinematicTrajectory("SampleRate", SampleRate);
            position = kin_trajectory(UserAcceleration,Gyroscope); %,orientation,velocity,acceleration,angularVelocity
            kinematic_trajectory = position;
        end

        function kinematic_list = return_kinematic_traj(projectdir)
            %Uses MATLAB kinematic trajectory function to determine both
            %trajectory and orientation. NOTE: Additional function for
            %kinematic trajectory example
            dinfo = dir(fullfile(projectdir));
            dinfo([dinfo.isdir]) = [];     %get rid of all directories including . and ..
            nfiles = length(dinfo);
            %Defining Origin
            intial_point = [0,0,0];
            %In terms of meter
            loc_change(1,:) = intial_point;
            count = 1;
            for i = 2:nfiles
                change = sensor_functions.kinematic_trajectory_deneme(projectdir,i);
                for j = 1:length(change)
                    loc_change(count,:) = change(j,:);
                    count = count + 1;
                end
            end
            kinematic_list = loc_change;
        end

        function summed_list = return_kinematic_traj_sum_list(kinematic_list)
            %Uses MATLAB kinematic trajectory function to determine both
            %trajectory and orientation. NOTE: Additional function for
            %kinematic trajectory example
            sum_list = zeros([length(kinematic_list),3]);
            for i = 1:length(kinematic_list)
                sum_list(i,:) = sum(kinematic_list(1:i,:),1);
            end
            summed_list = sum_list;
        end

    end
end

