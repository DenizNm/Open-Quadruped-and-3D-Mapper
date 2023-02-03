csv_file = readmatrix('.../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_pointClouds/19-33-25.csv');
old_cld = pointCloud(csv_file);

deneme = cart2hom(csv_file)
new_cld = pointCloud(deneme(:,1:3));

figure
pcshow(old_cld)

figure
pcshow(new_cld)

% motion_csv = readmatrix('.../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_sensorCSV/16-51-07-10.csv')
% 
% 
% accel_data = [motion_csv(:,7), motion_csv(:,8), motion_csv(:,9)];
% gyro_data = [motion_csv(:,4), motion_csv(:,5), motion_csv(:,6)];
% 
% sz = size(motion_csv,1);
% csv_file(:,3) = -1.*csv_file(:,3);
% csv_file(:,1) = -1.*csv_file(:,1);
% 
% decim=1;
% Fs = 100
% 
% trajectory = kinematicTrajectory
% 
% aFilter = imufilter('SampleRate',Fs,'DecimationFactor',decim, 'ReferenceFrame', 'NED');
% aFilter.GyroscopeNoise          = 7.6154e-7;
% aFilter.AccelerometerNoise      = 0.0015398;
% aFilter.GyroscopeDriftNoise     = 3.0462e-12;
% aFilter.LinearAccelerationNoise = 0.00096236;
% aFilter.InitialProcessNoise     = aFilter.InitialProcessNoise*10;
% %%% Z eksenine göre Xcode da test edip eğer ekran yönönde ivmelenirken
% %%% pozitif değer okuyorsam bu durumda ENU referans koordinatlarını
% %%% kullanmam ancak ekran tarafında negatif bir değer okuyorsam bu durumda
% %%% NED sistemini kullanmam gerekiyor.
% 
% time = (0:decim:size(accel_data,1)-1)/Fs;
% 
% q = aFilter(accel_data,gyro_data);
% 
% [w,x,y,z] = parts(q);
% 
% inverted  = quatinv([w,x,y,z]);
% inverted_quat = quaternion(inverted(:,1),inverted(:,2),inverted(:,3),inverted(:,4));
% 
% inverse_rotated_point = rotatepoint(inverted_quat, accel_data); %accel_data
% 
% 
% velocity_x = cumtrapz(inverse_rotated_point(:,1), time);
% velocity_x_new = detrend(velocity_x);
% location_array_x = cumtrapz(velocity_x_new, time);
% loc_X = sum(location_array_x)*100
% 
% 
% velocity_y = cumtrapz(inverse_rotated_point(:,2), time);
% velocity_y_new = detrend(velocity_y);
% location_array_y = cumtrapz(velocity_y_new, time);
% loc_Y = sum(location_array_y)*100
% 
% 
% velocity_z = cumtrapz(inverse_rotated_point(:,3), time);
% velocity_z_new = detrend(velocity_z);
% location_array_z = cumtrapz(velocity_z_new, time);
% loc_Z = sum(location_array_z)*100
% 
% 
% [position,orientation,velocity,acceleration,angularVelocity] = trajectory(accel_data,gyro_data);
% position = position*100; %%Converting to cm
% 
% pos_arr = [transpose(location_array_x), transpose(location_array_y), transpose(location_array_z)];
% figure(1)
% plot3(position(:,1),position(:,2),position(:,3))
% %plot3(pos_arr(:,1), pos_arr(:,2), pos_arr(:,3))
% hold on;
% %plot3(position(-1,1) - position(1,1), position(-1,2) - position(1,2), position(-1,3) - position(1,3))
% xlabel('North (cm)')
% ylabel('East (cm)')
% zlabel('Down (cm)')
% title('Position from Trajectory')
% % grid on
% % plot(time,eulerd(q,'ZXY','frame'))
% % title('Orientation Estimate')
% % legend('Z-axis', 'Y-axis', 'X-axis')
% % xlabel('Time (s)')
% % ylabel('Rotation (degrees)')
% 
% delta_change = sqrt((position(sz,1) - position(1,1))^2 + (position(sz,2) - position(1,2))^2 + (position(sz,3) - position(1,3))^2)
% delta_X = (position(sz,1) - position(1,1))
% delta_Y = (position(sz,2) - position(1,2))
% delta_Z = (position(sz,3) - position(1,3))
% %pcshow(csv_file)
