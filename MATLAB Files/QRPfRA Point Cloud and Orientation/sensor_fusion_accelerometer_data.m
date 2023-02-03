projectdir = '.../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_sensorCSV';

[wayp, coordinate_with_time_of_arrival] = sensor_functions.return_waypoints_and_coordinate_with_time(projectdir);

% kinematic_list = sensor_functions.return_kinematic_traj(projectdir);
% summed_list = sensor_functions.return_kinematic_traj_sum_list(kinematic_list)
% 
% 
% plot3(summed_list(:,1),summed_list(:,2),summed_list(:,3))
figure
plot3(coordinate_with_time_of_arrival(:,1),coordinate_with_time_of_arrival(:,2),coordinate_with_time_of_arrival(:,3))
axis equal
xlabel("X")
ylabel("Y")
zlabel("Z")

% % kin_traj = sensor_functions.return_kinematic_traj(projectdir);
% % summed = sum(kin_traj)
% % sum_llist = sensor_functions.return_kinematic_traj_sum_list(kin_traj)./100;
% % plot3(sum_llist(:,3), sum_llist(:,2), sum_llist(:,1))
% % xlabel("X")
% % ylabel("Y")
% % zlabel("Z")
