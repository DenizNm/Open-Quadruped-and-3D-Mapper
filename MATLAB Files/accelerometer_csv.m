% quat  =  quaternion(0.7190231680870056, 0.6798363924026489, 0.0744403675198555 ,0.12364043295383453)
% angles = -euler(quat,'ZXY','frame')
clear all
clc

csv_file = readmatrix('/Users/deniz/Desktop/sensor_data_(0).csv');

sz = size(csv_file,1);

acc_x = zeros(sz,1);
acc_y = zeros(sz,1);
acc_z = zeros(sz,1);
acc_list = zeros(sz,3);
sqrt(-1)
for i = 1:sz
    x = csv_file(i, 9);
    y = csv_file(i, 10);
    z = csv_file(i, 11);
    w = csv_file(i, 12);

    quat_array = [w x y z];
    quat = quaternion(w, x, y, z);
    %conj = conj(quat)
    inverted  = quatinv([w x y z]);
    %inverted = quaternion(inverted)
    %angles = -euler(quat,'ZXY','frame');
    %new_quat = quaternion(angles,'euler','ZXY');

    inverted_quat = quaternion(inverted(1),inverted(2),inverted(3),inverted(4));
    %%%
    acc_array = [csv_file(i,7), csv_file(i,8), csv_file(i,9)];
    %inverse_rotated_point = rotatepoint(inverted_quat, acc_array);
    inverse_rotated_point = rotatepoint(inverted_quat, acc_array);
    %acc_list(i) = inverse_rotated_point

    for j = 1:3
        acc_list(i,j) = inverse_rotated_point(j);
    end

end

for k = 1:sz
        acc_x(k) = acc_list(k, 2);
        acc_y(k) = acc_list(k, 3);
        acc_z(k) = acc_list(k, 1);
end

dt = transpose(linspace(0, sz/100, sz));

velocity_x = cumtrapz(acc_x, dt);
velocity_x_new = detrend(velocity_x);
location_array_x = cumtrapz(velocity_x_new, dt);
loc_X = sum(location_array_x)*100


velocity_y = cumtrapz(acc_y, dt);
velocity_y_new = detrend(velocity_y);
location_array_y = cumtrapz(velocity_y_new, dt);
loc_Y = sum(location_array_y)*100


velocity_z = cumtrapz(acc_z, dt);
velocity_z_new = detrend(velocity_z);
location_array_z = cumtrapz(velocity_z_new, dt);
loc_Z = sum(location_array_z)*100


