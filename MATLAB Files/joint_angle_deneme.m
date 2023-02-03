time = 0:0.1:5;
angles = [0];

for i = 1:length(time)
    if (i < length(time)/2)
        angles(end+1) = angles(end) + 5;
    elseif (i > (length(time)/2-1))
        angles(end+1) = angles(end) - 5;
    end
end
angles(end) = [];

angle_table = cat(1,time,angles);

dangles = zeros([length(angles), 3]);
dangles(1:2,:) = 0;
dangles(:,3) = transpose(angles);

dangles = deg2rad(dangles);
ang_quat = eul2quat(dangles, "XYZ")

%angles = transpose(angles);

rads = deg2rad(angles);
rad_table = transpose(cat(1,time,rads))
