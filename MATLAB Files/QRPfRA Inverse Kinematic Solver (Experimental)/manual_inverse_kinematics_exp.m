axis equal
hold on

%left = 0
%right = 1

leg = 0;

%for i = -6:0.2:8
%[hind_L, mid_L, wrist_L] = inv_kinematic_solver(3, -2.6, -17, 0)
%[hind_R, mid_R, wrist_R] = inv_kinematic_solver(3, 2.6, -17, 1)
%end

for i = -10:0.2:10
[hind_R, mid_R, wrist_R] = inv_kinematic_solver(i, -2.6, -17.8, 0)
end 


function [hindlimb_angle_deg, midlimb_angle_deg, wrist_angle_deg] = inv_kinematic_solver(x_disp, y_disp, z_disp, leg)
if (leg == 0)
    hindlimb = -2.6; %Left Leg Hindlimb lenght
    midlimb = 10.0;
    wrist = 14.0;
    y_h = y_disp;
    x_h = x_disp;
    z_h = z_disp;

    total_lenght_to_end_factor_from_ZY_plane = sqrt(y_h^2 + z_h^2 );
    midlimb_to_ground = sqrt(total_lenght_to_end_factor_from_ZY_plane^2 - hindlimb^2);
    hindlimb_angle = 180 - (90 + asind(abs(hindlimb / total_lenght_to_end_factor_from_ZY_plane)) + asind(abs(z_h/total_lenght_to_end_factor_from_ZY_plane)))

    delta_z = sqrt(midlimb_to_ground^2 + x_h^2); %midlimb - ground difference for one leg. This is what we want
    end_factor_X_angle = -atand(x_h/(-midlimb_to_ground));

    midlimb_angle = end_factor_X_angle - solve_for_sss_trig(wrist, midlimb, delta_z);
    wrist_ang = 180 + midlimb_angle - solve_for_sss_trig(delta_z, midlimb, wrist);

    if ((y_h > hindlimb) && (y_h <= 0))
        hindlimb_angle_deg = hindlimb_angle;

    elseif ((y_h > hindlimb) && (y_h > 0))
        hindlimb_angle_deg =  -8.79 + (-8.79 - hindlimb_angle);

    else
        hindlimb_angle_deg = hindlimb_angle;
    end

    %hindlimb_angle_deg = hindlimb_angle;
    midlimb_angle_deg = midlimb_angle;
    wrist_angle_deg = wrist_ang;
    

        %%Plotting The Necessary Vectors (Projecting 3D space to 2D)
        midlimb_vec = [0 -midlimb]';
        wrist_vec = [0 -wrist]';
    
        rot_midlimb = rotate_vec(midlimb_angle,midlimb_vec);
        rot_wrist = rotate_vec(wrist_ang, wrist_vec);
    
        totalpoint = [(rot_midlimb(1)+rot_wrist(1)), (rot_midlimb(2)+rot_wrist(2))];
        buffer_lenght = -(hindlimb*sind(hindlimb_angle))%/sind(90-hindlimb_angle);
    
        plot(0,0,"ro")
        pause(0.02)
        plot(rot_midlimb(1),rot_midlimb(2),"bo")
        plot(totalpoint(1),totalpoint(2)+buffer_lenght,"bo")
        quiver(0,buffer_lenght,rot_midlimb(1),rot_midlimb(2),"off")
        quiver(rot_midlimb(1),rot_midlimb(2)+buffer_lenght,rot_wrist(1),rot_wrist(2), "off")

elseif (leg == 1)
    hindlimb = 2.6;
    midlimb = 10.0;
    wrist = 14.0;
    y_h = y_disp;
    x_h = x_disp;
    z_h = z_disp;

    total_lenght_to_end_factor_from_ZY_plane = sqrt(y_h^2 + z_h^2 );
    midlimb_to_ground = sqrt(total_lenght_to_end_factor_from_ZY_plane^2 - hindlimb^2);
    hindlimb_angle = 180 - (90 + asind(abs(hindlimb / total_lenght_to_end_factor_from_ZY_plane)) + asind(abs(z_h/total_lenght_to_end_factor_from_ZY_plane)));

    delta_z = sqrt(midlimb_to_ground^2 + x_h^2); %midlimb - ground difference for one leg. This is what we want
    end_factor_X_angle = -atand(x_h/(-midlimb_to_ground));

    midlimb_angle = end_factor_X_angle - solve_for_sss_trig(wrist, midlimb, delta_z);
    wrist_ang = 180 + midlimb_angle - solve_for_sss_trig(delta_z, midlimb, wrist);

    if ((y_h < hindlimb) && (y_h >= 0))
        hindlimb_angle_deg = hindlimb_angle;

    elseif ((y_h < hindlimb) && (y_h < 0))
        hindlimb_angle_deg =  -8.79 + (-8.79 - hindlimb_angle);

    else
        hindlimb_angle_deg = hindlimb_angle;
    end

    %hindlimb_angle_deg = hindlimb_angle;
    midlimb_angle_deg = midlimb_angle;
    wrist_angle_deg = wrist_ang;

        %%Plotting The Necessary Vectors (Projecting 3D space to 2D)
        midlimb_vec = [0 -midlimb]';
        wrist_vec = [0 -wrist]';
    
        rot_midlimb = rotate_vec(midlimb_angle,midlimb_vec);
        rot_wrist = rotate_vec(wrist_ang, wrist_vec);
    
        totalpoint = [(rot_midlimb(1)+rot_wrist(1)), (rot_midlimb(2)+rot_wrist(2))];
        buffer_lenght = (hindlimb*sind(hindlimb_angle))%/sind(90-hindlimb_angle);
    
        plot(0,0,"ro")
        pause(0.02)
        plot(rot_midlimb(1),rot_midlimb(2),"bo")
        plot(totalpoint(1),totalpoint(2)+buffer_lenght,"bo")
        quiver(0,buffer_lenght,rot_midlimb(1),rot_midlimb(2),"off")
        quiver(rot_midlimb(1),rot_midlimb(2)+buffer_lenght,rot_wrist(1),rot_wrist(2), "off")
inv_kin
else
    sprintf("none")
end
end

function rotated_vector = rotate_vec(degree, vector)
    R = [cosd(degree) -sind(degree); sind(degree) cosd(degree)];
    rotated_vector = R*vector;
end

function angle_deg = solve_for_sss_trig(opposite, edge1, edge2) %#codegen
    angle_rad = acos(((edge1^2) + (edge2^2) - (opposite^2))/(2*edge1*edge2));
    angle_deg = rad2deg(angle_rad);
end
