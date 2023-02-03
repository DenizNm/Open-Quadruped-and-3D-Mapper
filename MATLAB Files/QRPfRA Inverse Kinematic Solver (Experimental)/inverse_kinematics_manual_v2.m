
%[hind_R, mid_R, wrist_R] = inv_kinematic_solver(7, 0, -17, 1)

function [hindlimb_angle_deg, midlimb_angle_deg, wrist_angle_deg] = inv_kinematic_solver(x_disp, y_disp, z_disp, leg) %#codegen
hindlimb_angle_deg = 0;
midlimb_angle_deg = -55.41;
wrist_angle_deg = 36.01;
%left = 0
%right = 1
if (leg == 0)
    hindlimb = -2.6;
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

    midlimb_angle = end_factor_X_angle - rad2deg(acos(((midlimb^2) + (delta_z^2) - (wrist^2))/(2*midlimb*delta_z)));
    wrist_ang = 180 + midlimb_angle - rad2deg(acos(((midlimb^2) + (wrist^2) - (delta_z^2))/(2*midlimb*wrist)));

    if ((y_h > hindlimb) && (y_h <= 0))
        hindlimb_angle_deg = hindlimb_angle;

    elseif ((y_h > hindlimb) && (y_h > 0))
        hindlimb_angle_deg =  -8.79 + (-8.79 - hindlimb_angle);

    else
        hindlimb_angle_deg = hindlimb_angle;
    end
    midlimb_angle_deg = midlimb_angle;
    wrist_angle_deg = wrist_ang;

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

    midlimb_angle = end_factor_X_angle - rad2deg(acos(((midlimb^2) + (delta_z^2) - (wrist^2))/(2*midlimb*delta_z)));
    wrist_ang = 180 + midlimb_angle - rad2deg(acos(((midlimb^2) + (wrist^2) - (delta_z^2))/(2*midlimb*wrist)));

    if ((y_h < hindlimb) && (y_h >= 0))
        hindlimb_angle_deg = hindlimb_angle;

    elseif ((y_h < hindlimb) && (y_h < 0))
        hindlimb_angle_deg =  -8.79 + (-8.79 - hindlimb_angle);

    else
        hindlimb_angle_deg = hindlimb_angle;
    end

    midlimb_angle_deg = midlimb_angle;
    wrist_angle_deg = wrist_ang;

else
    sprintf("none")
end
end
