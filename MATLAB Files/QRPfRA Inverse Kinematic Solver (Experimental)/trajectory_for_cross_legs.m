
[FR_RL_X, FR_RL_Y, FR_RL_Z, FL_RR_X, FL_RR_Y, FL_RR_Z] = forward_pattern_generator(17, 4, 20, 2)



function [FR_RL_X, FR_RL_Y, FR_RL_Z, FL_RR_X, FL_RR_Y, FL_RR_Z] = forward_pattern_generator(height_from_ground, step_lenght, trap_traj_amount, which_phase)%#codegen

%height_from_ground = 15;
%step_lenght = 7;
%trap_traj_amount = 6;
%which_phase = 0

% NONE = 0
% START = 1
% CONTINUOUS = 2
% STOP = 3
first_wp_FR_RL = [0 step_lenght/2 step_lenght; 0 0 0; -height_from_ground (-height_from_ground+2) -height_from_ground]; %First Phase Front_Right x Rear_Left 125ms START
second_wp_FR_RL = [step_lenght 0 -step_lenght; 0 0 0; -height_from_ground -height_from_ground -height_from_ground]; %Second Phase Front_Right x Rear_Left 125ms CONTINUOUS
third_wp_FR_RL = [-step_lenght 0 step_lenght;  0 0 0; -height_from_ground (-height_from_ground+4) -height_from_ground]; %Third Phase Front_Right x Rear_Left 125ms CONTINUOUS
fourth_wp_FR_RL = [step_lenght step_lenght/2 0; 0 0 0; -height_from_ground -height_from_ground -height_from_ground]; %Fourth Phase Front_Right x Rear_Left 125ms STOP


first_wp_FL_RR = [0 -step_lenght/2 -step_lenght; 0 0 0; -height_from_ground -height_from_ground -height_from_ground]; %First Phase 125ms START
second_wp_FL_RR = [-step_lenght 0 step_lenght; 0 0 0 ;-height_from_ground (-height_from_ground+4) -height_from_ground]; %Second Phase  125ms CONTINUOUS
third_wp_FL_RR = [step_lenght 0 -step_lenght; 0 0 0 ; -height_from_ground -height_from_ground -height_from_ground]; %Third Phase 125ms CONTINUOUS
fourth_wp_FL_RR = [step_lenght step_lenght/2 0; 0 0 0; -height_from_ground (-height_from_ground+2) -height_from_ground]; %Fourth Phase 125ms STOP
%If hindlimb has a value other than 0 lenght must be considered differently
%for  left and right hindlimb as left size must have negative value for
%end factor position for Y-Axis.

if (which_phase == 0)
    NONE = [0 0 0; 0 0 0; -height_from_ground -height_from_ground -height_from_ground];
    [None] = trapveltraj(NONE, trap_traj_amount);
    FR_RL_X = None(1,:);
    FR_RL_Y = None(2,:);
    FR_RL_Z = None(3,:);
    FL_RR_X = None(1,:);
    FL_RR_Y = None(2,:);
    FL_RR_Z = None(3,:);

elseif (which_phase == 1)
    [FR_RL_q] = trapveltraj(first_wp_FR_RL, trap_traj_amount);
    FR_RL_X = FR_RL_q(1,:);
    FR_RL_Y = FR_RL_q(2,:);
    FR_RL_Z = FR_RL_q(3,:);
    
    [FL_RR_q] = trapveltraj(first_wp_FL_RR, trap_traj_amount);
    FL_RR_X = FL_RR_q(1,:);
    FL_RR_Y = FL_RR_q(2,:);
    FL_RR_Z = FL_RR_q(3,:);

elseif (which_phase == 2)
    second_FR_RL = trapveltraj(second_wp_FR_RL,trap_traj_amount)
    third_FR_RL = trapveltraj(third_wp_FR_RL,trap_traj_amount)
    [FR_RL_q] = cat(2,second_FR_RL,third_FR_RL);%trapveltraj((second_wp_FR_RL + third_wp_FR_RL), trap_traj_amount);
    FR_RL_X = FR_RL_q(1,:);
    FR_RL_Y = FR_RL_q(2,:);
    FR_RL_Z = FR_RL_q(3,:);
    
    second_FL_RR = trapveltraj(second_wp_FL_RR, trap_traj_amount);
    third_FL_RR = trapveltraj(third_wp_FL_RR, trap_traj_amount);
    [FL_RR_q] = cat(2,second_FL_RR,third_FL_RR);%trapveltraj((second_wp_FL_RR + third_wp_FL_RR), trap_traj_amount);
    FL_RR_X = FL_RR_q(1,:);
    FL_RR_Y = FL_RR_q(2,:);
    FL_RR_Z = FL_RR_q(3,:);

elseif(which_phase == 3)
    [FR_RL_q] = trapveltraj(fourth_wp_FR_RL, trap_traj_amount);
    FR_RL_X = FR_RL_q(1,:);
    FR_RL_Y = FR_RL_q(2,:);
    FR_RL_Z = FR_RL_q(3,:);
    
    [FL_RR_q] = trapveltraj(fourth_wp_FL_RR, trap_traj_amount);
    FL_RR_X = FL_RR_q(1,:);
    FL_RR_Y = FL_RR_q(2,:);
    FL_RR_Z = FL_RR_q(3,:);
else
    NONE = [0 0 0; 0 0 0; -height_from_ground -height_from_ground -height_from_ground];
    [None] = trapveltraj(NONE, trap_traj_amount);
    FR_RL_X = None(1,:);
    FR_RL_Y = None(2,:);
    FR_RL_Z = None(3,:);
    FL_RR_X = None(1,:);
    FL_RR_Y = None(2,:);
    FL_RR_Z = None(3,:);
  end
end