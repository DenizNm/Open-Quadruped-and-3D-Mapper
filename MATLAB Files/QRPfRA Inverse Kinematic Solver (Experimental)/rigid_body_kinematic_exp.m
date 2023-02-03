robot = rigidBodyTree;
axis equal

hindlimb = 2.6;
midlimb = 10.0;
wrist = 14.0;


for i = 12:0.2:21
    x_h = 0;
    z_h = -i;
    delta_z = sqrt(z_h^2 + x_h^2);
    
    hip_angle = -atand(x_h/z_h)


midlimb_angle =   - solve_for_sss_trig(wrist, midlimb, delta_z);
wrist_ang = 180 + midlimb_angle - solve_for_sss_trig(delta_z, midlimb, wrist);



body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

jnt1.JointAxis = [0 1 0];
tform = trvec2tform([0, 2.6, 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
addBody(robot,body1,'base')

body2 = rigidBody("base1");
jnt2 = rigidBodyJoint('jnt2','revolute');

jnt2.JointAxis = [0 1 0];
tform2 = trvec2tform([0, 0, -10]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');

jnt3.JointAxis = [0 1 0];
tform3 = trvec2tform([0, 0, -14]); % User defined
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body2 to body1

body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4');
tform4 = trvec2tform([0, 0, -0.0001]); % User defined
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4;
addBody(robot,body4,'body3'); % Add body2 to body1


jnt1.HomePosition = deg2rad(midlimb_angle);
jnt2.HomePosition = deg2rad(wrist_ang); % User defined
%jnt3.HomePosition = 0; % User defined

hold on
show(robot,'PreservePlot',true)

end

function angle_deg = solve_for_sss_trig(opposite, edge1, edge2) %#codegen
    angle_deg = acos(((edge1^2) + (edge2^2) - (opposite^2))/(2*edge1*edge2));
    %angle_deg = rad2deg(angle_rad);
end












