robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);

LB = 2.6; % hindlimb
L1 = 10; % midlimb
L2 = 14; % wrist

body = rigidBody('start');
joint = rigidBodyJoint('fix0','fixed');
setFixedTransform(joint, trvec2tform([0, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([LB, 0, 0]));
joint.JointAxis = [1 1 1];
body.Joint = joint;
addBody(robot, body, 'start');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint,trvec2tform([0, 0, -L1]));
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([0,0, -L2]));
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(robot, body, 'link2');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([0, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');


t = (0:0.2:10)'; % Time
count = length(t);

lenght_of_arc = L1 + L2 - 1;
deneme = linspace(-16,-18,count)';
denemex = linspace(-0,2,count)';
wayps = linspace(-lenght_of_arc,lenght_of_arc,count)';
points = [(denemex.*ones([count,1])) zeros([count,1]) deneme];%zeros([count,1])%[2.6.*ones([count,1]) denemey 17.*ones([count,1])];

% center = [0.05 0.05 -(L1+L2-0.05)];
% radius = 0.1;
% theta = t*(2*pi/t(end));
% points = center + radius*[zeros(size(theta)) cos(theta) -sin(theta)];

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25, 0.25, 0.25, 1, 1, 1];
endEffector = 'tool';


qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end


hold on
axis equal
% framesPerSecond = 5;
% r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',true);
%     drawnow
%     waitfor(r);
end