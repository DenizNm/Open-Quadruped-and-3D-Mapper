x = 0.5;
y = 0.5;
z = 0.0;
origin = [0,0,0]
initial_point = [x,y,z]

quiver3(0,0,0,x,y,z,'r')
hold on
axis([-1 1 -1 1])

quat = quaternion([0,0,pi/4],'euler','XYZ','point');           
rotatedPoint = quatrotate(compact(quat),initial_point)




qparts = compact(quat);
inverted  = quatinv(qparts);
%inverted_quat = quaternion(inverted(1),inverted(2),inverted(3),inverted(4));
inverse_rotated_point = quatrotate(inverted, rotatedPoint)





% quat_as_tform = quat2tform(inverted_quat);
% quat_as_tform = affine3d(quat_as_tform);
% [invx,invy,invz] =  transformPointsForward(quat_as_tform,x,y,z);
% inv_point = [invx,invy,invz]

quiver3(0,0,0, rotatedPoint(1),rotatedPoint(2),rotatedPoint(3), 'b')
quiver3(0,0,0, inverse_rotated_point(1),inverse_rotated_point(2),inverse_rotated_point(3), 'g')


