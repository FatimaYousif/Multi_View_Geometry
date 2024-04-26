% same as step 4
function points_2d=project_3d_points(P, points_3d)

%initialize
points_2d= zeros(size(points_3d,1),3);

%projection
for i =1: size(points_3d,1)
    point_2d=P*points_3d(i,:)';  
    points_2d(i,:)=point_2d(1:3)/point_2d(3); 
end
