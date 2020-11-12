function plane3D_points=transform_coordinates_in_3D(Joint_vector,globe_phi)
plane_x0=zeros(1,length(globe_phi));plane_x1=zeros(1,length(globe_phi));plane_x2=zeros(1,length(globe_phi));plane_x3=zeros(1,length(globe_phi));plane_y0=zeros(1,length(globe_phi));plane_y1=zeros(1,length(globe_phi));plane_y2=zeros(1,length(globe_phi));plane_y3=zeros(1,length(globe_phi));plane_z0=zeros(1,length(globe_phi));plane_z1=zeros(1,length(globe_phi));plane_z2=zeros(1,length(globe_phi));plane_z3=zeros(1,length(globe_phi));
for i=1:length(globe_phi)
points_plane=[cos(globe_phi(i)) -sin(globe_phi(i)) 0;sin(globe_phi(i)) cos(globe_phi(i)) 0;0 0 1]*[Joint_vector(1,i);0;Joint_vector(2,i)];
plane_x0(1,i)=points_plane(1);
plane_y0(1,i)=points_plane(2);
plane_z0(1,i)=points_plane(3);
points_plane=[cos(globe_phi(i)) -sin(globe_phi(i)) 0;sin(globe_phi(i)) cos(globe_phi(i)) 0;0 0 1]*[Joint_vector(3,i);0;Joint_vector(4,i)];
plane_x1(1,i)=points_plane(1);
plane_y1(1,i)=points_plane(2);
plane_z1(1,i)=points_plane(3);
points_plane=[cos(globe_phi(i)) -sin(globe_phi(i)) 0;sin(globe_phi(i)) cos(globe_phi(i)) 0;0 0 1]*[Joint_vector(5,i);0;Joint_vector(6,i)];
plane_x2(1,i)=points_plane(1);
plane_y2(1,i)=points_plane(2);
plane_z2(1,i)=points_plane(3);
points_plane=[cos(globe_phi(i)) -sin(globe_phi(i)) 0;sin(globe_phi(i)) cos(globe_phi(i)) 0;0 0 1]*[Joint_vector(7,i);0;Joint_vector(8,i)];
plane_x3(1,i)=points_plane(1);
plane_y3(1,i)=points_plane(2);
plane_z3(1,i)=points_plane(3);
% plain3D_points=[plane_x0(i)-Leg_variables(1,i)*cos(Leg_variables(4));plane_y0-Leg_variables(1)*sin(Leg_variables(4));plane_z0+Leg_variables(2);plane_x1-Leg_variables(1)*cos(Leg_variables(4));plane_y1-Leg_variables(1)*sin(Leg_variables(4));plane_z1+Leg_variables(2);plane_x2-Leg_variables(1)*cos(Leg_variables(4));plane_y2-Leg_variables(1)*sin(Leg_variables(4));plane_z2+Leg_variables(2);plane_x3-Leg_variables(1)*cos(Leg_variables(4));plane_y3-Leg_variables(1)*sin(Leg_variables(4));plane_z3+Leg_variables(2)];
end
plane3D_points=[plane_x0;plane_y0;plane_z0;plane_x1;plane_y1;plane_z1;plane_x2;plane_y2;plane_z2;plane_x3;plane_y3;plane_z3];
end