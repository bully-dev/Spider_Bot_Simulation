%% Skipped Main_script6. This script gets Main_script5 ready for the 4th Optimization problem
function [Joint_angles_1,Joint_angles_2,Joint_angles_3,Joint_angles_4,Phi_angle_1,Phi_angle_2,Phi_angle_3,Phi_angle_4,globe_rad_end,globe_tht_end,globe_phi_end]=walk_controller(feet_pos,thetas_init,thetas_final,Body_disp,Leg_set)
switcher=0;
%[Leg1rad,Leg2rad,Leg3rad,Leg4rad,Leg1phi,Leg2phi,Leg3phi,Leg4phi,Time]
%[Body dx,Body dy,Body dz]
%% Spider parameters
Spider=struct;
Spider.globe_hgt=0.3;
Spider.l1=0.4;Spider.l2=0.4;Spider.l3=0.4;
Spider.T_end=0.5;
Spider.delta_T=0.001;
Spider.time_steps=Spider.T_end/Spider.delta_T;
%% Initial Variables of each leg
Leg_1=struct;
Leg_2=struct;
Leg_3=struct;
Leg_4=struct;

%% Remember to give limits here
globe_rad_1=feet_pos(1);
globe_rad_2=feet_pos(2);
globe_rad_3=feet_pos(3);
globe_rad_4=feet_pos(4);

globe_tht_1=thetas_init(1);
globe_tht_2=thetas_init(2);
globe_tht_3=thetas_init(3);
globe_tht_4=thetas_init(4);

globe_phi_1=feet_pos(5);
globe_phi_2=feet_pos(6);
globe_phi_3=feet_pos(7);
globe_phi_4=feet_pos(8);

%% Creating Leg structures
Leg_1.T_tht_start=0.05;Leg_1.T_tht_end=0.5;
Leg_2.T_tht_start=0.05;Leg_2.T_tht_end=0.5;
Leg_3.T_tht_start=0.05;Leg_3.T_tht_end=0.5;
Leg_4.T_tht_start=0.05;Leg_4.T_tht_end=0.5;

Leg_3.globe_fps=[0;0;0];
Leg_1.globe_fps=[-globe_rad_3*cos(globe_phi_3)+globe_rad_1*cos(globe_phi_1);-globe_rad_3*sin(globe_phi_3)+globe_rad_1*sin(globe_phi_1);0];
Leg_2.globe_fps=[-globe_rad_3*cos(globe_phi_3)+globe_rad_2*cos(globe_phi_2);-globe_rad_3*sin(globe_phi_3)+globe_rad_2*sin(globe_phi_2);0];
Leg_4.globe_fps=[-globe_rad_3*cos(globe_phi_3)+globe_rad_4*cos(globe_phi_4);-globe_rad_3*sin(globe_phi_3)+globe_rad_4*sin(globe_phi_4);0];

Leg_1.init_variables=[globe_rad_1;Spider.globe_hgt;globe_tht_1;globe_phi_1;Leg_1.globe_fps];
Leg_2.init_variables=[globe_rad_2;Spider.globe_hgt;globe_tht_2;globe_phi_2;Leg_2.globe_fps];
Leg_3.init_variables=[globe_rad_3;Spider.globe_hgt;globe_tht_3;globe_phi_3;Leg_3.globe_fps];
Leg_4.init_variables=[globe_rad_4;Spider.globe_hgt;globe_tht_4;globe_phi_4;Leg_4.globe_fps];

Leg_1.globe_tht_end=thetas_final(1);
Leg_2.globe_tht_end=thetas_final(2);
Leg_3.globe_tht_end=thetas_final(3);
Leg_4.globe_tht_end=thetas_final(4);



%% Body movement
Body_x_disp=Body_disp(1);
Body_y_disp=Body_disp(2);
Body_z_disp=0.0;
Spider.Heading_angle=atan2(Body_y_disp,Body_x_disp);
Body_displacements=[Body_x_disp;Body_y_disp;Body_z_disp];

Body_position=Body_position_finder(Body_displacements,Leg_3,Spider);

%% THE Kinematics:

Leg_1=Leg_kinematics(Leg_1,Body_position,Spider);
Leg_2=Leg_kinematics(Leg_2,Body_position,Spider);
Leg_3=Leg_kinematics(Leg_3,Body_position,Spider);
Leg_4=Leg_kinematics(Leg_4,Body_position,Spider);

Joint_angles_1=[Leg_1.globe_tht;Leg_1.tht2;Leg_1.tht3];
Joint_angles_2=[Leg_2.globe_tht;Leg_2.tht2;Leg_2.tht3];
Joint_angles_3=[Leg_3.globe_tht;Leg_3.tht2;Leg_3.tht3];
Joint_angles_4=[Leg_4.globe_tht;Leg_4.tht2;Leg_4.tht3];
if Leg_set==1
    Phi_angle_1=Leg_1.globe_phi-0;
    Phi_angle_2=Leg_2.globe_phi-pi/2;
    Phi_angle_3=Leg_3.globe_phi-pi;
    Phi_angle_4=Leg_4.globe_phi+pi/2;
    globe_phi_end=[Phi_angle_1(end);Phi_angle_2(end);Phi_angle_3(end);Phi_angle_4(end)];
else
    Phi_angle_1=Leg_1.globe_phi-pi/4;
    Phi_angle_2=Leg_2.globe_phi-3*pi/4;
    Phi_angle_3=Leg_3.globe_phi+3*pi/4;
    Phi_angle_4=Leg_4.globe_phi+pi/4;
    globe_phi_end=[Phi_angle_1(end);Phi_angle_2(end);Phi_angle_3(end);Phi_angle_4(end)];
end
if any(any(imag(Joint_angles_1)~=0)) || any(any(imag(Joint_angles_2)~=0)) || any(any(imag(Joint_angles_3)~=0)) || any(any(imag(Joint_angles_4)~=0))
    fprintf("danger wc");
end
globe_rad_end=[Leg_1.globe_rad(end);Leg_2.globe_rad(end);Leg_3.globe_rad(end);Leg_4.globe_rad(end)];
globe_tht_end=[Leg_1.globe_tht(end);Leg_2.globe_tht(end);Leg_3.globe_tht(end);Leg_4.globe_tht(end)];
%% Plotting
if switcher==1
figure(1)
for i=1:3:Spider.time_steps

H1=plot3([Leg_1.x0(i),Leg_1.x1(i)],[Leg_1.y0(i),Leg_1.y1(i)],[Leg_1.z0(i),Leg_1.z1(i)],'y','LineWidth',2);
hold on
U1=plot3([Leg_1.x1(i),Leg_1.x2(i)],[Leg_1.y1(i),Leg_1.y2(i)],[Leg_1.z1(i),Leg_1.z2(i)],'k','LineWidth',2);
Z1=plot3([Leg_1.x2(i),Leg_1.x3(i)],[Leg_1.y2(i),Leg_1.y3(i)],[Leg_1.z2(i),Leg_1.z3(i)],'k','LineWidth',2);

H2=plot3([Leg_2.x0(i),Leg_2.x1(i)],[Leg_2.y0(i),Leg_2.y1(i)],[Leg_2.z0(i),Leg_2.z1(i)],'k','LineWidth',2);
hold on
U2=plot3([Leg_2.x1(i),Leg_2.x2(i)],[Leg_2.y1(i),Leg_2.y2(i)],[Leg_2.z1(i),Leg_2.z2(i)],'k','LineWidth',2);
Z2=plot3([Leg_2.x2(i),Leg_2.x3(i)],[Leg_2.y2(i),Leg_2.y3(i)],[Leg_2.z2(i),Leg_2.z3(i)],'k','LineWidth',2);

H3=plot3([Leg_3.x0(i),Leg_3.x1(i)],[Leg_3.y0(i),Leg_3.y1(i)],[Leg_3.z0(i),Leg_3.z1(i)],'k','LineWidth',2);
hold on
U3=plot3([Leg_3.x1(i),Leg_3.x2(i)],[Leg_3.y1(i),Leg_3.y2(i)],[Leg_3.z1(i),Leg_3.z2(i)],'k','LineWidth',2);
Z3=plot3([Leg_3.x2(i),Leg_3.x3(i)],[Leg_3.y2(i),Leg_3.y3(i)],[Leg_3.z2(i),Leg_3.z3(i)],'k','LineWidth',2);

H4=plot3([Leg_4.x0(i),Leg_4.x1(i)],[Leg_4.y0(i),Leg_4.y1(i)],[Leg_4.z0(i),Leg_4.z1(i)],'k','LineWidth',2);
hold on
U4=plot3([Leg_4.x1(i),Leg_4.x2(i)],[Leg_4.y1(i),Leg_4.y2(i)],[Leg_4.z1(i),Leg_4.z2(i)],'k','LineWidth',2);
Z4=plot3([Leg_4.x2(i),Leg_4.x3(i)],[Leg_4.y2(i),Leg_4.y3(i)],[Leg_4.z2(i),Leg_4.z3(i)],'k','LineWidth',2);

pause
delete(H1);delete(U1);delete(Z1);
delete(H2);delete(U2);delete(Z2);
delete(H3);delete(U3);delete(Z3);
delete(H4);delete(U4);delete(Z4);

end
end


%% Functions

function Body_position=Body_position_finder(Body_displacements,Leg,Spider)
Body_position=zeros(3,round(Spider.time_steps,0));
dx=Body_displacements(1);
dy=Body_displacements(2);
% dz=Body_displacements(3);
t=Spider.delta_T:Spider.delta_T:round(Spider.T_end,3);
oblique_distance=sqrt(dx^2+dy^2)*(3*(t/Spider.T_end).^2-2*(t/Spider.T_end).^3);
x_init=-Leg.init_variables(1)*cos(Leg.init_variables(4));
y_init=-Leg.init_variables(1)*sin(Leg.init_variables(4));
z_init=Spider.globe_hgt;

Body_position(1,1:end)=x_init+oblique_distance.*cos(Spider.Heading_angle);
Body_position(2,1:end)=y_init+oblique_distance.*sin(Spider.Heading_angle);
Body_position(3,1:end)=z_init;

end


%%
function Leg=Leg_kinematics(Leg,Body_position,Spider)

globe_rad=sqrt((Body_position(1,:)-Leg.globe_fps(1)).^2+(Body_position(2,:)-Leg.globe_fps(2)).^2);
globe_hgt=ones(1,Spider.time_steps)*Leg.init_variables(2);

T_tht=[Leg.T_tht_start,Leg.T_tht_end];

globe_phi=atan2(Leg.globe_fps(2)-Body_position(2,:),Leg.globe_fps(1)-Body_position(1,:));

globe_tht=zeros(1,Spider.time_steps);

globe_tht(1:round(T_tht(1)/Spider.delta_T,0))=Leg.init_variables(3);
globe_tht(round(T_tht(2)/Spider.delta_T,0)+1:end)=Leg.globe_tht_end;
t=(round(T_tht(1),3)+Spider.delta_T):Spider.delta_T:round(T_tht(2),3);%t=t';
globe_tht(round((T_tht(1)/Spider.delta_T+1),0):round(T_tht(2)/Spider.delta_T,0))=Leg.init_variables(3)+(Leg.globe_tht_end-Leg.init_variables(3))*(3*((t-T_tht(1))/(T_tht(2)-T_tht(1))).^2-2*((t-T_tht(1))/(T_tht(2)-T_tht(1))).^3);

x0=zeros(1,Spider.time_steps);
z0=zeros(1,Spider.time_steps);
x3=x0+globe_rad;
z3=z0-globe_hgt;

x1=x0+Spider.l1*cos(globe_tht);
z1=z0+Spider.l1*sin(globe_tht);

kappa=acos(((x3-x1).^2+(z3-z1).^2+Spider.l2*Spider.l2-Spider.l3*Spider.l3)/2/Spider.l2./sqrt((x3-x1).^2+(z3-z1).^2));
lambda=acos((-(x3-x1).^2-(z3-z1).^2+Spider.l2*Spider.l2+Spider.l3*Spider.l3)/2/Spider.l2/Spider.l3);
tht2=atan2((z3-z1),(x3-x1))+kappa;
tht3=tht2-pi+lambda;

x2=x1+Spider.l2*cos(tht2);
z2=z1+Spider.l2*sin(tht2);


Joint_8D_array=[x0;z0;x1;z1;x2;z2;x3;z3];
plane_points=transform_coordinates_in_3D(Joint_8D_array,globe_phi);
Leg.x0=plane_points(1,:)+Body_position(1,:);
Leg.y0=plane_points(2,:)+Body_position(2,:);
Leg.z0=plane_points(3,:)+Body_position(3,:);
Leg.x1=plane_points(4,:)+Body_position(1,:);
Leg.y1=plane_points(5,:)+Body_position(2,:);
Leg.z1=plane_points(6,:)+Body_position(3,:);
Leg.x2=plane_points(7,:)+Body_position(1,:);
Leg.y2=plane_points(8,:)+Body_position(2,:);
Leg.z2=plane_points(9,:)+Body_position(3,:);
Leg.x3=plane_points(10,:)+Body_position(1,:);
Leg.y3=plane_points(11,:)+Body_position(2,:);
Leg.z3=plane_points(12,:)+Body_position(3,:);
Leg.globe_tht=globe_tht;
Leg.tht2=tht2;
Leg.tht3=tht3;
Leg.globe_phi=globe_phi;
Leg.globe_rad=globe_rad;
end



end