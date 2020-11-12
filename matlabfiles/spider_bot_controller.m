clear;

Times=[0.1 0.5 0.1 0.5 0.1 0.5 0.1 0.5 0.5];
parameters_init_pr1=[0 1.2 0 0];
parameters_final_pr2=[atan2(-0.1,0.8) sqrt(0.1^2+0.8^2) pi/4 0];
% parameters_pr1=[pi/6*0 atan2(-0.1,0.8) 1.15+0.05 sqrt(0.1^2+0.8^2) 0*pi/3.5];
[Joint_angles_desired_common]=place_controller(Times,parameters_init_pr1,parameters_final_pr2);

parameters_pr2=[0 0.5 0 0.5 0 0.5 0.8 0.1 0.3 parameters_final_pr2(3) pi/6];
Joint_angles_desired_common=[Joint_angles_desired_common(1:3,:) stand_controller(parameters_pr2)];

globe_fps=[parameters_pr2(7) parameters_pr2(7) parameters_pr2(7) parameters_pr2(7) 0*pi/4 pi/2 2*pi/2 3*pi/2];
thetas_init=[parameters_pr2(11) parameters_pr2(11) parameters_pr2(11) parameters_pr2(11)];
thetas_final=[pi/3 pi/6 10*pi/180 pi/6];
Body_disp=[0.25;0.0];

[Joint_angles_desired_1_wc,Joint_angles_desired_3_wc,Joint_angles_desired_5_wc,Joint_angles_desired_7_wc,Phi_angle_desired_1_wc,Phi_angle_desired_3_wc,Phi_angle_desired_5_wc,Phi_angle_desired_7_wc,globe_rad_end,globe_tht_end,globe_phi_end]=walk_controller(globe_fps,thetas_init,thetas_final,Body_disp,1);
Phi_angle_desired_1=[zeros(1,1000) Phi_angle_desired_1_wc]; 
Phi_angle_desired_3=[zeros(1,1000) Phi_angle_desired_3_wc]; 
Phi_angle_desired_5=[zeros(1,1000) Phi_angle_desired_5_wc]; 
Phi_angle_desired_7=[zeros(1,1000) Phi_angle_desired_7_wc]; 

Joint_angles_desired_1=[Joint_angles_desired_common Joint_angles_desired_1_wc];
Joint_angles_desired_3=[Joint_angles_desired_common Joint_angles_desired_3_wc];
Joint_angles_desired_5=[Joint_angles_desired_common Joint_angles_desired_5_wc];
Joint_angles_desired_7=[Joint_angles_desired_common Joint_angles_desired_7_wc];

% Times=[0.0 0.25 0.0 0.25 0.0 0.25 0.0 0.25 0.25];
% parameters_init_pr1=[0 1.2 pi/6 ];
% parameters_final_pr2=[atan2(-0.1,0.8) sqrt(0.1^2+0.8^2) pi/3 0];
% variables=[0.05 0.25 0.05 0.25 0.05 0.25 pi/3]; 
% parameters=[atan2(-0.3,0.8) 0 0.8544 0.8 pi/6];
aa1=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,0.8) 0.8544 pi/6 0],[0 sqrt(1^2+0.3^2) pi/5 -pi/4+pi/18]);
aa2=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,0.8) 0.8544 pi/6 0],[0 sqrt(0.5^2+0.3^2) 8*pi/18 -3*pi/4+11*pi/18]);
aa3=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,0.8) 0.8544 pi/6 0],[0 sqrt(0.3^2+0.3^2) 9*pi/18 3*pi/4-17*pi/18]);
aa4=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,0.8) 0.8544 pi/6 0],[0 sqrt(0.5^2+0.3^2) 8*pi/18 pi/4-pi/3]);
ab1=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(1^2+0.3^2) pi/5 -pi/4+pi/18],[atan2(-0.3,1) sqrt(1^2+0.3^2) pi/5-0.35 -pi/4+pi/18]);
ab2=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.5^2+0.3^2) 8*pi/18 -3*pi/4+11*pi/18],[atan2(-0.3,0.5) sqrt(0.5^2+0.3^2) 8*pi/18-0.4 -3*pi/4+11*pi/18]);
ab3=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.3^2+0.3^2) 9*pi/18 3*pi/4-17*pi/18],[atan2(-0.3,0.3) sqrt(0.3^2+0.3^2) 9*pi/18-0.4 3*pi/4-17*pi/18]);
ab4=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.5^2+0.3^2) 8*pi/18 pi/4-pi/3],[atan2(-0.3,0.5) sqrt(0.5^2+0.3^2) 8*pi/18-0.4 pi/4-pi/3]);

Joint_angles_desired_2=[Joint_angles_desired_common aa1(1:3,:) ab1(1:3,:)];
Joint_angles_desired_4=[Joint_angles_desired_common aa2(1:3,:) ab2(1:3,:)];
Joint_angles_desired_6=[Joint_angles_desired_common aa3(1:3,:) ab3(1:3,:)];
Joint_angles_desired_8=[Joint_angles_desired_common aa4(1:3,:) ab4(1:3,:)];
Phi_angle_desired_2=[zeros(1,1000) aa1(4,:) ab1(4,:)];
Phi_angle_desired_4=[zeros(1,1000) aa2(4,:) ab2(4,:)];
Phi_angle_desired_6=[zeros(1,1000) aa3(4,:) ab3(4,:)];
Phi_angle_desired_8=[zeros(1,1000) aa4(4,:) ab4(4,:)];

walking_cycles=3;
%% Now the walking starts
globe_rad_end_s1=globe_rad_end;globe_tht_end_s1=globe_tht_end;globe_phi_end_s1=globe_phi_end;
for j=1:walking_cycles
    
%% Leg set 1
aa1=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s1(1)) sqrt(globe_rad_end_s1(1)^2+0.3^2) globe_tht_end_s1(1) globe_phi_end_s1(1)],[0 sqrt(1^2+0.3^2) pi/5 -0-pi/18]);
aa2=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s1(2)) sqrt(globe_rad_end_s1(2)^2+0.3^2) globe_tht_end_s1(2) globe_phi_end_s1(2)],[0 sqrt(0.5^2+0.3^2) 8*pi/18 -pi/2+pi/3]);
aa3=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s1(3)) sqrt(globe_rad_end_s1(3)^2+0.3^2) globe_tht_end_s1(3) globe_phi_end_s1(3)],[0 sqrt(0.3^2+0.3^2) 9*pi/18 -pi+17*pi/18]);
aa4=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s1(4)) sqrt(globe_rad_end_s1(4)^2+0.3^2) globe_tht_end_s1(4) globe_phi_end_s1(4)],[0 sqrt(0.5^2+0.3^2) 8*pi/18 pi/2-11*pi/18]);
ab1=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(1^2+0.3^2) pi/5 -0-pi/18],[atan2(-0.3,1) sqrt(1^2+0.3^2) pi/5-0.35 -0-pi/18]);
ab2=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.5^2+0.3^2) 8*pi/18 -pi/2+pi/3],[atan2(-0.3,0.5) sqrt(0.5^2+0.3^2) 8*pi/18-0.4 -pi/2+pi/3]);
ab3=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.3^2+0.3^2) 9*pi/18 -pi+17*pi/18],[atan2(-0.3,0.3) sqrt(0.3^2+0.3^2) 9*pi/18-0.4 -pi+17*pi/18]);
ab4=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.5^2+0.3^2) 8*pi/18 pi/2-11*pi/18],[atan2(-0.3,0.5) sqrt(0.5^2+0.3^2) 8*pi/18-0.4 pi/2-11*pi/18]);

Joint_angles_desired_1=[Joint_angles_desired_1 aa1(1:3,:) ab1(1:3,:)];
Joint_angles_desired_3=[Joint_angles_desired_3 aa2(1:3,:) ab2(1:3,:)];
Joint_angles_desired_5=[Joint_angles_desired_5 aa3(1:3,:) ab3(1:3,:)];
Joint_angles_desired_7=[Joint_angles_desired_7 aa4(1:3,:) ab4(1:3,:)];
Phi_angle_desired_1=[Phi_angle_desired_1 aa1(4,:) ab1(4,:)];
Phi_angle_desired_3=[Phi_angle_desired_3 aa2(4,:) ab2(4,:)];
Phi_angle_desired_5=[Phi_angle_desired_5 aa3(4,:) ab3(4,:)];
Phi_angle_desired_7=[Phi_angle_desired_7 aa4(4,:) ab4(4,:)];

globe_fps=[1 0.5 0.3 0.5 -pi/18 pi/3 17*pi/18 -11*pi/18];
thetas_init=[pi/5-0.35 8*pi/18-0.4 9*pi/18-0.4 8*pi/18-0.4];
thetas_final=[pi/3 8*pi/18-0.2 9*pi/18-0.8 8*pi/18-0.6];
Body_disp=[0.45;0.0];

[Joint_angles_desired_1_wc,Joint_angles_desired_3_wc,Joint_angles_desired_5_wc,Joint_angles_desired_7_wc,Phi_angle_desired_1_wc,Phi_angle_desired_3_wc,Phi_angle_desired_5_wc,Phi_angle_desired_7_wc,globe_rad_end_s1,globe_tht_end_s1,globe_phi_end_s1]=walk_controller(globe_fps,thetas_init,thetas_final,Body_disp,1);
Phi_angle_desired_1=[Phi_angle_desired_1 Phi_angle_desired_1_wc]; 
Phi_angle_desired_3=[Phi_angle_desired_3 Phi_angle_desired_3_wc]; 
Phi_angle_desired_5=[Phi_angle_desired_5 Phi_angle_desired_5_wc]; 
Phi_angle_desired_7=[Phi_angle_desired_7 Phi_angle_desired_7_wc]; 

Joint_angles_desired_1=[Joint_angles_desired_1 Joint_angles_desired_1_wc];
Joint_angles_desired_3=[Joint_angles_desired_3 Joint_angles_desired_3_wc];
Joint_angles_desired_5=[Joint_angles_desired_5 Joint_angles_desired_5_wc];
Joint_angles_desired_7=[Joint_angles_desired_7 Joint_angles_desired_7_wc];


globe_fps=[1 0.5 0.3 0.5 pi/18 11*pi/18 -17*pi/18 -6*pi/18];
thetas_init=[pi/5-0.35 8*pi/18-0.4 9*pi/18-0.4 8*pi/18-0.4];
thetas_final=[pi/3 8*pi/18-0.6 9*pi/18-0.8 8*pi/18-0.2];
Body_disp=[0.45;0.0];

[Joint_angles_desired_2_wc,Joint_angles_desired_4_wc,Joint_angles_desired_6_wc,Joint_angles_desired_8_wc,Phi_angle_desired_2_wc,Phi_angle_desired_4_wc,Phi_angle_desired_6_wc,Phi_angle_desired_8_wc,globe_rad_end_s2,globe_tht_end_s2,globe_phi_end_s2]=walk_controller(globe_fps,thetas_init,thetas_final,Body_disp,2);
Phi_angle_desired_2=[Phi_angle_desired_2 Phi_angle_desired_2_wc]; 
Phi_angle_desired_4=[Phi_angle_desired_4 Phi_angle_desired_4_wc]; 
Phi_angle_desired_6=[Phi_angle_desired_6 Phi_angle_desired_6_wc]; 
Phi_angle_desired_8=[Phi_angle_desired_8 Phi_angle_desired_8_wc]; 

Joint_angles_desired_2=[Joint_angles_desired_2 Joint_angles_desired_2_wc];
Joint_angles_desired_4=[Joint_angles_desired_4 Joint_angles_desired_4_wc];
Joint_angles_desired_6=[Joint_angles_desired_6 Joint_angles_desired_6_wc];
Joint_angles_desired_8=[Joint_angles_desired_8 Joint_angles_desired_8_wc];

aa1=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s2(1)) sqrt(globe_rad_end_s2(1)^2+0.3^2) globe_tht_end_s2(1) globe_phi_end_s2(1)],[0 sqrt(1^2+0.3^2) pi/5 -pi/4+pi/18]);
aa2=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s2(2)) sqrt(globe_rad_end_s2(2)^2+0.3^2) globe_tht_end_s2(2) globe_phi_end_s2(2)],[0 sqrt(0.5^2+0.3^2) 8*pi/18 -3*pi/4+11*pi/18]);
aa3=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s2(3)) sqrt(globe_rad_end_s2(3)^2+0.3^2) globe_tht_end_s2(3) globe_phi_end_s2(3)],[0 sqrt(0.3^2+0.3^2) 9*pi/18 3*pi/4-17*pi/18]);
aa4=place_controller([0.0 0.3 0.0 0.3 0.0 0.3 0.0 0.3 0.3],[atan2(-0.3,globe_rad_end_s2(4)) sqrt(globe_rad_end_s2(4)^2+0.3^2) globe_tht_end_s2(4) globe_phi_end_s2(4)],[0 sqrt(0.5^2+0.3^2) 8*pi/18 pi/4-pi/3]);
ab1=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(1^2+0.3^2) pi/5 -pi/4+pi/18],[atan2(-0.3,1) sqrt(1^2+0.3^2) pi/5-0.35 -pi/4+pi/18]);
ab2=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.5^2+0.3^2) 8*pi/18 -3*pi/4+11*pi/18],[atan2(-0.3,0.5) sqrt(0.5^2+0.3^2) 8*pi/18-0.4 -3*pi/4+11*pi/18]);
ab3=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.3^2+0.3^2) 9*pi/18 3*pi/4-17*pi/18],[atan2(-0.3,0.3) sqrt(0.3^2+0.3^2) 9*pi/18-0.4 3*pi/4-17*pi/18]);
ab4=place_controller([0.0 0.2 0.0 0.2 0.0 0.2 0.0 0.2 0.2],[0 sqrt(0.5^2+0.3^2) 8*pi/18 pi/4-pi/3],[atan2(-0.3,0.5) sqrt(0.5^2+0.3^2) 8*pi/18-0.4 pi/4-pi/3]);


Joint_angles_desired_2=[Joint_angles_desired_2 aa1(1:3,:) ab1(1:3,:)];
Joint_angles_desired_4=[Joint_angles_desired_4 aa2(1:3,:) ab2(1:3,:)];
Joint_angles_desired_6=[Joint_angles_desired_6 aa3(1:3,:) ab3(1:3,:)];
Joint_angles_desired_8=[Joint_angles_desired_8 aa4(1:3,:) ab4(1:3,:)];
Phi_angle_desired_2=[Phi_angle_desired_2 aa1(4,:) ab1(4,:)];
Phi_angle_desired_4=[Phi_angle_desired_4 aa2(4,:) ab2(4,:)];
Phi_angle_desired_6=[Phi_angle_desired_6 aa3(4,:) ab3(4,:)];
Phi_angle_desired_8=[Phi_angle_desired_8 aa4(4,:) ab4(4,:)];




end


[a1j1pub,a1j1msg]=rospublisher('/joint1_position_controller/command');[a1j2pub,a1j2msg]=rospublisher('/joint2_position_controller/command');[a1j3pub,a1j3msg]=rospublisher('/joint3_position_controller/command');
[a2j1pub,a2j1msg]=rospublisher('/joint4_position_controller/command');[a2j2pub,a2j2msg]=rospublisher('/joint5_position_controller/command');[a2j3pub,a2j3msg]=rospublisher('/joint6_position_controller/command');
[a3j1pub,a3j1msg]=rospublisher('/joint7_position_controller/command');[a3j2pub,a3j2msg]=rospublisher('/joint8_position_controller/command');[a3j3pub,a3j3msg]=rospublisher('/joint9_position_controller/command');
[a4j1pub,a4j1msg]=rospublisher('/joint10_position_controller/command');[a4j2pub,a4j2msg]=rospublisher('/joint11_position_controller/command');[a4j3pub,a4j3msg]=rospublisher('/joint12_position_controller/command');
[a5j1pub,a5j1msg]=rospublisher('/joint13_position_controller/command');[a5j2pub,a5j2msg]=rospublisher('/joint14_position_controller/command');[a5j3pub,a5j3msg]=rospublisher('/joint15_position_controller/command');
[a6j1pub,a6j1msg]=rospublisher('/joint16_position_controller/command');[a6j2pub,a6j2msg]=rospublisher('/joint17_position_controller/command');[a6j3pub,a6j3msg]=rospublisher('/joint18_position_controller/command');
[a7j1pub,a7j1msg]=rospublisher('/joint19_position_controller/command');[a7j2pub,a7j2msg]=rospublisher('/joint20_position_controller/command');[a7j3pub,a7j3msg]=rospublisher('/joint21_position_controller/command');
[a8j1pub,a8j1msg]=rospublisher('/joint22_position_controller/command');[a8j2pub,a8j2msg]=rospublisher('/joint23_position_controller/command');[a8j3pub,a8j3msg]=rospublisher('/joint24_position_controller/command');
[a1ppub,a1pmsg]=rospublisher('/phi1_position_controller/command');
[a2ppub,a2pmsg]=rospublisher('/phi2_position_controller/command');
[a3ppub,a3pmsg]=rospublisher('/phi3_position_controller/command');
[a4ppub,a4pmsg]=rospublisher('/phi4_position_controller/command');
[a5ppub,a5pmsg]=rospublisher('/phi5_position_controller/command');
[a6ppub,a6pmsg]=rospublisher('/phi6_position_controller/command');
[a7ppub,a7pmsg]=rospublisher('/phi7_position_controller/command');
[a8ppub,a8pmsg]=rospublisher('/phi8_position_controller/command');
% for time_step=1:100
%     
% end
% for time_step=1:999
%     a1j1msg.Data=-Joint_angles_desired_common(1,time_step);
%     a1j2msg.Data=-Joint_angles_desired_common(2,time_step)+Joint_angles_desired_common(1,time_step);
%     a1j3msg.Data=-Joint_angles_desired_common(3,time_step)+Joint_angles_desired_common(2,time_step);
%     a2j1msg.Data=a1j1msg.Data;a2j2msg.Data=a1j2msg.Data;a2j3msg.Data=a1j3msg.Data;
%     a3j1msg.Data=a1j1msg.Data;a3j2msg.Data=a1j2msg.Data;a3j3msg.Data=a1j3msg.Data;
%     a4j1msg.Data=a1j1msg.Data;a4j2msg.Data=a1j2msg.Data;a4j3msg.Data=a1j3msg.Data;
%     a5j1msg.Data=a1j1msg.Data;a5j2msg.Data=a1j2msg.Data;a5j3msg.Data=a1j3msg.Data;
%     a6j1msg.Data=a1j1msg.Data;a6j2msg.Data=a1j2msg.Data;a6j3msg.Data=a1j3msg.Data;
%     a7j1msg.Data=a1j1msg.Data;a7j2msg.Data=a1j2msg.Data;a7j3msg.Data=a1j3msg.Data;
%     a8j1msg.Data=a1j1msg.Data;a8j2msg.Data=a1j2msg.Data;a8j3msg.Data=a1j3msg.Data;
%     send(a1j1pub,a1j1msg);
%     send(a1j2pub,a1j2msg);                  
%     send(a1j3pub,a1j3msg);
%     send(a2j1pub,a2j1msg);
%     send(a2j2pub,a2j2msg);                  
%     send(a2j3pub,a2j3msg);
%     send(a3j1pub,a3j1msg);
%     send(a3j2pub,a3j2msg);                  
%     send(a3j3pub,a3j3msg);
%     send(a4j1pub,a4j1msg);
%     send(a4j2pub,a4j2msg);                  
%     send(a4j3pub,a4j3msg);
%     send(a5j1pub,a5j1msg);
%     send(a5j2pub,a5j2msg);                  
%     send(a5j3pub,a5j3msg);
%     send(a6j1pub,a6j1msg);
%     send(a6j2pub,a6j2msg);                  
%     send(a6j3pub,a6j3msg);
%     send(a7j1pub,a7j1msg);
%     send(a7j2pub,a7j2msg);                  
%     send(a7j3pub,a7j3msg);
%     send(a8j1pub,a8j1msg);
%     send(a8j2pub,a8j2msg);                  
%     send(a8j3pub,a8j3msg);
%     pause(0.05)
%     disp(time_step)
% end
for time_step=1:4500
    a1j1msg.Data=-Joint_angles_desired_1(1,time_step);
    a1j2msg.Data=-Joint_angles_desired_1(2,time_step)+Joint_angles_desired_1(1,time_step);
    a1j3msg.Data=-Joint_angles_desired_1(3,time_step)+Joint_angles_desired_1(2,time_step);
    a3j1msg.Data=-Joint_angles_desired_3(1,time_step);
    a3j2msg.Data=-Joint_angles_desired_3(2,time_step)+Joint_angles_desired_3(1,time_step);
    a3j3msg.Data=-Joint_angles_desired_3(3,time_step)+Joint_angles_desired_3(2,time_step);
    a5j1msg.Data=-Joint_angles_desired_5(1,time_step);
    a5j2msg.Data=-Joint_angles_desired_5(2,time_step)+Joint_angles_desired_5(1,time_step);
    a5j3msg.Data=-Joint_angles_desired_5(3,time_step)+Joint_angles_desired_5(2,time_step);
    a7j1msg.Data=-Joint_angles_desired_7(1,time_step);
    a7j2msg.Data=-Joint_angles_desired_7(2,time_step)+Joint_angles_desired_7(1,time_step);
    a7j3msg.Data=-Joint_angles_desired_7(3,time_step)+Joint_angles_desired_7(2,time_step);
    a2j1msg.Data=-Joint_angles_desired_2(1,time_step);
    a2j2msg.Data=-Joint_angles_desired_2(2,time_step)+Joint_angles_desired_2(1,time_step);
    a2j3msg.Data=-Joint_angles_desired_2(3,time_step)+Joint_angles_desired_2(2,time_step);
    a4j1msg.Data=-Joint_angles_desired_4(1,time_step);
    a4j2msg.Data=-Joint_angles_desired_4(2,time_step)+Joint_angles_desired_4(1,time_step);
    a4j3msg.Data=-Joint_angles_desired_4(3,time_step)+Joint_angles_desired_4(2,time_step);
    a6j1msg.Data=-Joint_angles_desired_6(1,time_step);
    a6j2msg.Data=-Joint_angles_desired_6(2,time_step)+Joint_angles_desired_6(1,time_step);
    a6j3msg.Data=-Joint_angles_desired_6(3,time_step)+Joint_angles_desired_6(2,time_step);
    a8j1msg.Data=-Joint_angles_desired_8(1,time_step);
    a8j2msg.Data=-Joint_angles_desired_8(2,time_step)+Joint_angles_desired_8(1,time_step);
    a8j3msg.Data=-Joint_angles_desired_8(3,time_step)+Joint_angles_desired_8(2,time_step);
    a1pmsg.Data=Phi_angle_desired_1(time_step);
    a3pmsg.Data=Phi_angle_desired_3(time_step);
    a5pmsg.Data=Phi_angle_desired_5(time_step);
    a7pmsg.Data=Phi_angle_desired_7(time_step);
    a2pmsg.Data=Phi_angle_desired_2(time_step);
    a4pmsg.Data=Phi_angle_desired_4(time_step);
    a6pmsg.Data=Phi_angle_desired_6(time_step);
    a8pmsg.Data=Phi_angle_desired_8(time_step);
    send(a1j1pub,a1j1msg);send(a1j2pub,a1j2msg);send(a1j3pub,a1j3msg);
    send(a2j1pub,a2j1msg);send(a2j2pub,a2j2msg);send(a2j3pub,a2j3msg);
    send(a3j1pub,a3j1msg);send(a3j2pub,a3j2msg);send(a3j3pub,a3j3msg);
    send(a4j1pub,a4j1msg);send(a4j2pub,a4j2msg);send(a4j3pub,a4j3msg);
    send(a5j1pub,a5j1msg);send(a5j2pub,a5j2msg);send(a5j3pub,a5j3msg);
    send(a6j1pub,a6j1msg);send(a6j2pub,a6j2msg);send(a6j3pub,a6j3msg);
    send(a7j1pub,a7j1msg);send(a7j2pub,a7j2msg);send(a7j3pub,a7j3msg);
    send(a8j1pub,a8j1msg);send(a8j2pub,a8j2msg);send(a8j3pub,a8j3msg);
    send(a1ppub,a1pmsg);send(a2ppub,a2pmsg);send(a3ppub,a3pmsg);
    send(a4ppub,a4pmsg);send(a5ppub,a5pmsg);send(a6ppub,a6pmsg);
    send(a7ppub,a7pmsg);send(a8ppub,a8pmsg);
    pause(0.05)
    disp(time_step)    
end
    
    