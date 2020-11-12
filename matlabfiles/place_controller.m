function [Joint_angles]=place_controller(Times,parameters_init,parameters_final)
% effort=0;
close all
switcher=0;
effort=10000;
% rot_par=[-30 -20 0 10 20 30];
% str_par=[0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1];

l1=.4;
l2=.4;
l3=.4;

T_start=0;
T_end=Times(9);
delta_T=0.001;
time_steps=round(1*(T_end-T_start)/delta_T,0);

%% One Variable Test
T_rot=[Times(1) Times(2)];
T_str=[Times(3) Times(4)];
T_tht=[Times(5) Times(6)];
T_phi=[Times(7) Times(8)];


%%
x0=0*ones(time_steps,1);y0=0*ones(time_steps,1);
m1=0.2;m2=0.2;m3=0.2;M=0;g=9.81;xcm=m1*0.5*l1/(m1+M);

globe_start=parameters_init;   % [Rotation,Stretch,theta];
globe_end  =parameters_final;

% if static_flag==1 
% globe_end=globe_start;
% end

globe_rot=zeros(time_steps,1);
globe_str=zeros(time_steps,1);
globe_tht=zeros(time_steps,1);
globe_phi=zeros(time_steps,1);


globe_rot(1:round(T_rot(1)/delta_T,0))=globe_start(1);
globe_rot(round(T_rot(2)/delta_T,0)+1:end)=globe_end(1);
globe_str(1:round(T_str(1)/delta_T,0))=globe_start(2);
globe_str(round(T_str(2)/delta_T,0)+1:end)=globe_end(2);
globe_tht(1:round(T_tht(1)/delta_T,0))=globe_start(3);
globe_tht(round(T_tht(2)/delta_T,0)+1:end)=globe_end(3);
globe_phi(1:round(T_phi(1)/delta_T,0))=globe_start(4);
globe_phi(round(T_phi(2)/delta_T,0)+1:end)=globe_end(4);

tr(:,1)=(round(T_rot(1),3)+delta_T):delta_T:round(T_rot(2),3);
globe_rot(round((T_rot(1)/delta_T+1),0):round(T_rot(2)/delta_T,0))=globe_start(1)+(globe_end(1)-globe_start(1))*(3*((tr-T_rot(1))/(T_rot(2)-T_rot(1))).^2-2*((tr-T_rot(1))/(T_rot(2)-T_rot(1))).^3);
ts(:,1)=(round(T_str(1),3)+delta_T):delta_T:round(T_str(2),3);
globe_str(round((T_str(1)/delta_T+1),0):round(T_str(2)/delta_T,0))=globe_start(2)+(globe_end(2)-globe_start(2))*(3*((ts-T_str(1))/(T_str(2)-T_str(1))).^2-2*((ts-T_str(1))/(T_str(2)-T_str(1))).^3);
tt(:,1)=(round(T_tht(1),3)+delta_T):delta_T:round(T_tht(2),3);
globe_tht(round((T_tht(1)/delta_T+1),0):round(T_tht(2)/delta_T,0))=globe_start(3)+(globe_end(3)-globe_start(3))*(3*((tt-T_tht(1))/(T_tht(2)-T_tht(1))).^2-2*((tt-T_tht(1))/(T_tht(2)-T_tht(1))).^3);
tp(:,1)=(round(T_phi(1),3)+delta_T):delta_T:round(T_phi(2),3);
globe_phi(round((T_phi(1)/delta_T+1),0):round(T_phi(2)/delta_T,0))=globe_start(4)+(globe_end(4)-globe_start(4))*(3*((tp-T_phi(1))/(T_phi(2)-T_phi(1))).^2-2*((tp-T_phi(1))/(T_phi(2)-T_phi(1))).^3);

x3=x0+globe_str.*cos(globe_rot);
y3=y0+globe_str.*sin(globe_rot);

xc1=x0+xcm.*cos(globe_tht);
yc1=y0+xcm.*sin(globe_tht);
 x1=x0+l1*cos(globe_tht);
 y1=y0+l1*sin(globe_tht);

kappa=acos(((x3-x1).^2+(y3-y1).^2+l2*l2-l3*l3)/2/l2./sqrt((x3-x1).^2+(y3-y1).^2));
lambda=acos((-(x3-x1).^2-(y3-y1).^2+l2*l2+l3*l3)/2/l2/l3);
tht2=atan2((y3-y1),(x3-x1))+kappa;
tht3=tht2-pi+lambda;

x2=x1+l2*cos(tht2);
y2=y1+l2*sin(tht2);
xc2=x1+0.5*l2*cos(tht2);
yc2=y1+0.5*l2*sin(tht2);

xc3=x2+0.5*l3*cos(tht3);
yc3=y2+0.5*l3*sin(tht3);




%% Dynamics


Joint_angles=[globe_tht tht2 tht3 globe_phi]';

if any(imag(x2)) || any(imag(x3))
   fprintf("danger pc \n");
   effort=effort+10000*(sum(abs(imag(x2)))+sum(abs(imag(x3))));
end

if any(globe_tht<tht2) || any(tht2-tht3>(3*pi/4))  || any(tht2<-pi/2-pi/18)
   effort = effort  +  10*abs(sum((tht2-globe_tht).*(globe_tht<tht2),'all'))  +  10*abs(sum((tht2-tht3-3*pi/4).*(tht2-tht3>3*pi/4),'all'))  +  10*abs(sum((-tht2-pi/2-pi/18).*(tht2<-pi/2-pi/18),'all'));
   fprintf("one more danger pc \n");
end



% fprintf('%.5f\n',effort);
if switcher==1
%% Initial Plotting
figure(1)

% legend('Moment in Joint 1','Moment in Joint 2','Moment in Joint 3');
% subplot(1,2,1)
h=plot([x0(1),x1(1)],[y0(1),y1(1)],'k','LineWidth',2);
hold on
o=plot([x1(1),x2(1)],[y1(1),y2(1)],'k','LineWidth',2);
p=plot([x2(1),x3(1)],[y2(1),y3(1)],'k','LineWidth',2);
axis([-1.5 1.5 -1.5 1.5])

% delete(h);delete(o);delete(p);

% subplot(1,2,2)
% plot(100,100,'.b');
% hold on
% plot(100,101,'.r');
% plot(100,102,'.g');
% % legend('Moment in Joint 1','Moment in Joint 2','Moment in Joint 3');
% % plot(1,X(6,1),'.b')
% % plot(1,X(7,1),'.r');
% % plot(1,X(8,i),'.g');
% plot(1,X(7,1),'.b','HandleVisibility','off')
% plot(1,X(8,1),'.r','HandleVisibility','off');
% plot(1,X(9,i),'.g','HandleVisibility','off');
% axis([0 1000 -25 25]);
% pause


%% Actual Plotting

for i=1:10:time_steps-1
i
% subplot(1,2,1);hold on
h=plot([x0(i),x1(i)],[y0(i),y1(i)],'k','LineWidth',2);
hold on
o=plot([x1(i),x2(i)],[y1(i),y2(i)],'k','LineWidth',2);
p=plot([x2(i),x3(i)],[y2(i),y3(i)],'k','LineWidth',2);


% axis equal

% my_flipbook(frame)=getframe(gcf);
%     frame=frame+1;
% my_movie=VideoWriter('Spider_arm');
% my_movie.FrameRate=20;
% open(my_movie);
% writeVideo(my_movie,my_flipbook);
% close(my_movie);
%

% figure;plot(1:1000,xc1);
% figure;plot(1:1000,dot_xc1);
% figure;plot(1: 999,ddot_xc1);

% subplot(1,2,1);
% h=plot([x0(i),x1(i)],[y0(i),y1(i)],'k','LineWidth',2);
% hold on
% o=plot([x1(i),x2(i)],[y1(i),y2(i)],'k','LineWidth',2);
% p=plot([x2(i),x3(i)],[y2(i),y3(i)],'k','LineWidth',2);

% axis equal
%   
% subplot(1,2,2);
% plot(i/2,X(7,i),'.m')
% plot(i/2,X(8,i),'.y');
% plot(i/2,X(9,i),'.c');
% plot(i/2,X(7,i),'.b','HandleVisibility','off')
% plot(i/2,X(8,i),'.r','HandleVisibility','off');
% plot(i/2,X(9,i),'.g','HandleVisibility','off');

hold on

drawnow 

% if i>time_steps-100
%     pause
% end
delete(h);delete(o);delete(p);
end
end

end