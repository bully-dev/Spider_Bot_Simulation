function Joint_angles=stand_controller(parameters)

switcher=0;

l1=.4; l2=.4; l3=.4;
T_start=0;
T_end=0.5;
delta_T=0.001;
time_steps=1*(T_end-T_start)/delta_T;

T_rad=[parameters(1) parameters(2)];
T_hgt=[parameters(3) parameters(4)];
T_tht=[parameters(5) parameters(6)];

x3=0*ones(time_steps,1);y3=0*ones(time_steps,1);

globe_start=[parameters(7);parameters(8);parameters(10)];   % [Radius,Height,theta,omega];
globe_end  =[parameters(7);parameters(9);parameters(11)];

globe_rad=zeros(time_steps,1);
globe_hgt=zeros(time_steps,1);
globe_tht=zeros(time_steps,1);

globe_rad(1:round(T_rad(1)/delta_T,0))=globe_start(1);
globe_rad(round(T_rad(2)/delta_T,0)+1:end)=globe_end(1);
globe_hgt(1:round(T_hgt(1)/delta_T,0))=globe_start(2);
globe_hgt(round(T_hgt(2)/delta_T,0)+1:end)=globe_end(2);
globe_tht(1:round(T_tht(1)/delta_T,0))=globe_start(3);
globe_tht(round(T_tht(2)/delta_T,0)+1:end)=globe_end(3);

t=(round(T_rad(1),3)+delta_T):delta_T:round(T_rad(2),3);t=t';
globe_rad(round((T_rad(1)/delta_T+1),0):round(T_rad(2)/delta_T,0))=globe_start(1)+(globe_end(1)-globe_start(1))*(3*((t-T_rad(1))/(T_rad(2)-T_rad(1))).^2-2*((t-T_rad(1))/(T_rad(2)-T_rad(1))).^3);
t=(round(T_hgt(1),3)+delta_T):delta_T:round(T_hgt(2),3);t=t';
globe_hgt(round((T_hgt(1)/delta_T+1),0):round(T_hgt(2)/delta_T,0))=globe_start(2)+(globe_end(2)-globe_start(2))*(3*((t-T_hgt(1))/(T_hgt(2)-T_hgt(1))).^2-2*((t-T_hgt(1))/(T_hgt(2)-T_hgt(1))).^3);
t=(round(T_tht(1),3)+delta_T):delta_T:round(T_tht(2),3);t=t';
globe_tht(round((T_tht(1)/delta_T+1),0):round(T_tht(2)/delta_T,0))=globe_start(3)+(globe_end(3)-globe_start(3))*(3*((t-T_tht(1))/(T_tht(2)-T_tht(1))).^2-2*((t-T_tht(1))/(T_tht(2)-T_tht(1))).^3);

x0=x3-globe_rad;
y0=y3+globe_hgt;

x1=x0+l1*cos(globe_tht);
y1=y0+l1*sin(globe_tht);

kappa=acos(((x3-x1).^2+(y3-y1).^2+l2*l2-l3*l3)/2/l2./sqrt((x3-x1).^2+(y3-y1).^2));
lambda=acos((-(x3-x1).^2-(y3-y1).^2+l2*l2+l3*l3)/2/l2/l3);
tht2=atan2((y3-y1),(x3-x1))+kappa;
tht3=tht2-pi+lambda;
Joint_angles=[globe_tht';tht2';tht3'];
x2=x1+l2*cos(tht2);
y2=y1+l2*sin(tht2);

if any(imag(x2)) || any(imag(x3))
   fprintf("danger");
end
if any(globe_tht<tht2)
    fprintf("another danger");
end

if switcher==1
figure(1)

% legend('Moment in Joint 1','Moment in Joint 2','Moment in Joint 3');
plot([x0(1),x1(1)],[y0(1),y1(1)],'y','LineWidth',2);
hold on
plot([x1(1),x2(1)],[y1(1),y2(1)],'y','LineWidth',2);
plot([x2(1),x3(1)],[y2(1),y3(1)],'y','LineWidth',2);
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])

%% Actual Plotting

for i=9:10:time_steps-1
i

h=plot([x0(i),x1(i)],[y0(i),y1(i)],'k','LineWidth',2);
hold on
u=plot([x1(i),x2(i)],[y1(i),y2(i)],'k','LineWidth',2);
z=plot([x2(i),x3(i)],[y2(i),y3(i)],'k','LineWidth',2);

xlabel("xaxis");
ylabel("yaxis");
zlabel("zaxis");

drawnow 

delete(h);delete(u);delete(z);
end
end
end