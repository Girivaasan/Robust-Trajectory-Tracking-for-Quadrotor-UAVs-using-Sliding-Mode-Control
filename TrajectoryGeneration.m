% syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot 'real';
syms r1 r2 l1 l2 m1 m2 I1 I2 g L time 'real';

t1=0;
t2=5;
t3=20;
t4=35;
t5=50;
t6=65;

Tm=[[1,   1,   1,   1,   1,   1];
    [t1,  t2,  t3,  t4,  t5,  t6];
    [t1^2,t2^2,t3^2,t4^2,t5^2,t6^2];
    [t1^3,t2^3,t3^3,t4^3,t5^3,t6^3];
    [t1^4,t2^4,t3^4,t4^4,t5^4,t6^4];
    [t1^5,t2^5,t3^5,t4^5,t5^5,t6^5]];

x=[0,0,1,1,0,0];

ax=x/Tm;
% disp(ax)

y=[0,0,0,1,1,0];

ay=y/Tm;
% disp(ay)

z=[0,1,1,1,1,1];

az=z/Tm;
% disp(az)

T= [1;time;time^2;time^3;time^4;time^5];
Tdash = [0;1;2*time;3*time^2;4*time^3;5*time^4];
Tddash = [0;0;2;6*time;12*time^2;20*time^3];

px_traj     = ax * T ;
vx_traj     = ax * Tdash;
ax_traj     = ax * Tddash;

py_traj     = ay * T ;
vy_traj     = ay * Tdash;
ay_traj     = ay * Tddash;

pz_traj     = az * T ;
vz_traj     = az * Tdash;
az_traj     = az * Tddash;

x_des_list = [];
vx_des_list = [];
ax_des_list = [];
y_des_list = [];
vy_des_list=[];
ay_des_list=[];
z_des_list = [];
vz_des_list=[];
az_des_list=[];
t = 0:0.01:65;

for i=1:size(t')
   
    x_des_list(end+1) = double(subs(px_traj ,t(i)));
    vx_des_list(end+1) = double(subs(vx_traj ,t(i))); 
    ax_des_list(end+1) = double(subs(ax_traj ,t(i))); 
    y_des_list(end+1) = double(subs(py_traj ,t(i)));
    vy_des_list(end+1) = double(subs(vy_traj ,t(i))); 
    ay_des_list(end+1) = double(subs(ay_traj ,t(i))); 
    z_des_list(end+1) = double(subs(pz_traj ,t(i)));
    vz_des_list(end+1) = double(subs(vz_traj ,t(i))); 
    az_des_list(end+1) = double(subs(az_traj ,t(i))); 

end

subplot(3,3,1);
% plot(t,y(:,1));
title('X-Position');
xlabel('Time (t)');
ylabel('X');
hold on;
plot(t,x_des_list);
% legend('y','x_des_list')

subplot(3,3,2);
% plot(t,y(:,2));
title('X-Velocity');
xlabel('Time (t)');
ylabel('X-dot');
hold on;
plot(t,vx_des_list);
% legend('y','vx_des_list')

subplot(3,3,3);
% plot(t,y(:,3));
title('X-Acceleration');
xlabel('Time (t)');
ylabel('X-ddot');
hold on;
plot(t,ax_des_list);
% legend('y','ax_des_list')

subplot(3,3,4);
% plot(t,y(:,1));
title('Y-Position');
xlabel('Time (t)');
ylabel('Y');
hold on;
plot(t,y_des_list);
% legend('y','y_des_list')

subplot(3,3,5);
% plot(t,y(:,2));
title('Y-Velocity');
xlabel('Time (t)');
ylabel('Y-dot');
hold on;
plot(t,vy_des_list);
% legend('y','vx_des_list')

subplot(3,3,6);
% plot(t,y(:,3));
title('Y-Acceleration');
xlabel('Time (t)');
ylabel('Y-ddot');
hold on;
plot(t,ay_des_list);
% legend('y','ax_des_list')

subplot(3,3,7);
% plot(t,y(:,1));
title('Z-Position');
xlabel('Time (t)');
ylabel('Z');
hold on;
plot(t,z_des_list);
% legend('y','x_des_list')

subplot(3,3,8);
plot(t,y(:,2));
title('Z-Velocity');
xlabel('Time (t)');
ylabel('Z-dot');
hold on;
plot(t,vz_des_list);
% legend('y','vx_des_list')

subplot(3,3,9);
plot(t,y(:,3));
title('Z-Acceleration');
xlabel('Time (t)');
ylabel('Z-ddot');
hold on;
plot(t,az_des_list);
% legend('y','ax_des_list')

f2=figure;
plot3(x_des_list,y_des_list,z_des_list);


