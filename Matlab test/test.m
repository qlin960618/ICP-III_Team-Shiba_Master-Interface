clear all;
close all;
include_namespace_dq


l=20 ;

r1=(cos(-3*pi/8)+sin(-3*pi/8)*j_)*(cos(-pi/4)+sin(-pi/4)*k_);
t1=-l*j_;
cam1 = r1+E_*t1*r1/2

r2=(cos(3*pi/8)+sin(3*pi/8)*i_);
t2=-l*i_;
cam2 = r2+E_*t2*r2/2

figure(1)
subplot(1,2,1)
plot(cam1);
title('camera 1')
xlabel("x")
zlabel("z")
subplot(1,2,2)
plot(cam2);
title('camera 2')
xlabel("x")
zlabel("z")

figure(2)
hold on
plot(cam1);
plot(cam2);
hold off
xlabel("x")
ylabel("y")
zlabel("z")


balldir = 30*k_ + 1*i_ + 1*j_;
%pt = 4*k_ + 3*i_ + 4*j_;
pt=DQ(0);
dir = balldir/norm(balldir);
m1 = cross(pt,dir);
l1_dq = dir + E_*m1
l1_dq1 = dir + E_*cross(balldir,dir)
DQ(1+k_)
figure(3)
hold on
plot(l1_dq,'line', 5); 
plot3(0,0,0, 'ro');
hold off
xlabel("x")
ylabel("y")
zlabel("z")


l2_1 = cam1*l1_dq*conj(cam1);
l2_2 = cam2*l1_dq*conj(cam2);
[pt1, pt2] = get_points_between_lines(l2_1, l2_2);
%l2=l2/norm(l2)
figure(4)
hold on
plot(l2_1,'line', 60); 
plot(cam1);
plot(l2_2,'line', 60); 
plot(cam2);
pt1=vec3(pt1);
pt2=vec3(pt2);
plot3(pt1(1),pt1(2),pt1(3), 'ro')
plot3(pt2(1),pt2(2),pt2(3), 'ro')
hold off

xlabel("x")
ylabel("y")
zlabel("z")

