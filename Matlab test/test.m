clear all;
close all;
include_namespace_dq


l=25;

r1=(cos(pi/4)+sin(pi/4)*k_)*(cos(3*pi/8)+sin(3*pi/8)*i_);
t1=l*j_ + 2*l*k_;
cam1 = r1+E_*t1*r1/2

r2=cos(3*pi/8)+sin(3*pi/8)*i_;
t2=l*i_ + 2*l*j_ + 2*l*k_;
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


balldir = 30*k_ + 0*i_ + 0*j_;
%pt = 4*k_ + 3*i_ + 4*j_;
pt=DQ(0);
dir = balldir/norm(balldir);
m1 = cross(pt,dir);
l1_dq = dir + E_*m1
figure(2)
plot(l1_dq,'line', 5); 

l2 = l1_dq*cam2
%l2=l2/norm(l2)
is_line(l2)
figure(3)
plot(l2,'line', 5); 
