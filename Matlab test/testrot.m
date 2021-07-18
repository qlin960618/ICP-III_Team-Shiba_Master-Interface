clear all;
close all;
include_namespace_dq

r_ref=(cos(-pi/4)+sin(-pi/4)*k_)*(cos(-pi/4)+sin(-pi/4)*j_);
f_ref = r_ref+E_*DQ(0)*r_ref/2;

dir=-1*i_+3*j_+6*k_;

%normalization
dir=dir/norm(dir);

%cop=dot(r_ref, dir)
vdir=vec3(dir);




ri=atan(vdir(2)/vdir(1));
%ri=0;


tmp_pov = (cos(-ri/2)+sin(-ri/2)*k_);
%tmp_pov = (cos(-ri/2)+sin(-ri/2)*i_);
ne_line=tmp_pov*dir*conj(tmp_pov);
vne_line=vec3(ne_line)
rk=atan(-vne_line(3)/vne_line(1));
%rk=0;


t_offset=0.0*k_;
t_rot=r_ref*(cos(ri/2)+sin(ri/2)*i_)*(cos(rk/2)+sin(rk/2)*k_);
f_target = t_rot+E_*t_offset*t_rot/2
figure(1)
hold on
plot(f_ref);
plot(dir, 'line', 5);
%plot(ne_line, 'line', 5);

plot(f_target);

hold off
xlabel("x")
ylabel("y")
zlabel("z")
