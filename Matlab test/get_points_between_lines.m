function [pt1,pt2] = get_points_between_lines(plucker1,plucker2)
include_namespace_dq
%GET_POINTS_BETWEEN_LINES find the two cloest points on two plucker line
%   Detailed explanation goes here

m1=D(plucker1);
l1=P(plucker1);
m2=D(plucker2);
l2=P(plucker2);

pt1 = (cross(-1.0*m1, cross(l2, cross(l1, l2)))+dot(m2,cross(l1, l2))*l1)*(1.0/norm(vec4(cross(l1, l2))))^2;
pt2 = (cross(     m2, cross(l1, cross(l1, l2)))-dot(m1,cross(l1, l2))*l2)*(1.0/norm(vec4(cross(l1, l2))))^2;


end

