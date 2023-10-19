function rcs = brcsellipsoid_circle(len,radius,bistatic_angle_phi,theta_i,theta_s)
% bistatic rcs for an ellipsoid with circular section.
a = radius;
b = radius;
c = len/2;
rcs = (4*pi*a^2*b^2*c^2)*((1+cos(theta_i)*cos(theta_s))*cos(bistatic_angle_phi)+sin(theta_i)*sin(theta_s))^2/ ...
    (a^2*(sin(theta_i)^2+sin(theta_s)^2+2*sin(theta_i)*sin(theta_s)*cos(bistatic_angle_phi))+...
    c^2*(cos(theta_i)+cos(theta_s))^2 ...
    )^2;
end