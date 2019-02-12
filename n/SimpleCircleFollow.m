function guidance_command = SimpleCircleFollow(u,circle_gain_struct)
%
% u = aircraft_state
%
% guidance_command = [hdot_c, h_c, chi_dot_c, chi_c, Va_c]
%
% circle_gain_struct = [center, radius, flag, speed, gain]
h_c = -circle_gain_struct.center(3,1);
h_dot_c = 0;
Va_c = circle_gain_struct.speed;


position_inertial = u(1:3,1);

center_inertial = circle_gain_struct.center;

rel_pos = position_inertial - center_inertial;

clock_angle = atan2(rel_pos(2), rel_pos(1));
radial_dist = norm(rel_pos(1:2,1));
rad_error = radial_dist - circle_gain_struct.radius;

chi_0 = clock_angle + circle_gain_struct.flag*pi/2;
chi_c = chi_0 + circle_gain_struct.flag*atan(circle_gain_struct.gain*rad_error/circle_gain_struct.radius);


%%% A hack to keep straigter when far away from desired circle
%chi_dot_c = circle_gain_struct.speed/circle_gain_struct.radius;
scale_factor = 1; %(circle_gain_struct.radius/radial_dist)^4;
chi_dot_c = scale_factor*circle_gain_struct.speed/circle_gain_struct.radius;

guidance_command = [h_dot_c, h_c, chi_dot_c, chi_c, Va_c];