function PlotGuidanceError(time, aircraft_state_array, guidance_struct, col)
%
% Assumes arrays of column vectors
%
%

len = length(time);
center_inertial = guidance_struct.center;
des_rad = guidance_struct.radius;

for i=1:len
    rel_pos(:,i) = aircraft_state_array(1:2,i) - center_inertial(1:2,1);
    radial_dist(i) = norm(rel_pos(1:2,i));
    rad_error(i) = radial_dist(i) - des_rad;
end




%%% Position components
figure(10);
plot(time, rad_error,col); hold on;
ylabel('x_E');
title('Radial Tracking Error');
xlabel('time [sec]');





