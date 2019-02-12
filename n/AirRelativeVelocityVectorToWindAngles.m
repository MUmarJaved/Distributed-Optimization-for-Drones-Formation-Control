function [wind_angles] = AirRelativeVelocityVectorToWindAngles(velocity_body)

V = norm(velocity_body);
alpha = atan2(velocity_body(3,1),velocity_body(1,1));
beta = asin(velocity_body(2,1)/V);

wind_angles = [V; beta; alpha];