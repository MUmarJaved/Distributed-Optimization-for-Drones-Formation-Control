function [flight_angles] = FlightPathAnglesFromState(aircraft_state)


vel_inertial = TransformFromBodyToInertial(aircraft_state(7:9,1), aircraft_state(4:6,1));
V = norm(vel_inertial);

if (V==0)
    chi=0;
    gamma=0;
    V=21;
else
    chi = atan2(vel_inertial(2,1), vel_inertial(1,1));
    gamma = -asin(vel_inertial(3,1)/V);
end

flight_angles = [V; chi; gamma];