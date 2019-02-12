function [force_moment_vec] = S_AeroForcesAndMoments(u, aircraft_parameters)

aircraft_state  = u(1:12,1);
control_inputs  = u(13:16,1);
wind_inertial = u(17:19,1);
density = u(20,1);


%[aero_force, aero_moment] = AeroForcesAndMoments_BodyState_WindCoeffs(aircraft_state, control_inputs, wind_inertial, density, aircraft_parameters);

%force_moment_vec = [aero_force; aero_moment];

[total_force, total_moment] = AircraftForcesAndMoments(aircraft_state, control_inputs, wind_inertial, density, aircraft_parameters);
force_moment_vec = [total_force; total_moment];
