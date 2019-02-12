function cost = AeroCostForTrim(trim_variables, trim_definition, aircraft_parameters)
%
%
% trim_definition = [V0; gamma0; rho0]
%
% trim_variables = [alpha0; de0; dt0];
%

[aircraft_state_trim, control_surfaces_trim] = TrimConditionFromDefinitionAndVariables(trim_variables, trim_definition);

rho0=stdatmo(trim_definition(3));

[aircraft_forces, aircraft_moments] = AircraftForcesAndMoments(aircraft_state_trim, control_surfaces_trim, zeros(3,1), rho0, aircraft_parameters);

cost = norm(aircraft_forces) + norm(aircraft_moments); 