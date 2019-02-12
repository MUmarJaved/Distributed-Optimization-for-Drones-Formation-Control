function cost = CostForTrimWithTurn(trim_variables, trim_definition, aircraft_parameters)
%
%
% trim_definition = [V0; gamma0; height0; radius0]
%
% trim_variables = [alpha0; de0; dt0; beta0; roll0; da0; dr0];
%

[aircraft_state_trim, control_surfaces_trim] = TrimConditionFromDefinitionAndVariablesWithTurn(trim_variables, trim_definition);%%%

rho0=stdatmo(trim_definition(3));

[aircraft_forces, aircraft_moments] = AircraftForcesAndMoments(aircraft_state_trim, control_surfaces_trim, zeros(3,1), rho0, aircraft_parameters);
forces_inertial = TransformFromBodyToInertial(aircraft_forces, aircraft_state_trim(4:6,1));

a_des_inertial = aircraft_parameters.m*[0; trim_definition(1)*trim_definition(1)/trim_definition(4); 0];

%sum_forces = forces_inertial - aircraft_parameters.m*[0; trim_definition(1)*trim_definition(1)/trim_definition(4); 0];

sum_forces = aircraft_forces - TransformFromInertialToBody(a_des_inertial, aircraft_state_trim(4:6,1));

[aero_forces, aero_moments] = AerodynamicForcesAndMoments(aircraft_state_trim, control_surfaces_trim, zeros(3,1), rho0, aircraft_parameters);

cost = norm(sum_forces) + norm(aircraft_moments) + aero_forces(2)^2;