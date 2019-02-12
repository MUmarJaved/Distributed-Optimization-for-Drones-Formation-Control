function [aircraft_state, control_surfaces] = TrimConditionFromDefinitionAndVariables(trim_variables, trim_definition)


V0 = trim_definition(1);
gamma0 = trim_definition(2);
h0 = trim_definition(3);

alpha0 = trim_variables(1);
theta0 = gamma0 + alpha0;

position_inertial = [0;0;-h0];
euler_angles = [0;theta0;0];
velocity_body = WindAnglesToAirRelativeVelocityVector([V0; 0; alpha0]);
omega_body = [0;0;0];

aircraft_state = [position_inertial; euler_angles; velocity_body; omega_body];
control_surfaces = [trim_variables(2); 0; 0; trim_variables(3)];


