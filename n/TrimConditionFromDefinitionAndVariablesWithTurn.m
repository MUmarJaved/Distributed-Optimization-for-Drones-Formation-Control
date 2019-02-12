function [aircraft_state, control_surfaces] = TrimConditionFromDefinitionAndVariablesWithTurn(trim_variables, trim_definition)
%
% trim_definition = [V0; gamma0; height0; radius0]
%
% trim_variables = [alpha0; de0; dt0; roll0; beta0; da0; dr0];
%

V0 = trim_definition(1);
gamma0 = trim_definition(2);
h0 = trim_definition(3);
rad0 = trim_definition(4);

alpha0 = trim_variables(1);
beta0 = trim_variables(5);
theta0 = gamma0 + alpha0;

%%% coordinated turn equations
chi_dot = V0/rad0;
g = 9.81;
phi0 = trim_variables(4);%atan(chi_dot*V0/g);



position_inertial = [0;0;-h0];
euler_angles = [phi0;theta0;0];
velocity_body = WindAnglesToAirRelativeVelocityVector([V0; beta0; alpha0]);
omega_body = chi_dot*[-sin(theta0); sin(phi0)*cos(theta0); cos(phi0)*cos(theta0)];

aircraft_state = [position_inertial; euler_angles; velocity_body; omega_body];
control_surfaces = [trim_variables(2); trim_variables(6); trim_variables(7); trim_variables(3)];


