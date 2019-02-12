% Eric W. Frew
% ASEN 3128
% CalculateTrimVariables.m
% Created: 2/23/14

function [trim_variables, fval] = CalculateTrimVariablesWithTurn(trim_definition, aircraft_parameters)
%
% Inputs:	trim_definition = [V0; gamma0; h0]
%
%
%
% Outputs:	trim_variables = [alpha0; de0; dt0];
%
%
% Methodology: Uses the fmincon optimization function to minimize the
% magnitude of the total force and total moment acting on the aircraft. The
% function 'AeroCostForTrim' is the cost function being minimized


%%% Initial conditions for the optimizer, start with a known set of trim
%%%% variables
dt0 = .02;
de0 = 0;
alpha0 = 0;
beta0 = 0;
roll0 = 0;
da0 = 0;
dr0 = 0;
%[alpha0, de0] = CalculateSteadyTrim(trim_definition(3), trim_definition(1), aircraft_parameters);

x0 = [alpha0; de0; dt0; roll0; beta0; da0; dr0];



%%% Call the optimization routine that calculates the set of trim_variables
%%% that minimizes 'AeroCostForTrim'

max_angle = 45*pi/180; % constraint that says no angle (alpha or elevator) can have a magnitude greater than this value
LB = [-max_angle;-max_angle; 0;-max_angle;-max_angle;-max_angle;-max_angle ];
UB = [max_angle;max_angle; 1;max_angle;max_angle;max_angle;max_angle];

[trim_variables, fval] = fmincon(@(x)CostForTrimWithTurn(x, trim_definition, aircraft_parameters), x0,[],[],[],[],LB,UB);


