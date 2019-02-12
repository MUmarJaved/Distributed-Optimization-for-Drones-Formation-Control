function [aero_forces, aero_moments] = AerodynamicForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
%
%
% aircraft_state = [xi, yi, zi, roll, pitch, yaw, uE, vE, wE, p, q, r]
%
% aircraft_surfaces = [de da dr dt];
%
% Lift and Drag are calculated in Wind Frame then rotated to body frame
% Thrust is given in Body Frame
% Sideforce calculated in Body Frame
%

%%% redefine states and inputs for ease of use
ap = aircraft_parameters;

euler_angles = aircraft_state(4:6,1);

wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state(4:6,1));
air_rel_vel_body = aircraft_state(7:9,1) - wind_body;

[wind_angles] = AirRelativeVelocityVectorToWindAngles(air_rel_vel_body);
V = wind_angles(1,1);
beta = wind_angles(2,1);
alpha = wind_angles(3,1);

p = aircraft_state(10,1);
q = aircraft_state(11,1);
r = aircraft_state(12,1);

de = aircraft_surfaces(1,1);
da = aircraft_surfaces(2,1);
dr = aircraft_surfaces(3,1);
dt = aircraft_surfaces(4,1);

alpha_dot = 0;


Q = 0.5*density*V*V;

sa = sin(alpha);
ca = cos(alpha);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aerodynamic Forces and Moments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% determine aero force coefficients
CL = ap.CL0 + ap.CLalpha*alpha + ap.CLq*q*ap.c/(2*V) + ap.CLde*de;
%CD = ap.CDmin + ap.K*CL*CL;
CD = ap.CDmin + ap.K*(CL-ap.CLmin)^2;

CX = -CD*ca + CL*sa;
CZ = -CD*sa - CL*ca;

CY = ap.CY0 + ap.CYbeta*beta + ap.CYp*p*ap.b/(2*V) + ap.CYr*r*ap.b/(2*V) + ap.CYda*da + ap.CYdr*dr;

%%% determine aero forces from coeffficients 
X = Q*ap.S*CX;
Y = Q*ap.S*CY;
Z = Q*ap.S*CZ;

aero_forces = [X;Y;Z];

%%% determine aero moment coefficients
% NOTE: The wing span or chord is included here 
Cl = ap.b*[ap.Cl0 + ap.Clbeta*beta + ap.Clp*p*ap.b/(2*V) + ap.Clr*r*ap.b/(2*V) + ap.Clda*da + ap.Cldr*dr];
Cm = ap.c*[ap.Cm0 + ap.Cmalpha*alpha + ap.Cmq*q*ap.c/(2*V) + ap.Cmde*de];
Cn = ap.b*[ap.Cn0 + ap.Cnbeta*beta + ap.Cnp*p*ap.b/(2*V) + ap.Cnr*r*ap.b/(2*V) + ap.Cnda*da + ap.Cndr*dr];

%%% determine aero moments from coeffficients
aero_moments = Q*ap.S*[Cl; Cm; Cn];%[l;m;n];

