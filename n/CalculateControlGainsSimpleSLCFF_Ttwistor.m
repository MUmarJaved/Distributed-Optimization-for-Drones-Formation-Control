function [control_gains]=CalculateControlGainsSimpleSLC_Ttwistor(trim_variables, trim_definition, aircraft_parameters)
%
% function [control_gains, linear_terms]=CalculateControlGainsSimpleSLC_Nondim(aircraft_parameters)
%
% This function determines the control gains that are required for
% SimpleSLCAutopilot.m. It assumes that the aircraft_parameters structure
% has the NONDIMENSIONAL aircraft coefficients. It further assumes the trim
% condition is straight, level flight.
%

g=aircraft_parameters.g;
control_gains.g=g;

Va = trim_definition(1);

density = stdatmo(trim_definition(3));

alpha_trim = trim_variables(1);
de0 = trim_variables(2);
dt0 = trim_variables(3);

CL_trim = aircraft_parameters.CL0 + aircraft_parameters.CLalpha*alpha_trim + aircraft_parameters.CLde*de0;
CD_trim = aircraft_parameters.CDmin + aircraft_parameters.K*(CL_trim - aircraft_parameters.CLmin)^2;

QS = 0.5*density*Va*Va*aircraft_parameters.S;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Actuator and state saturation limits
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
control_gains.max_roll      = 45*pi/180;
control_gains.max_roll_rate = 45*pi/180;
control_gains.max_pitch     = 30*pi/180;
control_gains.max_da        = 30*pi/180;
control_gains.max_dr        = 30*pi/180;
control_gains.max_de        = 45*pi/180;


%%%%%%%%%%%%%%%%%%%%%%%%
% Linear model terms
%%%%%%%%%%%%%%%%%%%%%%%

% roll
a_phi1 = -QS*aircraft_parameters.b*aircraft_parameters.Cpp*aircraft_parameters.b/(2*Va);
a_phi2 = QS*aircraft_parameters.b*aircraft_parameters.Cpda;

% sideslip
a_beta1 = -density*Va*aircraft_parameters.S*aircraft_parameters.CYbeta/(2*aircraft_parameters.m);
a_beta2 = density*Va*aircraft_parameters.S*aircraft_parameters.CYdr/(2*aircraft_parameters.m);

% pitch 
a_theta1 = -density*Va*aircraft_parameters.c*aircraft_parameters.S*aircraft_parameters.Cmq*aircraft_parameters.c/(4*aircraft_parameters.Iy);
a_theta2 = -density*Va*Va*aircraft_parameters.c*aircraft_parameters.S*aircraft_parameters.Cmalpha/(2*aircraft_parameters.Iy);
a_theta3 = density*Va*Va*aircraft_parameters.c*aircraft_parameters.S*aircraft_parameters.Cmde/(2*aircraft_parameters.Iy);

% airspeed (from pitch)
a_v1 = (density/aircraft_parameters.m)*(Va*aircraft_parameters.S*CD_trim -  aircraft_parameters.Sprop*aircraft_parameters.Cprop*(2*(dt0-1)*Va + aircraft_parameters.kmotor*(1-2*dt0)));
a_v2 = density*aircraft_parameters.Sprop*aircraft_parameters.Cprop*(Va*(aircraft_parameters.kmotor-Va)+2*dt0*(aircraft_parameters.kmotor-Va)^2)/aircraft_parameters.m;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longitudinal control gains
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%
% pitch hold
e_theta_max = 2*control_gains.max_pitch;% 2*control_gains.max_pitch; % used by saturation method to select proportional gain, assume never give step commanded of greater than full pitch limit
zeta_pitch = 1.6; %2.8;%1.6;

control_gains.Kp_pitch = (control_gains.max_de / e_theta_max)*sign(a_theta3);
wn_pitch = sqrt(a_theta2 + control_gains.Kp_pitch*a_theta3);
control_gains.Kd_pitch = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3;

%%%%%%%%
% height hold
Kpitch_DC = a_theta3*control_gains.Kp_pitch/(a_theta3*control_gains.Kp_pitch+a_theta2);
wn_height = (1/100)*wn_pitch;
zeta_height = 1.8;

control_gains.Kp_height = 2*zeta_height*wn_height/(Kpitch_DC*Va);
control_gains.Ki_height = wn_height*wn_height/(Kpitch_DC*Va);

%%%%%%%%%
% airspeed (from throttle)
wn_airspeed = (1/100)*wn_pitch;
zeta_airspeed = 8;
control_gains.Kp_speed_throttle = (2*zeta_airspeed*wn_airspeed-a_v1)/a_v2;
control_gains.Ki_speed_throttle = wn_airspeed*wn_airspeed/a_v2;

%%%%%%%%%
% airspeed (from pitch)

wn_airspeed = (1/100)*wn_pitch;
zeta_airspeed = 8;

control_gains.Kp_speed_pitch = (a_v1 - 2*zeta_airspeed*wn_airspeed)/(Kpitch_DC*aircraft_parameters.g);
control_gains.Ki_speed_pitch = -wn_airspeed*wn_airspeed/(Kpitch_DC*aircraft_parameters.g);


%%%%%%%%
% height control state machine parameters
control_gains.Kpitch_DC = Kpitch_DC;
control_gains.takeoff_height = 10;
control_gains.takeoff_pitch = 12*pi/180;
control_gains.height_hold_limit = 25;
control_gains.climb_throttle = 0.5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longitudinal control gains
%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%
% % roll hold gains
% zeta_roll = 1.2;
% e_phi_max = control_gains.max_roll; % used by saturation method to select proportional gain, assume never give step commanded of greater than full roll limit
% 
% control_gains.Kp_roll = control_gains.max_da/e_phi_max*sign(a_phi2);
% wn_roll = sqrt(abs(a_phi2*control_gains.Kp_roll));
% 
% control_gains.Kd_roll = (2*zeta_roll*wn_roll - a_phi1)/a_phi2;
% 
% control_gains.Ki_roll = -.05;
% 
% den_roll = [1 a_phi1+control_gains.Kd_roll*a_phi2 a_phi2*control_gains.Kp_roll control_gains.Ki_roll];
% roll_poles = roots(den_roll);


control_gains.Ki_roll = 0*1.5;%0
control_gains.Kp_roll = 2*control_gains.max_roll_rate/control_gains.max_roll;%10;
control_gains.Kd_roll = -0.8;%-.3;%.3

% %%%%%%%
% % course hold gains
% wn_chi = (1/100)*wn_roll;%(1/100)*wn_roll;
% zeta_chi = 4;%12;
% 
% control_gains.Kp_course = 2*zeta_chi*wn_chi*Va/g;
% control_gains.Ki_course = wn_chi*wn_chi*Va/g;

%%%%%%%%%
% desired course rate gains 
control_gains.Kff_course_rate = 1;
control_gains.Kp_course_rate = control_gains.max_roll/(90*pi/180);


%%%%%%%%%
% sideslip hold gains
e_beta_max = 5*pi/180;%control_gains.max_roll; % used by saturation method to select proportional gain, assume never give step commanded of greater than full roll limit
zeta_beta = 10;

control_gains.Kp_beta = (control_gains.max_dr / e_beta_max);
wn_beta = (a_beta1 + a_beta2*control_gains.Kp_beta)/(2*zeta_beta);

control_gains.Ki_beta = (wn_beta*wn_beta)/a_beta2;
control_gains.Kd_beta = 2;

%%%%%%%%%
% feedforward terms
control_gains.Kff_de = -(aircraft_parameters.Cmq*aircraft_parameters.c/(2*Va*aircraft_parameters.Cmde));
control_gains.Kff_da = -(aircraft_parameters.Clp*aircraft_parameters.b/(2*Va*aircraft_parameters.Clda));
control_gains.Kff_dr = -(aircraft_parameters.Cnr*aircraft_parameters.b/(2*Va*aircraft_parameters.Cndr));
%control_gains.Kff_dr = -(aircraft_parameters.CYr*aircraft_parameters.b/(2*Va*aircraft_parameters.CYdr))





