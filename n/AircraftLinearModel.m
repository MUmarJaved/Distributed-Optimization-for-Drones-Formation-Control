function [Alon, Blon, Alat, Blat] = AircraftLinearModel(trim_definition, aircraft_parameters)
%%% Assumes straight, level, horizontal trim condition so beta, q, are zero at trim
%
% Follows definitions from ASEN 6519 using the Beard and Mclain text book


ap = aircraft_parameters;

[trim_variables] = CalculateTrimVariables(trim_definition, ap);
[aircraft_state0, control_surfaces0] = TrimConditionFromDefinitionAndVariables(trim_variables, trim_definition);
utrim = aircraft_state0(7,1);
wtrim = aircraft_state0(9,1);

Va = trim_definition(1);
height = trim_definition(3);
alpha = trim_variables(1);
de = control_surfaces0(1);
dt = control_surfaces0(4);
theta = alpha;

density = stdatmo(height);

inertia_terms = InertiaTerms(aircraft_parameters);

%%%%%%%%%%%%%%%%%%%%%%%
%%% CD derivatives
CLtrim = ap.CL0 + ap.CLalpha*alpha;
CDtrim = ap.CDmin+ap.K*(CLtrim-ap.CLmin)^2;

%CDtrim = ap.CDpa+ap.K*(CLtrim-ap.CLmin)^2; %%%ERROR!!!!!

dCDdCL = 2*ap.K*(CLtrim-ap.CLmin);
CDalpha = dCDdCL*ap.CLalpha;
CDq = dCDdCL*ap.CLq;
CDde = dCDdCL*ap.CLde;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% CX and CZ derivatives
Mx = [-cos(alpha) sin(alpha)];
Mz = [-sin(alpha) -cos(alpha)];

CD0 = ap.CDmin + ap.K*(ap.CL0-ap.CLmin)^2;
%CD0 = ap.CDpa + ap.K*(ap.CL0-ap.CLmin)^2; %%% ERROR!!!

CX0 = Mx*[CD0; ap.CL0];
CZ0 = Mz*[CD0; ap.CL0];

CXalpha = -CDalpha*cos(alpha)+CDtrim*sin(alpha)+ap.CLalpha*sin(alpha)+CLtrim*cos(alpha);
CZalpha = -CDalpha*sin(alpha)-CDtrim*cos(alpha)-ap.CLalpha*cos(alpha)+CLtrim*sin(alpha); 

CXq = Mx*[CDq; ap.CLq];
CZq = Mz*[CDq; ap.CLq]; 

CXde = Mx*[CDde; ap.CLde];
CZde = Mz*[CDde; ap.CLde]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Lateral dimensional derivatives
Yv  = density*ap.S*ap.CYbeta*Va/(2*ap.m);
Yp  = wtrim + density*Va*ap.S*ap.b*ap.CYp/(4*ap.m);
Yr  = -utrim + density*Va*ap.S*ap.b*ap.CYr/(4*ap.m); 
Yda  = density*Va*Va*ap.S*ap.CYda/(2*ap.m);
Ydr  = density*Va*Va*ap.S*ap.CYdr/(2*ap.m);
Lv  = density*ap.S*ap.b*ap.Cpbeta*Va/2;
Lp  = density*Va*ap.S*ap.b*ap.b*ap.Cpp/4;
Lr  = density*Va*ap.S*ap.b*ap.b*ap.Cpr/4;
Lda  = density*Va*Va*ap.S*ap.b*ap.Cpda/2;
Ldr  = density*Va*Va*ap.S*ap.b*ap.Cpdr/2;
Nv  = density*ap.S*ap.b*ap.Crbeta*Va/2;
Np  = density*Va*ap.S*ap.b*ap.b*ap.Crp/4;
Nr  = density*Va*ap.S*ap.b*ap.b*ap.Crr/4;
Nda  = density*Va*Va*ap.S*ap.b*ap.Crda/2;
Ndr  = density*Va*Va*ap.S*ap.b*ap.Crdr/2;

Alat = [Yv Yp Yr ap.g*cos(theta) 0;...
        Lv Lp Lr 0 0;...
        Nv Np Nr 0 0;...
        0 1 tan(theta) 0 0;...
        0 0 sec(theta) 0 0];
Blat = [Yda Ydr;...
        Lda Ldr;...
        Nda Ndr;...
        0 0;...
        0 0];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Longitudinal dimensional derivatives
Xu  = utrim*density*ap.S*(CX0 + CXalpha*alpha+CXde*de)/ap.m - density*ap.S*wtrim*CXalpha/(2*ap.m) + density*ap.Sprop*ap.Cprop*(2*(dt-1)*utrim+ap.kmotor*(1-2*dt))/ap.m;
Xw  = wtrim*density*ap.S*(CX0 + CXalpha*alpha+CXde*de)/ap.m + density*ap.S*utrim*CXalpha/(2*ap.m);
Xq  = -wtrim + density*Va*ap.S*ap.c*CXq/(4*ap.m);
Xde = density*Va*Va*ap.S*CXde/(2*ap.m);
Xdt = density*ap.Sprop*ap.Cprop*(Va*(ap.kmotor-Va)+2*dt*(ap.kmotor-Va)^2)/(ap.m);
Zu  = utrim*density*ap.S*(CZ0 + CZalpha*alpha+CZde*de)/ap.m - density*ap.S*wtrim*CZalpha/(2*ap.m);
Zw  = wtrim*density*ap.S*(CZ0 + CZalpha*alpha+CZde*de)/ap.m + density*ap.S*utrim*CZalpha/(2*ap.m);
Zq  = utrim + density*Va*ap.S*ap.c*CZq/(4*ap.m);
Zde = density*Va*Va*ap.S*CZde/(2*ap.m);
Mu  = utrim*density*ap.S*ap.c*(ap.Cm0+ap.Cmalpha*alpha+ap.Cmde*de)/ap.Iy - density*ap.S*ap.c*ap.Cmalpha*wtrim/(2*ap.Iy);
Mw  = wtrim*density*ap.S*ap.c*(ap.Cm0+ap.Cmalpha*alpha+ap.Cmde*de)/ap.Iy + density*ap.S*ap.c*ap.Cmalpha*utrim/(2*ap.Iy);
Mq  = density*Va*ap.S*ap.c*ap.c*ap.Cmq/(4*ap.Iy);
Mde  = density*Va*Va*ap.S*ap.c*ap.Cmde/(2*ap.Iy);
    
Alon = [Xu Xw Xq -ap.g*cos(theta) 0;...
        Zu Zw Zq -ap.g*sin(theta) 0;...
        Mu Mw Mq 0 0;...
        0 0 1 0 0;...
        sin(theta) -cos(theta) 0 utrim*cos(theta)+wtrim*sin(theta) 0];
Blon = [Xde Xdt;...
        Zde 0;...
        Mde 0;...
        0 0;...
        0 0];