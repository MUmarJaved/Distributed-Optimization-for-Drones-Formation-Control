% This script simulates formation control of a group of UAVs.
%
% -------> Scale of the formation is NOT controlled in this demo!
%
% -------> IMPORTANT:  CVX must be installed before running! 
%
% -------> Download CVX from:  http://cvxr.com/cvx/
%
%
% (C) Kaveh Fathian, 2017-2018.  Email: kaveh.fathian@gmail.com
%
% This program is a free software: you can redistribute it and/or modify it
% under the terms of the GNU lesser General Public License, 
% either version 3, or any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY. See the GNU Lesser General Public License 
% for more details <http://www.gnu.org/licenses/>.
%
addpath('Helpers');


%% Simulation parameters for triangle formation

% Desired formation coordinates
qs = [0    -2    -2    -4    -4    -4
      0    -1     1    -2     0     2]*sqrt(5)*2;

% Random initial positions
q0 = [ 12.9329    8.2202   10.2059    1.1734    0.7176    0.5700
        6.6439    8.5029    5.5707    6.8453   11.0739   14.3136];

% Random initial heading angles
theta0  = [5.6645    4.2256    1.8902    4.5136    3.6334    5.7688].';   

n       = size(qs,2);       % Number of agents

% Graph adjacency matrix
adj = [ 0     1     1     0     0     0
        1     0     1     1     1     0
        1     1     0     0     1     1
        0     1     0     0     1     0
        0     1     1     1     0     1
        0     0     1     0     1     0];

%% Simulation parameters for square formation

% % Desired formation coordinates
% qs = [0     0     0    -1    -1    -1    -2    -2    -2
%       0    -1    -2     0    -1    -2     0    -1    -2]*10;
% 
% % Random initial positions
% q0 = [18.2114   14.9169    7.6661   11.5099    5.5014    9.0328   16.0890    0.5998    1.7415;
%       16.0112   16.2623   12.3456   10.6010    4.9726    4.5543   19.7221   10.7133   16.0418]*1.5;
% 
% % Random initial heading angles
% theta0  = [6.2150    0.4206    5.9024    0.1142    4.2967    4.9244    3.3561    5.5629    5.6486].';   
% 
% n       = size(qs,2);       % Number of agents
% 
% % Graph adjacency matrix
% adj = [  0     1     0     1     0     0     0     0     0
%          1     0     1     0     1     0     0     0     0
%          0     1     0     0     0     1     0     0     0
%          1     0     0     0     1     0     1     0     0
%          0     1     0     1     0     1     0     1     0
%          0     0     1     0     1     0     0     0     1
%          0     0     0     1     0     0     0     1     0
%          0     0     0     0     1     0     1     0     1
%          0     0     0     0     0     1     0     1     0];
    
%% Parameters

T           = [0, 12];           % Simulation time interval 

vSat        = [3, 5];            % Speed range
omegaSat    = [-pi/4, pi/4];     % Allowed heading angle rate of change
p           = [1; 0];            % Desired travel direction 


%% Computing formation control gains

% Find stabilizing control gains (Needs CVX)
A = FindGains(qs(:), adj);

% % If optimization failed, perturbe 'qs' slightly:
% A = FindGains(qs(:)+0.0001*rand(2*n,1), adj);

%% Simulate the model

Tvec = linspace(T(1), T(2), 50);
state0 = [q0(:); theta0];  % Initial state

% Parameters passed down to the ODE solver
par.n        = n;
par.A        = A;
par.p        = p;
par.vSat     = vSat;
par.omegaSat = omegaSat;

% Simulate the dynamic system
opt = odeset('AbsTol', 1.0e-06, 'RelTol', 1.0e-06);
[t,stateMat] = ode45(@PlaneSys_Ver3_2, Tvec, state0, opt, par);


%% Make movie and plot the results

close all

% Simulation parameters
plotParam.adj       = adj;
plotParam.N         = n;
plotParam.stateMat  = stateMat;
plotParam.trace     = 10;           % Trace length

% Make movie and snapshots
fileName = 'UAVSim';

MoviePlaneVer2_1(fileName, plotParam)






































































































































