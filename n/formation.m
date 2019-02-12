function y = formation(states6)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% This script simulates formation control of a group of UAVs.
%
% -------> Scale of the formation IS controlled in this demo!
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

q0=[states6(1) states6(13) states6(25) states6(37) states6(49) states6(61)
    states6(2) states6(14) states6(26) states6(38) states6(50) states6(62)];
% Random initial heading angles
theta0  = [5.6645    4.2256    1.8902    4.5136    3.6334    5.7688].';   

n       = size(q0,2);       % Number of agents

% Graph adjacency matrix
adj = [ 0     1     1     0     0     0
        1     0     1     1     1     0
        1     1     0     0     1     1
        0     1     0     0     1     0
        0     1     1     1     0     1
        0     0     1     0     1     0];

%% Parameters

T           = [0, 50];           % Simulation time interval 

vSat        = [3, 5];            % Speed range
omegaSat    = [-pi/4, pi/4];     % Allowed heading angle rate of change
p           = [1; 0];            % Desired travel direction 


%% Matrix of desired distances 

% Element (i,j) in matrix Dd describes the distance between agents i and j 
% in the formation. The diagonals are zero by definition.
Dd = zeros(n,n); % inter-agent distances in desired formation
for i = 1 : n
    for j = i+1 : n
       % Dd(i,j) = norm(qs(:,i)-qs(:,j), 2);
       Dd(i,j)=10;
    end
end
Dd = Dd + Dd';


%% Computing formation control gains

% Find stabilizing control gains (Needs CVX)
%A = FindGains(qs(:), adj);
A=[ -0.7779         0   -0.0000   -1.1669    0.7779    1.1669    0.0000    0.0000    0.0000    0.0000   -0.0000   -0.0000;
         0   -0.7779    1.1669   -0.0000   -1.1669    0.7779   -0.0000    0.0000   -0.0000    0.0000    0.0000   -0.0000;
   -0.0000    1.1669   -5.7151         0    3.1254    0.6666    0.8365   -1.6731    1.7532   -0.1604   -0.0000   -0.0000;
   -1.1669   -0.0000         0   -5.7151   -0.6666    3.1254    1.6731    0.8365    0.1604    1.7532    0.0000   -0.0000;
    0.7779   -1.1669    3.1254   -0.6666   -6.4010         0    0.0000    0.0000    1.7072    0.2524    0.7905    1.5810;
    1.1669    0.7779    0.6666    3.1254         0   -6.4010   -0.0000    0.0000   -0.2524    1.7072   -1.5810    0.7905;
    0.0000   -0.0000    0.8365    1.6731    0.0000   -0.0000   -2.0913         0    1.2548   -1.6731    0.0000    0.0000;
    0.0000    0.0000   -1.6731    0.8365    0.0000    0.0000         0   -2.0913    1.6731    1.2548   -0.0000    0.0000;
    0.0000   -0.0000    1.7532    0.1604    1.7072   -0.2524    1.2548    1.6731   -5.9011         0    1.1858   -1.5810;
    0.0000    0.0000   -0.1604    1.7532    0.2524    1.7072   -1.6731    1.2548         0   -5.9011    1.5810    1.1858;
   -0.0000    0.0000   -0.0000    0.0000    0.7905   -1.5810    0.0000   -0.0000    1.1858    1.5810   -1.9763         0;
   -0.0000   -0.0000   -0.0000   -0.0000    1.5810    0.7905    0.0000    0.0000   -1.5810    1.1858         0   -1.9763];
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
par.Dd       = Dd;

% Simulate the dynamic system
opt = odeset('AbsTol', 1.0e-06, 'RelTol', 1.0e-06);
[t,stateMat] = ode45(@PlaneSysDist_Ver3_2, Tvec, state0, opt, par);
for i=1:length(t)
V=Speed(t(i),stateMat(i,:)',par);
end
chi=stateMat(end,13:18)';
y=[V;chi];
end

