%% Closed-loop dynamics of the unicycle model with Dubins constraints.
%  The control is augmented to fix the scale of the formation.
%
% (C) Kaveh Fathian, 2017-2018.  Email: kaveh.fathian@gmail.com
%
function V = Speed(t,state,par)

A        = par.A;             % Control gain matrix
n        = par.n;             % Number of agents
omegaSat = par.omegaSat;      % Heading angle saturation 
vSat     = par.vSat;          % Speed saturation
p_i      = par.p;             % Desired direction of motion
Dd       = par.Dd;            % Desired distances

q        = state(1:2*n);      % Position vector
theta    = state(2*n+1: 3*n); % Heading 


%% Current distances

Dc = zeros(n,n); % inter-agent distances in current formation
qMat = reshape(q,2,n);
for i = 1 : n
    for j = i+1 : n
        Dc(i,j) = norm(qMat(:,i)-qMat(:,j), 2);
    end
end
Dc = Dc + Dc';


%% Control to fix the scale

g = 1; % Gain
Ddiff = Dc-Dd;
F = g * atan( kron(Ddiff, ones(2)) );
for i = 1 : n
    for j = setdiff(1:n,i)
        F(2*i-1:2*i,2*i-1:2*i) = F(2*i-1:2*i,2*i-1:2*i) - F(2*i-1:2*i,2*j-1:2*j);
    end
end


%%

H  = zeros(2*n, n);         % Heading matrix
Hp = zeros(2*n, n);         % Perpendicular heading matrix
R  = [0 -1; 1 0];           % 90 degree roation matrix

h = [cos(theta).'; sin(theta).'];
h = h(:);

for i = 1 : n    
    H(2*i-1:2*i,i)  = h(2*i-1:2*i);
    Hp(2*i-1:2*i,i) = R * h(2*i-1:2*i);
end

% Constant speed
p = mean(vSat) * repmat(p_i, n,1);

% Formation control
u = A * q;

% Speed limiter
vCtrl       = H.'* (u + F*q);
vSatMean    = mean(vSat);
vMin        = ones(size(vCtrl)) * (vSat(1)-vSatMean);
vMax        = ones(size(vCtrl)) * (vSat(2)-vSatMean);
vCtrl       = max(vCtrl, vMin);
vCtrl       = min(vCtrl, vMax);

% Heading angle rate of change limiter
omegaCtrl   = Hp.' * (u + F*q);
omegaMin    = ones(size(omegaCtrl)) * omegaSat(1);
omegaMax    = ones(size(omegaCtrl)) * omegaSat(2);
omegaCtrl   = max(omegaCtrl, omegaMin);
omegaCtrl   = min(omegaCtrl, omegaMax);

% Control
v     = H.' * p + vCtrl;
omega = Hp.' * p + omegaCtrl;

% Speed limiter
vMin    = ones(size(v)) * vSat(1);
vMax    = ones(size(v)) * vSat(2);
v       = max(v, vMin);
v       = min(v, vMax);

% Heading angle rate of change limiter
omegaMin = ones(size(omega)) * omegaSat(1);
omegaMax = ones(size(omega)) * omegaSat(2);
omega    = max(omega, omegaMin);
omega    = min(omega, omegaMax);

% Derivative of state
dq      = H * v; 
V=v;
dtheta  = omega;
dstate  = [dq; dtheta]; 


end







































































