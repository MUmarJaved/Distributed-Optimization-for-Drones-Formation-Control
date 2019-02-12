% % This script simulates formation control of a group of UAVs.
% %
% % -------> Scale of the formation IS controlled in this demo!
% %
% % -------> IMPORTANT:  CVX must be installed before running! 
% %
% % -------> Download CVX from:  http://cvxr.com/cvx/
% %
% %
% % (C) Kaveh Fathian, 2017-2018.  Email: kaveh.fathian@gmail.com
% %
% % This program is a free software: you can redistribute it and/or modify it
% % under the terms of the GNU lesser General Public License, 
% % either version 3, or any later version.
% %
% % This program is distributed in the hope that it will be useful,
% % but WITHOUT ANY WARRANTY. See the GNU Lesser General Public License 
% % for more details <http://www.gnu.org/licenses/>.
% %
% addpath('Helpers');
% 
% 
% %% Simulation parameters for triangle formation
% 
% % Desired formation coordinates
% qs = [1    -2    -2    -4    -4    -4
%       1    -1     1    -2     0     2]*sqrt(5)*2;
% 
% % Random initial positions
% q0 = [ 12.9329    8.2202   10.2059    1.1734    0.7176    0.5700
%         6.6439    8.5029    5.5707    6.8453   11.0739   14.3136];
% 
% % Random initial heading angles
% theta0  = [5.6645    4.2256    1.8902    4.5136    3.6334    5.7688].';   
% 
% n       = size(qs,2);       % Number of agents
% 
% % Graph adjacency matrix
% adj = [ 0     1     1     0     0     0
%         1     0     1     1     1     0
%         1     1     0     0     1     1
%         0     1     0     0     1     0
%         0     1     1     1     0     1
%         0     0     1     0     1     0];
% 
% %% Simulation parameters for square formation
% 
% % % Desired formation coordinates
% % qs = [0     0     0    -1    -1    -1    -2    -2    -2
% %       0    -1    -2     0    -1    -2     0    -1    -2]*10;
% % 
% % % Random initial positions
% % q0 = [18.2114   14.9169    7.6661   11.5099    5.5014    9.0328   16.0890    0.5998    1.7415;
% %       16.0112   16.2623   12.3456   10.6010    4.9726    4.5543   19.7221   10.7133   16.0418]*1.5;
% % 
% % % Random initial heading angles
% % theta0  = [6.2150    0.4206    5.9024    0.1142    4.2967    4.9244    3.3561    5.5629    5.6486].';   
% % 
% % n       = size(qs,2);       % Number of agents
% % 
% % % Graph adjacency matrix
% % adj = [  0     1     0     1     0     0     0     0     0
% %          1     0     1     0     1     0     0     0     0
% %          0     1     0     0     0     1     0     0     0
% %          1     0     0     0     1     0     1     0     0
% %          0     1     0     1     0     1     0     1     0
% %          0     0     1     0     1     0     0     0     1
% %          0     0     0     1     0     0     0     1     0
% %          0     0     0     0     1     0     1     0     1
% %          0     0     0     0     0     1     0     1     0];
%     
% %% Parameters
% 
% T           = [0, 40];           % Simulation time interval 
% 
% vSat        = [3, 5];            % Speed range
% omegaSat    = [-pi/4, pi/4];     % Allowed heading angle rate of change
% p           = [1; 0];            % Desired travel direction 
% 
% 
% %% Matrix of desired distances 
% 
% % Element (i,j) in matrix Dd describes the distance between agents i and j 
% % in the formation. The diagonals are zero by definition.
% Dd = zeros(n,n); % inter-agent distances in desired formation
% for i = 1 : n
%     for j = i+1 : n
%        % Dd(i,j) = norm(qs(:,i)-qs(:,j), 2);
%        Dd(i,j)=10;
%     end
% end
% Dd = Dd + Dd';
% 
% 
% %% Computing formation control gains
% 
% % Find stabilizing control gains (Needs CVX)
% A = FindGains(qs(:), adj);
% 
% % % If optimization failed, perturbe 'qs' slightly:
% % A = FindGains(qs(:)+0.0001*rand(2*n,1), adj);
% 
% %% Simulate the model
% 
% Tvec = linspace(T(1), T(2), 50);
% state0 = [q0(:); theta0];  % Initial state
% 
% % Parameters passed down to the ODE solver
% par.n        = n;
% par.A        = A;
% par.p        = p;
% par.vSat     = vSat;
% par.omegaSat = omegaSat;
% par.Dd       = Dd;
% 
% % Simulate the dynamic system
% opt = odeset('AbsTol', 1.0e-06, 'RelTol', 1.0e-06);
% [t,stateMat] = ode45(@PlaneSysDist_Ver3_2, Tvec, state0, opt, par);
% V=zeros(6,1);
% for i=1:length(t)
% q=Speed(t(i),stateMat(i,:)',par);
% V=[V q];
% end
% V=V(:,2:end)';
% close all;
% %plot(X1,Y4,'Parent',subplot3,'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);
% 
% figure(1)
% subplot(411);hold on;
% set(gca,'FontSize',24);
% %plot(t(2:end),sqrt(stateMat(2:end,1).^2+stateMat(2:end,2).^2)./t(2:end));%hold on; plot(t,stateMat(:,1));hold on;
% plot(t,stateMat(:,1),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('x_1')
% subplot(412);hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,2),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('y_1')
% subplot(413);hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,13),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('\chi_1')
% subplot(414);hold on;
% set(gca,'FontSize',24);
% plot(t,V(:,1),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('V_1')
% figure(2)
% %plot(t(2:end),sqrt(stateMat(2:end,3).^2+stateMat(2:end,4).^2)./t(2:end));%
% subplot(411)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,3),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('x_2')
% subplot(412)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,4),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('y_2')
% subplot(413)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,14),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('\chi_2')
% subplot(414)
% hold on;
% set(gca,'FontSize',24);
% plot(t,V(:,2),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('V_2')
% figure(3)
% subplot(411)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,5),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('x_3')
% subplot(412)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,6),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('y_3')
% subplot(413)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,15),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('\chi_3')
% subplot(414)
% hold on;
% set(gca,'FontSize',24);
% plot(t,V(:,3),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('V_3')
% figure(4)
% subplot(411)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,7),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('x_4')
% subplot(412)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,8),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('y_4')
% subplot(413)
% hold on;
% set(gca,'FontSize',24);
% ;plot(t,stateMat(:,16),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('\chi_4')
% subplot(414)
% hold on;
% set(gca,'FontSize',24);
% plot(t,V(:,4),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('V_4')
% figure(5)
% subplot(411)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,9),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('x_5')
% subplot(412)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,10),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('y_5')
% subplot(413)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,17),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('\chi_5')
% subplot(414)
% hold on;
% set(gca,'FontSize',24);
% plot(t,V(:,5),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('V_5')
% figure(6)
% subplot(411)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,11),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('x_6')
% subplot(412)
% hold on;
% set(gca,'FontSize',24);
% plot(t,stateMat(:,12),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('y_6')
% subplot(413)
% hold on;
% set(gca,'FontSize',24);
% ;plot(t,stateMat(:,18),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('\chi_6')
% subplot(414)
% hold on;
% set(gca,'FontSize',24);
% plot(t,V(:,6),'LineWidth',3,   'Color',[0.63921568627451 0.07843137254902 0.180392156862745]);hold on;
% ylabel('V_6')
% figure(7)
% plot(stateMat(:,1),stateMat(:,2),'b');hold on;
% plot(stateMat(:,3),stateMat(:,4),'r');hold on;
% plot(stateMat(:,5),stateMat(:,6),'g');hold on;
% plot(stateMat(:,7),stateMat(:,8),'k');hold on;
% plot(stateMat(:,9),stateMat(:,10),'c');hold on;
% plot(stateMat(:,11),stateMat(:,12),'y');hold on;
% legend('1','2','3','4','5','6')
% 
% 
% %% Final distances
% 
% % Element (i,j) in 'Dc' describes the distance between agents i and j 
% % in the formation. The diagonals are zero by definition.
% Dc = zeros(n,n); % inter-agent distances in current formation
% Cc = zeros(n,n);
% qMat = reshape(stateMat(end,1:2*n),2,n);
% for i = 1 : n
%     for j = i+1 : n
%         Dc(i,j) = norm(qMat(:,i)-qMat(:,j), 2);
%         Cc(i,j)=10;
%         
%     end
% end
% Dc = Dc + Dc';
% Cc=Cc+Cc';
% y_r=graph(abs(Dc-Cc));
% figure(8)
% plot(y_r,'EdgeLabel',y_r.Edges.Weight);
% % 
% % 
% % %% Make movie and plot the results
% % 
% % close all
% % 
% % % Simulation parameters
% % plotParam.adj       = adj;
% % plotParam.N         = n;
% % plotParam.stateMat  = stateMat;
% % plotParam.trace     = 10;           % Trace length
% % 
% % % Make movie and snapshots
% % fileName = 'UAVDistSim';
% % 
% MoviePlaneVer2_1(fileName, plotParam)
close all
it=0:14;
pstep=[0.000 0.975 1.000 0.990 0.908 0.641 0.616 1.000 0.913 0.979 0.976 0.986 0.977 1 1];
dstep=[0.000 0.018 0.981 0.989 0.736 0.604 0.184 0.568 0.784 0.704 0.903 0.985 0.985 0.976 0.985];
gap=[3.6e+06 9.8e+05 1.3e+04 1.2e+02 1.3e+01 5.7e+00 3.4e+00 9.8e-01 1.6e-01 2.7e-02 2.2e-03 1.1e-04 4.5e-06 2.0e-07 8.5e-09];
prim_obj=[1.000000e+02 1.877928e+04 8.698169e+03 1.117371e+02 1.242764e+01 5.339426e+00 3.084544e+00 1.440823e+00 1.032011e+00  9.577734e-01 9.511950e-01 9.509428e-01 9.509392e-01 9.509392e-01 9.509391e-01];
dual_obj=[0.000000e+00 -5.456462e+01 -1.808770e+02 -1.910151e+00 1.804474e-01 -1.753253e-03  1.995994e-01 6.319251e-01 8.923692e-01 9.346569e-01 9.493679e-01 9.509161e-01 9.509388e-01 9.509391e-01 9.509391e-01];

figure(9)
plot(it,pstep);hold on; 
plot(it,dstep);hold on;
legend('pstep','dstep')
figure(10)
plot(it,gap);
ylabel('duality gap')
figure(11)
plot(it,prim_obj);hold on;
plot(it,dual_obj)
ylabel('prim-obj')



































% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
