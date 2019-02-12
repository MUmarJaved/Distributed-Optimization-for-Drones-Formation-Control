function PlotSimulation(time, aircraft_state_array1, col)
%
% Assumes arrays of column vectors
%
%
aircraft_state_array1(2,:)=250*aircraft_state_array1(2,:);

%%% Position components
figure(3);
subplot(311);
plot(time, aircraft_state_array1(1,:),col); hold on;
ylabel('x_E');
title('Inertial Position Components');
subplot(312);
plot(time, aircraft_state_array1(2,:),col); hold on;
ylabel('y_E');
subplot(313);
plot(time, aircraft_state_array1(3,:),col); hold on;
ylabel('z_E');
xlabel('time [sec]');


%%% Euler angles
figure(4);
subplot(311);
plot(time, aircraft_state_array1(4,:)*180/pi,col); hold on;
ylabel('roll [deg]');
title('Euler Angles');
subplot(312);
plot(time, aircraft_state_array1(5,:)*180/pi,col); hold on;
ylabel('pitch [deg]');
subplot(313);
plot(time, aircraft_state_array1(6,:)*180/pi,col); hold on;
ylabel('yaw [deg]');
xlabel('time [sec]');

%%% Velocity
figure(5);
subplot(311);
plot(time, aircraft_state_array1(7,:),col); hold on;
ylabel('u^E');
title('Inertial Velocity in Body Coordinates');
subplot(312);
plot(time, aircraft_state_array1(8,:),col); hold on;
ylabel('v^E');
subplot(313);
plot(time, aircraft_state_array1(9,:),col); hold on;
ylabel('w^E');
xlabel('time [sec]');
figure
plot(time, sqrt(aircraft_state_array1(9,:).^2+aircraft_state_array1(8,:).^2+aircraft_state_array1(9,:).^2),col);
%%% Angular Velocity
figure(6);
subplot(311);
plot(time, aircraft_state_array1(10,:)*180/pi,col); hold on;
ylabel('p [deg/sec]');
title('Inertial Angular Velocity in Body Coordinates');
subplot(312);
plot(time, aircraft_state_array1(11,:)*180/pi,col); hold on;
ylabel('q [deg/sec]');
subplot(313);
plot(time, aircraft_state_array1(12,:)*180/pi,col); hold on;
ylabel('r [deg/sec]');
xlabel('time [sec]');

%%% 3-D plot
% ind_f = length(aircraft_state_array(1,:));
figure(8);
plot3(aircraft_state_array1(1,:),aircraft_state_array1(2,:),-aircraft_state_array1(3,:),col);hold on;
% plot3(aircraft_state_array1(1,1),aircraft_state_array1(2,1),-aircraft_state_array1(3,1),'ks','MarkerFaceColo','g');hold on;
% plot3(aircraft_state_array1(1,ind_f),aircraft_state_array1(2,ind_f),-aircraft_state_array1(3,ind_f),'ko','MarkerFaceColo','r');hold on;

% 
% %%% Replicate the plot from PlotStateVariables
% for i=1:ind_f
% 
%     wind_inertial = background_wind_array(1:3,i);
%     wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state_array(4:6,i));
%     air_rel_body = aircraft_state_array(7:9,i) - wind_body;
%     wind_angles = AirRelativeVelocityVectorToWindAngles(air_rel_body);
%     
%     Va(i) = wind_angles(1);
%     beta(i) = wind_angles(2)*180/pi;
%     alpha(i) = wind_angles(3)*180/pi;
%     
%     [flight_angles] = FlightPathAnglesFromState(aircraft_state_array(1:12,i));
%     chi(i) = 180/pi*flight_angles(2);
%     gamma(i) = 180/pi*flight_angles(3);
% end
% 
%  figure(9), clf
% 
%         subplot(9,2,1)
%         hold on
%         pn_handle = graph_y(time, aircraft_state_array(1,:),col,'p_n', []);
%         
%         subplot(9,2,2)
%         hold on
%         Va_handle = graph_y(time, Va ,col,  'V_a', []);
% 
%         subplot(9,2,3)
%         hold on
%         pe_handle = graph_y(time, aircraft_state_array(2,:),col, 'p_e', []);
% 
%         subplot(9,2,4)
%         hold on
%         alpha_handle = graph_y(time, alpha ,col, '\alpha', []);
% 
%         subplot(9,2,5)
%         hold on
%         h_handle = graph_y(time, -aircraft_state_array(3,:),col, 'h', []);
% 
% 
%         subplot(9,2,6)
%         hold on
%         beta_handle = graph_y(time, beta ,col,  '\beta', []);
% 
%         subplot(9,2,7)
%         hold on
%         phi_handle = graph_y(time, aircraft_state_array(4,:)*180/pi,col,'\phi', []);
%         
%         subplot(9,2,8)
%         hold on
%         p_handle = graph_y(time, aircraft_state_array(10,:)*180/pi,col, 'p', []);
%         
%         subplot(9,2,9)
%         hold on
%         theta_handle = graph_y(time, aircraft_state_array(5,:)*180/pi,col,  '\theta', []);
%         
%         subplot(9,2,10)
%         hold on
%         q_handle = graph_y(time, aircraft_state_array(11,:)*180/pi,col, 'q', []);
%         
%         subplot(9,2,11)
%         hold on
%         psi_handle = graph_y(time, aircraft_state_array(6,:)*180/pi,col, '\psi', []);
%         
%         subplot(9,2,12)
%         hold on
%         r_handle = graph_y(time, aircraft_state_array(12,:)*180/pi,col,  'r', []);
%         
%         
%         subplot(9,2,13)
%         hold on
%         chi_handle = graph_y(time, chi ,col, '\chi', []);
%         
%         subplot(9,2,14)
%         hold on
%         gamma_handle = graph_y(time, gamma ,col, '\gamma', []);
%         
%         
%         subplot(9,2,15)
%         hold on
%         delta_e_handle = graph_y(time, control_input_array(1,:)*180/pi,col, 'de' , []);
%         
%         subplot(9,2,16)
%         hold on
%         delta_a_handle = graph_y(time, control_input_array(2,:)*180/pi,col, 'da' ,[]);
%    
% 
%         subplot(9,2,17)
%         hold on
%         delta_r_handle = graph_y(time, control_input_array(3,:)*180/pi,col, 'dr' ,[]);
%   
%         
%         subplot(9,2,18)
%         hold on
%         delta_t_handle = graph_y(time, control_input_array(4,:),col, 'dt' ,[]);
%         
%         
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % graph y with lable mylabel
% function handle = graph_y(t, y, ind, lab, handle)
%   
%   if isempty(handle),
%     handle    = plot(t,y,ind);
%     ylabel(lab)
%     set(get(gca, 'YLabel'),'Rotation',0.0);
%   else
%     set(handle,'Xdata',[get(handle,'Xdata'),t]);
%     set(handle,'Ydata',[get(handle,'Ydata'),y]);
%     %drawnow
%   end
%         
% 
% 
