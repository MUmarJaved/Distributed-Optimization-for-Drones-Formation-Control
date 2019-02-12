%%% This code was created by Eric W. Frew for ASEN 3128. 
%%% The code was modifed from software provided at http://uavbook.byu.edu/ 
%%% by Randy Beard and Tim McLain for their text book 
%%% "Small Unmanned Aircraft: Theory and Practice".
%%%
%%%

function PlotStateVariables(uu)

    % process inputs to function
    pn          = uu(1);             % North position (meters)
    pe          = uu(2);             % East position (meters)
    h           = -uu(3);            % altitude (meters)

    phi         = 180/pi*uu(4);      % roll angle (degrees)   
    theta       = 180/pi*uu(5);      % pitch angle (degrees)
    psi         = 180/pi*uu(6);             % yaw angle (degrees)
    
    inertial_vel_body = uu(7:9,1);

    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
    
    
    delta_e     = 180/pi*uu(13);     % elevator angle (degrees)
    delta_a     = 180/pi*uu(14);     % aileron angle (degrees)
    delta_r     = 180/pi*uu(15);     % rudder angle (degrees)
    delta_t     = uu(16);            % throttle setting (unitless)
    
    wind_inertial = uu(17:19, 1);
    
    
    pn_c        = uu(20);            % commanded North position (meters)
    pe_c        = uu(21);            % commanded East position (meters)
    h_c         = uu(22);            % commanded altitude (meters)
    Va_c        = uu(23);            % commanded airspeed (meters/s)
    alpha_c     = 180/pi*uu(24);     % commanded angle of attack (degrees)
    beta_c      = 180/pi*uu(25);     % commanded side slip angle (degrees)
    phi_c       = 180/pi*uu(26);     % commanded roll angle (degrees)   
    theta_c     = 180/pi*uu(27);     % commanded pitch angle (degrees)
    chi_c       = 180/pi*uu(28);     % commanded course (degrees)
    p_c         = 180/pi*uu(29);     % commanded body angular rate along x-axis (degrees/s)
    q_c         = 180/pi*uu(30);     % commanded body angular rate along y-axis (degrees/s)
    r_c         = 180/pi*uu(31);     % commanded body angular rate along z-axis (degrees/s)
    
    
    
    t           = uu(32);            % simulation time

    wind_body = TransformFromInertialToBody(wind_inertial, uu(4:6,1));
    air_rel_body = inertial_vel_body - wind_body;
    wind_angles = AirRelativeVelocityVectorToWindAngles(air_rel_body);
    
    Va = wind_angles(1);
    beta = wind_angles(2)*180/pi;
    alpha = wind_angles(3)*180/pi;
    
    [flight_angles] = FlightPathAnglesFromState(uu(1:12,1));
    chi = 180/pi*flight_angles(2);
    gamma = 180/pi*flight_angles(3);
    
 
    
    
    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent h_handle
    persistent Va_handle
    persistent alpha_handle
    persistent beta_handle
    persistent phi_handle
    persistent theta_handle
    persistent psi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    persistent delta_e_handle
    persistent delta_a_handle
    persistent delta_r_handle
    persistent delta_t_handle
    persistent chi_handle
    persistent gamma_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(2), clf

        subplot(9,2,1)
        hold on
        pn_handle = graph_y(t, pn,'p_n', []);
        
        subplot(9,2,2)
        hold on
        %Va_handle = graph_y(t, Va,  'V_a', []);
        Va_handle = graph_y_yd(t, Va, Va_c, 'V_a', []);

        subplot(9,2,3)
        hold on
        pe_handle = graph_y(t, pe, 'p_e', []);

        subplot(9,2,4)
        hold on
        alpha_handle = graph_y(t, alpha, '\alpha', []);

        subplot(9,2,5)
        hold on
        %h_handle = graph_y(t, h, 'h', []);
        h_handle = graph_y_yd(t, h, h_c, 'h', []);

        subplot(9,2,6)
        hold on
        %beta_handle = graph_y(t, beta,  '\beta', []);
        beta_handle = graph_y_yd(t, beta, beta_c, '\beta', []);

        subplot(9,2,7)
        hold on
        %phi_handle = graph_y(t, phi,'\phi', []);
        phi_handle = graph_y_yd(t, phi, phi_c, '\phi', []);
        
        subplot(9,2,8)
        hold on
        p_handle = graph_y(t, p, 'p', []);
        
        subplot(9,2,9)
        hold on
        %theta_handle = graph_y(t, theta,  '\theta', []);
        theta_handle = graph_y_yd(t, theta, theta_c, '\theta', []);
        
        subplot(9,2,10)
        hold on
        q_handle = graph_y(t, q, 'q', []);
        
        subplot(9,2,11)
        hold on
        psi_handle = graph_y(t, psi, '\psi', []);
        
        subplot(9,2,12)
        hold on
        r_handle = graph_y(t, r,  'r', []);
        
        
        subplot(9,2,13)
        hold on
        chi_handle = graph_y_yd(t, chi, chi_c, '\chi', []);
        
        subplot(9,2,14)
        hold on
        gamma_handle = graph_y(t, gamma, '\gamma', []);
        
        
        subplot(9,2,15)
        hold on
        delta_e_handle = graph_y(t, delta_e, 'de' , []);
        
        subplot(9,2,16)
        hold on
        delta_a_handle = graph_y(t, delta_a, 'da' ,[]);
   

        subplot(9,2,17)
        hold on
        delta_r_handle = graph_y(t, delta_r, 'dr' ,[]);
  
        
        subplot(9,2,18)
        hold on
        delta_t_handle = graph_y(t, delta_t, 'dt' ,[]);
        
        


        
    % at every other time step, redraw state variables
    else 
       graph_y(t, pn, 'p_n', pn_handle);
       graph_y(t, pe,  'p_e', pe_handle);
       
       %graph_y(t, h,  'h', h_handle);
       graph_y_yd(t, h, h_c, 'h', h_handle);
       
       %graph_y(t, Va,  'V_a', Va_handle);
       graph_y_yd(t, Va, Va_c, 'V_a', Va_handle);

       graph_y(t, alpha,  '\alpha', alpha_handle);
       
       %graph_y(t, beta,  '\beta', beta_handle);
       graph_y_yd(t, beta, beta_c, '\beta', beta_handle);
       
       %graph_y(t, phi,  '\phi', phi_handle);
       graph_y_yd(t, phi, phi_c, '\phi', phi_handle);
       
       %graph_y(t, theta,  '\theta', theta_handle);
       graph_y_yd(t, theta, theta_c, '\theta',theta_handle);
       graph_y(t, psi,  '\psi', psi_handle);
       graph_y(t, p, 'p', p_handle);
       graph_y(t, q,  'q', q_handle);
       graph_y(t, r,  'r', r_handle);
       graph_y(t, delta_e, 'de', delta_e_handle);
       graph_y(t, delta_a, 'da', delta_a_handle);
       graph_y(t, delta_r, 'dr', delta_r_handle);
       graph_y(t, delta_t, 'dt', delta_t_handle);
       
       graph_y_yd(t, chi, chi_c, '\chi', chi_handle);
       graph_y(t, gamma, '\gamma', gamma_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, lab, handle)
  
  if isempty(handle),
    handle    = plot(t,y);
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low,
      out = low;
  elseif in > high,
      out = high;
  else
      out = in;
  end

% end sat  


