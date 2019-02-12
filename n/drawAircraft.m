function drawAircraft(uu,pts)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);       % inertial Down position
      
    phi      = uu(4);       % roll angle         
    theta    = uu(5);       % pitch angle     
    psi      = uu(6);       % yaw angle     

    u        = uu(7);       % body frame velocities
    v        = uu(8);       
    w        = uu(9); 
    
    p        = uu(10);      % roll rate
    q        = uu(11);      % pitch rate     
    r        = uu(12);      % yaw rate    
    t        = uu(13);      % time

    % define persistent variables 
    persistent aircraft_handle1;
    persistent aircraft_handle2;
    persistent aircraft_handle3;
    persistent aircraft_handle4;

    persistent scale_plot;
    persistent axis_vec;

    
    % first time function is called, initialize plot and persistent vars
    if t<=0.1,
        tlim = 100;
        speed = sqrt(u*u+v*v+w*w);
        plim = speed*tlim;
        scale_plot = plim/60; %plim/30; %plim/60;
        xmin = pn - plim/2;
        xmax = pn + plim/2;
        ymin = pe - plim/2;
        ymax = pe + plim/2;
        zmin = pd - plim/2;
        zmax = pd + plim/2;
        axis_vec = [xmin, xmax, ymin, ymax, zmin, zmax];
        
        figure(1), clf
        aircraft_handle1 = drawAircraftBody(pts.fuse, pn,pe,pd,phi,theta,psi,scale_plot, []);
        aircraft_handle2 = drawAircraftBody(pts.wing, pn,pe,pd,phi,theta,psi,scale_plot, []);
        aircraft_handle3 = drawAircraftBody(pts.tailwing, pn,pe,pd,phi,theta,psi,scale_plot, []);
        aircraft_handle4 = drawAircraftBody(pts.tail, pn,pe,pd,phi,theta,psi,scale_plot, []);
        
        %vel_handle = drawAirRelativeVelocity([pn; pe; pd], [u;v;w], [phi; theta; psi], scale_plot, [], 'normal');

        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis(axis_vec);
        grid on;
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawAircraftBody(pts.fuse, pn,pe,pd,phi,theta,psi, scale_plot, aircraft_handle1);
        drawAircraftBody(pts.wing, pn,pe,pd,phi,theta,psi, scale_plot,aircraft_handle2);
        drawAircraftBody(pts.tailwing, pn,pe,pd,phi,theta,psi, scale_plot,aircraft_handle3);
        drawAircraftBody(pts.tail, pn,pe,pd,phi,theta,psi,scale_plot,aircraft_handle4);
        
        %drawAirRelativeVelocity([pn; pe; pd], [u;v;w], [phi; theta; psi], scale_plot, vel_handle);

        
        [flag, axis_new] = in_view(axis_vec,pn,pe,pd);
        if (flag)
            figure(1);
            axis_vec = axis_new;
            axis(axis_vec);
        end
        
        
    end
end

  




%=======================================================================
% drawAircraftBody
% return handle if 7th argument is empty, otherwise use 7th arg as handle
%=======================================================================
%
function handle = drawAircraftBody(NED,pn,pe,pd,phi,theta,psi, scale_plot, handle)
    SCALE = scale_plot;


  NED = SCALE*TransformFromBodyToInertial(NED, [phi; theta; psi]);
  
  NED = translate(NED,pn,pe,pd); % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
   R = [...
       0, 1, 0;...
       1, 0, 0;...
       0, 0, -1;...
       ];
   XYZ = R*NED;
  
  % plot aircraft
  if isempty(handle),
    handle = plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:));
    hold on;
  else
    set(handle,'XData',XYZ(1,:),'YData',XYZ(2,:),'ZData',XYZ(3,:));
  end
end
 

function handle = drawAirRelativeVelocity(position_inertial, air_rel_vel, euler_angles, scale_plot, handle)
  SCALE = scale_plot/2;


  air_rel_vel_inertial = SCALE*TransformFromBodyToInertial(air_rel_vel, euler_angles);
  
  air_rel_vel_pts = [position_inertial, air_rel_vel_inertial + position_inertial];
 
  
  % transform vertices from NED to XYZ (for matlab rendering)
   R = [...
       0, 1, 0;...
       1, 0, 0;...
       0, 0, -1;...
       ];
   XYZ = R*air_rel_vel_pts;
  
  %plot spacecraft
  if isempty(handle),
    handle = plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:),'ko-', 'LineWidth' ,2 , 'MarkerFaceColor', 'g');
    hold on;
  else
    set(handle,'XData',XYZ(1,:),'YData',XYZ(2,:),'ZData',XYZ(3,:));
  end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)

  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
  
end

% end translateXYZ

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% determines if aircraft is in current set of axis, and if not determines
% new axis
function [flag, axis_new] = in_view(axis_vec,pn,pe,pd)
    xp = pe;
    yp = pn;
    zp = -pd;
    
    flag = 0;
    axis_new = axis_vec;
    
    if (xp<axis_vec(1))
        flag = 1;
        dx = axis_vec(2) - axis_vec(1);
        axis_new(1:2) = axis_vec(1:2)-[dx dx];
    elseif(xp>axis_vec(2))
        flag = 2;
        dx = axis_vec(2) - axis_vec(1);
        axis_new(1:2) = axis_vec(1:2)+[dx dx];
    end

    if (yp<axis_vec(3))
        flag = 3;
        dy = axis_vec(4) - axis_vec(3);
        axis_new(3:4) = axis_vec(3:4)-[dy dy];
    elseif(yp>axis_vec(4))
        flag = 4;
        dy = axis_vec(4) - axis_vec(3);
        axis_new(3:4) = axis_vec(3:4)+[dy dy];
    end
        
    if (zp<axis_vec(5))
        flag = 5;
        dz = axis_vec(6) - axis_vec(5);
        axis_new(5:6) = axis_vec(5:6)-[dz dz];
    elseif(zp>axis_vec(6))
        flag = 6;
        dz = axis_vec(6) - axis_vec(5);
        axis_new(5:6) = axis_vec(5:6)+[dz dz];
    end
  
end

% end
  