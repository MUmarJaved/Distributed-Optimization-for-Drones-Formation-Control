function velocity_body = WindAnglesToAirRelativeVelocityVector(wind_angles)

va = wind_angles(1);
beta = wind_angles(2);
alpha = wind_angles(3);

velocity_body = va*[cos(beta)*cos(alpha); sin(beta); cos(beta)*sin(alpha)]; 