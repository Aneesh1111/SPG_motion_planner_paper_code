function angle = wrap(angle, angle_setpoint)

angle = mod(angle-angle_setpoint+pi,2*pi)+angle_setpoint-pi;
