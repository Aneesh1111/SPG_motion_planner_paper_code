function do_quickstop = quickstop_desired(d)

subtarget_direction = d.subtarget.p(1:2)-d.setpoint.p(1:2);
L = norm(subtarget_direction);  % distance from subtarget to setpoint (robot)
if L>1e-6  % if the distance from subtarget2robot is bigger than 1 micrometer then calculate subtarget_direction unit vector
    subtarget_direction = subtarget_direction/L;
else
    subtarget_direction = [ 0 0];  % else nothing
end
vel = d.setpoint.v(1:2);
speed = norm(vel);  % speed
if speed>1e-6  % if speed is bigger than 1 micrometer
    vel = vel/speed;  % vel = unit vector of velocity
else
    vel = [ 0 0]; % else nothing
end

moving_in_opposite_direction = subtarget_direction*vel'<-.5;
v_threshold = 1; %[m/s]
having_high_velocity = speed>v_threshold; % tune parameter later

% aneesh checking if this helps
having_high_velocity = 0;

do_quickstop = (...
        (...
            (moving_in_opposite_direction && having_high_velocity)...
            || (d.subtarget.action==0 && having_high_velocity)...
        )...
            && d.input.robot.human_dribble_flag==0 ...
        )...
        || d.input.robot.quickstop_trigger;

