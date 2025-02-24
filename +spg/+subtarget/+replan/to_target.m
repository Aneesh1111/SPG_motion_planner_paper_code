function subtarget_target = to_target(d)

%initialize output
subtarget_target = d.subtarget;

% DO THE CLIPPING OF TARGET VELOCITY PROPERLY!!!
% % clip target velocity such that we can actually reach that velocity
% robot_vel = d.setpoint.v(1:2);
% target_vel = d.target.v(1:2);
% dist_between_robot_and_target = norm(d.setpoint.p(1:2) - d.target.p(1:2));
% initial_vel_in_end_direction = dot(robot_vel,target_vel)./(norm(target_vel)^2) .* target_vel;
% clipped_target_v = sqrt(initial_vel_in_end_direction.^2 + 2*d.par.amax_move*dist_between_robot_and_target);
% clipped_target_v_max = min(clipped_target_v, d.par.vmax_move);
% clip_vmax = min(clipped_target_v_max, d.target.v(1:2));

%create subtarget at target
subtarget_target.p = [d.target.p(1:2) d.subtarget.p(3)];
subtarget_target.v = d.target.v(1:3);%[clip_vmax(1:2) d.target.v(3)];
subtarget_target = spg.subtarget.replan.determine_setpoint_limits(d, subtarget_target);
subtarget_target.target = d.target.p;
subtarget_target = spg.subtarget.check_collisionfree(d, subtarget_target, d.par.margin_replan); %check collision towards target
subtarget_target.age = 0;
