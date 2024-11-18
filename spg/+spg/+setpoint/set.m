function  d = set(d)

%setpoint segments 
% 1: initial state
% 2: after acceleration to maximum speed
% 3: after constant max speed,
% 4: final state after deceleration
if (-(d.par.field_size(1)*.5 + d.par.field_border_margin) < d.input.robot.p(1) && d.input.robot.p(1) < (d.par.field_size(1)*.5 + d.par.field_border_margin) ... % Clip target to inside field
        && -(d.par.field_size(2)*.5 + d.par.field_border_margin) < d.input.robot.p(2) && d.input.robot.p(2) < (d.par.field_size(2)*.5 + d.par.field_border_margin))...  % Clip target to inside field
        || d.subtarget.automatic_substitution_flag==int32(1)

        
    % check if we're tipping, if so, decelerate slower! (hopefully we don't
    % tip then)
    tipping_degrees = 5.0;
    robot_tipping_degrees = d.input.robot.IMU_orientation(1)*d.input.robot.IMU_orientation(1) + d.input.robot.IMU_orientation(2)*d.input.robot.IMU_orientation(2);
    if (sqrt(robot_tipping_degrees) > tipping_degrees)
        dmax = 0.5.*[d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate];
        d.aux.segment = spg.setpoint.get_segments(d.aux.segment, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, dmax);
    else
        d.aux.segment = spg.setpoint.get_segments(d.aux.segment, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
    end
        
    %propagate 1 sample
    d.traj = spg.setpoint.traj1(d.traj, d.aux.segment, d.par.Ts); %propagate only 1 sample
    % d = spg.setpoint.traj_predict(d, d.aux.segment); %propagate only 1 sample
    d.setpoint.p = d.traj.p(1,:);
    d.setpoint.v = d.traj.v(1,:);
    d.setpoint.a = d.traj.a(1,:);
            
    % if we do automatic substitution, make field margin larger
    if d.subtarget.automatic_substitution_flag==1
        
        % clip x-direction trajectory to inside the field (with a margin)
        field_width_half = d.par.field_size(1)*.5 + d.par.field_border_margin + d.par.technical_area_width;  
        if -field_width_half > d.setpoint.p(1,1)
            clipped_x_pos = -field_width_half;
            d.setpoint.p(1) = clipped_x_pos;
        end
        if field_width_half < d.setpoint.p(1,1)
            clipped_x_pos = field_width_half;
            d.setpoint.p(1) = clipped_x_pos;
        end
        dist2sideline = field_width_half - abs(d.traj.p(1,1));
        vx_max = 2*d.par.dmax_move*dist2sideline;
        if d.setpoint.v(1,1) < -vx_max
            clipped_x_vel = -vx_max;
            d.setpoint.v(1) = clipped_x_vel;
        end
        if d.setpoint.v(1,1) > vx_max
            clipped_x_vel = vx_max;
            d.setpoint.v(1) = clipped_x_vel;
        end

        % clip y-direction trajectory to inside the field
        field_length_half = d.par.field_size(2)*.5 + d.par.field_border_margin;
        if -field_length_half > d.setpoint.p(1,2)
            clipped_y_pos = -field_length_half;
            d.setpoint.p(2) = clipped_y_pos;
        end
        if field_length_half < d.setpoint.p(1,2)
            clipped_y_pos = field_length_half;
            d.setpoint.p(2) = clipped_y_pos;
        end
        dist2goalline = field_length_half - abs(d.traj.p(1,2));
        vy_max = 2*d.par.dmax_move*dist2goalline;
        if d.setpoint.v(1,2) < -vy_max
            clipped_y_vel = -vy_max;
            d.setpoint.v(2) = clipped_y_vel;
        end
        if d.setpoint.v(1,2) > vy_max
            clipped_y_vel = vy_max;
            d.setpoint.v(2) = clipped_y_vel;
        end

        % decelerate if velocity is greater than max allowable velocity
        if d.setpoint.v(1) > vx_max
           if d.setpoint.p(1) > 0
               d.setpoint.a(1,1) = -d.par.dmax_move;  % if on right side of field accelerate left 1kHz       
           end
        end
        if d.setpoint.v(1) < -vx_max  % if robot is on left side of field 
           if d.setpoint.p(1) < 0
               d.setpoint.a(1,1) = d.par.dmax_move;
           end
        end
        % decelerate in y-direction if we're going outside of the field
        if d.setpoint.v(2) > vy_max
           if d.setpoint.p(2) > 0
               d.setpoint.a(1,2) = -d.par.dmax_move;  % if on right side of field accelerate left 1kHz       
           end
        end
        if d.setpoint.v(2) < -vy_max  % if robot is on bottom half of field 
           if d.setpoint.p(2) < 0
               d.setpoint.a(1,2) = d.par.dmax_move;
           end
        end
        
    else
    
        % clip x-direction trajectory to inside the field (with a margin)
        field_width_half = d.par.field_size(1)*.5 + d.par.field_border_margin;  
        if -field_width_half > d.setpoint.p(1,1)
            clipped_x_pos = -field_width_half;
            d.setpoint.p(1) = clipped_x_pos;
        end
        if field_width_half < d.setpoint.p(1,1)
            clipped_x_pos = field_width_half;
            d.setpoint.p(1) = clipped_x_pos;
        end
        dist2sideline = field_width_half - abs(d.traj.p(1,1));
        vx_max = 2*d.par.dmax_move*dist2sideline;
        if d.setpoint.v(1,1) < -vx_max
            clipped_x_vel = -vx_max;
            d.setpoint.v(1) = clipped_x_vel;
        end
        if d.setpoint.v(1,1) > vx_max
            clipped_x_vel = vx_max;
            d.setpoint.v(1) = clipped_x_vel;
        end

        % clip y-direction trajectory to inside the field
        field_length_half = d.par.field_size(2)*.5 + d.par.field_border_margin;
        if -field_length_half > d.setpoint.p(1,2)
            clipped_y_pos = -field_length_half;
            d.setpoint.p(2) = clipped_y_pos;
        end
        if field_length_half < d.setpoint.p(1,2)
            clipped_y_pos = field_length_half;
            d.setpoint.p(2) = clipped_y_pos;
        end
        dist2goalline = field_length_half - abs(d.traj.p(1,2));
        vy_max = 2*d.par.dmax_move*dist2goalline;
        if d.setpoint.v(1,2) < -vy_max
            clipped_y_vel = -vy_max;
            d.setpoint.v(2) = clipped_y_vel;
        end
        if d.setpoint.v(1,2) > vy_max
            clipped_y_vel = vy_max;
            d.setpoint.v(2) = clipped_y_vel;
        end

        % decelerate if velocity is greater than max allowable velocity
        if d.setpoint.v(1) > vx_max
           if d.setpoint.p(1) > 0
               d.setpoint.a(1,1) = -d.par.dmax_move;  % if on right side of field accelerate left 1kHz       
           end
        end
        if d.setpoint.v(1) < -vx_max  % if robot is on left side of field 
           if d.setpoint.p(1) < 0
               d.setpoint.a(1,1) = d.par.dmax_move;
           end
        end
        % decelerate in y-direction if we're going outside of the field
        if d.setpoint.v(2) > vy_max
           if d.setpoint.p(2) > 0
               d.setpoint.a(1,2) = -d.par.dmax_move;  % if on right side of field accelerate left 1kHz       
           end
        end
        if d.setpoint.v(2) < -vy_max  % if robot is on bottom half of field 
           if d.setpoint.p(2) < 0
               d.setpoint.a(1,2) = d.par.dmax_move;
           end
        end
        
    end

    
else
    d.setpoint.p = d.input.robot.p;
    d.setpoint.v = [0 0 0];
    d.setpoint.a = [0 0 0];
end



