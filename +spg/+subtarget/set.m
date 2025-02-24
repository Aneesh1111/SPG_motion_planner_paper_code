function d = set(d)

robot2ball_dist = norm(d.input.ball.p(1:2) - d.input.robot.p(1:2));
% check if robot is moving in same direction as the ball
robotVel_same_direction_as_ballVel = dot(d.input.robot.v(1:2)/norm(d.input.robot.v(1:2)), d.input.ball.v(1:2)/norm(d.input.ball.v(1:2))) > cos(deg2rad(10));  % less than 10 degrees and we are behind the ball

robot2ball = d.input.ball.p(1:2) - d.input.robot.p(1:2);
behind_ball_flag = dot(robot2ball/norm(robot2ball), d.input.ball.v(1:2)/norm(d.input.ball.v(1:2))) > cos(pi/4);  % be behind ball within +- 45 degrees

if (d.input.robot.human_dribble_flag == 1) && (robot2ball_dist < 3.5) && behind_ball_flag && robotVel_same_direction_as_ballVel
    % if human dribble AND robot has not pushed the ball too far from
    % itself, then you can just intercept the ball without worrying about
    % collisions, etc.
    d.subtarget.p = [d.target.p(1:2) d.subtarget.p(3)];
    d.subtarget.v = d.target.v(1:3);%[clip_vmax(1:2) d.target.v(3)];
    d.subtarget = spg.subtarget.replan.determine_setpoint_limits(d, d.subtarget);
    d.subtarget.target = d.target.p;
    d.subtarget.age = 0;
    d.subtarget.action = 1; %move to target
    return

else
    
    %% update current subtarget

    d.subtarget.p(3) = spg.subtarget.angle.set(d);
    d.subtarget = spg.subtarget.check_collisionfree(d, d.subtarget, 0); 
    d.subtarget.age = d.subtarget.age+1;
    
    %% perform quickstop if required

    if spg.subtarget.replan.quickstop_desired(d)
        d.subtarget = spg.subtarget.replan.quickstop(d, d.subtarget);
        d.subtarget.action = 0; %quickstop
        return
    end

    %% try to move to target

    subtarget_target = spg.subtarget.replan.to_target(d);
    if subtarget_target.collisionfree
        subtarget_target.action = 1; %move to target
        d.subtarget = subtarget_target;
        return
    end

    %% try to replan subtarget if desired

    if spg.subtarget.replan.new_subtarget_desired(d)
        subtarget = spg.subtarget.replan.new_subtarget(d, d.subtarget);
        if subtarget.collisionfree
            subtarget.action = 2; %replan subtarget
            d.subtarget = subtarget;
            return
        end
    end

    %% keep collisionfree subtarget

    if d.subtarget.collisionfree
        d.subtarget.action = 3; %keep subtarget
        return
    end

    %% else quickstop

    d.subtarget = spg.subtarget.replan.quickstop(d, d.subtarget);
    d.subtarget.action = 0; %quickstop
    
end
