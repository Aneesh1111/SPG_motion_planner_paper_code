% function [subtarget, p_robot, p_obstacles_traj] = check_collisionfree(d, subtarget, obstacle_margin)
% 
% %updates: subtarget.segment, subtarget.collisionfree, subtarget.eta
% 
% %% determine robot trajectory prediction
% 
% subtarget.segment = spg.setpoint.get_segments(subtarget.segment, d.setpoint.p, d.setpoint.v, subtarget.p, subtarget.v, subtarget.vmax, subtarget.amax, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
% subtarget.eta = max(subtarget.segment(3).t(1:2));
% d = spg.setpoint.traj_predict(d, subtarget.segment);
% 
% subtarget.segment_id = d.traj.segment_id(1,1:3);
% p_robot = d.traj.p;
% v_robot = d.traj.v;
% 
% %% preparations for trajectory check
% 
% p_obstacles = d.input.obstacles.p(d.input.obstacles.active,:);
% v_obstacles = d.input.obstacles.v(d.input.obstacles.active,:);
% r_obstacles = d.input.obstacles.r(d.input.obstacles.active);
% collision_distance = d.par.robot_radius+r_obstacles;
% 
% violation = struct('collisionfree', true, 'count', 0, 'SubtargetAvoidPolygon', 0, 'obstacle', 1e10, 'field', 1e10);
% 
% npropagate = size(d.traj.p,1); %amount of samples to propagate trajectory
% p_obstacles_traj = nan([size(p_obstacles) npropagate]);
% 
% %% if trajectory ended with nonzero velocity
% 
% %if norm(d.setpoint.p(1:2)-subtarget.p(1:2))<1e-3 && norm(subtarget.v(1:2))>1e-3
% %    subtarget.collisionfree = false;
% %    return
% %end
% 
% %% propagate to assess trajectory feasibility
% 
% for i=1:npropagate
% 
%     %obstacle path (constant/declining velocity model)
%     v_obstacles = v_obstacles*d.par.obstacle_vel_gain;
%     p_obstacles = p_obstacles+v_obstacles*d.par.Ts_predict; %Aneesh: could make Ts_predict a number that changes with obstacle velocity?
%     p_obstacles_traj(:,:,i) = p_obstacles;
% 
%     %check collision --> WAAN: only when there are obstacles (min in C does not work on empty array)
%     if isempty(p_obstacles)
%         violation = update_violation(violation, 'obstacle', 0);
%     else
%         %minimum obstacle distance
%         p_diff = bsxfun(@minus,p_obstacles,p_robot(i,1:2));
%         p_diff_norm = sqrt(sum(p_diff.^2,2));
% 
%         % if we're intercepting and we're closer to the target than the
%         % opponent, then I don't mind a collision because I will win the
%         % referees decision
%         p_diff_obst2Target = min(vecnorm(abs(bsxfun(@minus,p_obstacles,d.target.p(1:2))),2,2));
%         p_diff_robot2Target = bsxfun(@minus,p_robot(i,1:2),d.target.p(1:2));
%         target2ball_distance = norm(d.target.p(1:2) - d.input.ball.p(1:2));
%         margin = d.input.robot.dist2ball_vs_opp;
%         ball2obstacle = bsxfun(@minus,p_obstacles,d.input.ball.p(1:2));
%         robot2ball = p_robot(i,1:2) - d.input.ball.p(1:2);
%         
%         angle = zeros(1,size(ball2obstacle,1));
%         for j=1:size(ball2obstacle,1)
%             norm_arr = norm(cross([ball2obstacle(j,1:2) 0], [robot2ball 0]));
%             dot_arr = dot(ball2obstacle(j,1:2), robot2ball);
%             angle(j) = abs(rad2deg(atan2(norm_arr, dot_arr)));
%         end
%         no_obstacle_between_me_and_ball = all(angle >= 45);
%                         
%         if ((d.input.robot.skillID == 5) || (d.input.robot.skillID == 0))...
%                 && (p_diff_obst2Target + margin > norm(p_diff_robot2Target(1:2)))...
%                 && (target2ball_distance < 0.5)...
%                 && no_obstacle_between_me_and_ball
%             violation_value = 0;
%         elseif (d.input.robot.skillID == 5) && (p_diff_obst2Target > norm(p_diff_robot2Target(1:2)))
%             violation_value = 0;
%         else
%             obs_violation = max(0, (collision_distance+obstacle_margin*norm(v_robot(i,1:2)))-p_diff_norm);
%             violation_value = max(obs_violation);
%         end
% 
%         %scaled projection point
%         violation = update_violation(violation, 'obstacle', violation_value);
%     end
% 
%     % check illegal driving zone
%     if d.input.SubtargetAvoidPolygon.valid
%         if inpolygon(p_robot(i,1), p_robot(i,2), d.input.SubtargetAvoidPolygon.polygon(1,:), d.input.SubtargetAvoidPolygon.polygon(2,:)) && ...
%             ~inpolygon(d.target.p(1), d.target.p(2), d.input.SubtargetAvoidPolygon.polygon(1,:), d.input.SubtargetAvoidPolygon.polygon(2,:)) && ...
%             ~inpolygon(p_robot(1,1), p_robot(1,2), d.input.SubtargetAvoidPolygon.polygon(1,:), d.input.SubtargetAvoidPolygon.polygon(2,:))
%             violation = update_violation(violation, 'SubtargetAvoidPolygon', 1);
%         else
%             violation = update_violation(violation, 'SubtargetAvoidPolygon', 0);
%         end
%     else
%         violation = update_violation(violation, 'SubtargetAvoidPolygon', 0);
%     end
% 
%     % check field if possess ball
%     if norm(subtarget.p(1:2)-d.target.p(1:2))>1e-3
%         if any(d.input.robot.skillID==1:4) %ball possession
%             robot_ball_distance = d.par.robot_radius+d.par.ball_radius; %[m]
%             ball_pos = p_robot(i,1:2)+robot_ball_distance*[-sin(p_robot(i,3)) cos(p_robot(i,3))];
%             violation_value = max([0 abs(ball_pos)-d.par.field_size*.5]);
%             violation = update_violation(violation, 'field', violation_value);
%         else
%             violation = update_violation(violation, 'field', 0);
%         end
%         if -(d.par.field_size(1)*.5 + d.par.field_border_margin) < subtarget.p(1) && subtarget.p(1) < (d.par.field_size(1)*.5 + d.par.field_border_margin) ... % Clip subtarget to inside field
%             && -(d.par.field_size(2)*.5 + d.par.field_border_margin) < subtarget.p(2) && subtarget.p(2) < (d.par.field_size(2)*.5 + d.par.field_border_margin) % Clip subtarget to inside field
% 
%             % clip to 3m radius if dribbling
%             max_radius = 3.0;
%             if d.input.robot.human_dribble_flag == 1
%                 distance_between_cpb_poi_xy_and_subtarget = 0.0;
%             else
%                 distance_between_cpb_poi_xy_and_subtarget = norm(subtarget.p(1:2) - d.input.robot.cpb_poi_xy(1:2));
%             end
% 
%             % don't go inside goal area
%             xpos = .5*d.par.field_goal_area(1);
%             ypos = d.par.field_size(2)*.5-d.par.field_goal_area(2);
%             subtarget_is_in_goal_area = abs(subtarget.p(1))<xpos & abs(subtarget.p(2))>ypos;
% 
%             if (any(d.input.robot.skillID==1:4) && (distance_between_cpb_poi_xy_and_subtarget > max_radius)) || (subtarget_is_in_goal_area)
%                 violation_value = 1e11;
%             else
%                 violation_value = 0;
%             end
% 
%         else
%             if (d.subtarget.automatic_substitution_flag==1) && -(d.par.field_size(1)*.5 + d.par.field_border_margin + d.par.technical_area_width) < subtarget.p(1) && subtarget.p(1) < (d.par.field_size(1)*.5 + d.par.field_border_margin + d.par.technical_area_width) ... % Clip subtarget to inside field + technical area margin
%             && -(d.par.field_size(2)*.5 + d.par.field_border_margin) < subtarget.p(2) && subtarget.p(2) < (d.par.field_size(2)*.5 + d.par.field_border_margin) % Clip subtarget to inside field + technical area margin
%                 violation_value = 0;
%             else
%                 violation_value = 1e11;
%             end
%         end
%         violation = update_violation(violation, 'field', violation_value);
%     end
% 
%     %check penalty area
% %     violation_value = spg.subtarget.replan.get_distance_inside_penalty_area(d, p_robot(i,:));
% %     violation = update_violation(violation, 'penalty_area', violation_value);
% 
%     %check if close to arrival
% %     if all(d.traj.segment_id(i,1:2)==3)
% %         break
% %     end
%     if norm(p_robot(i,1:2)-subtarget.p(1:2))<1e-2
%         break
%     end
% end
% 
% % check for subtarget to target collision free path
% subtarget2target_line = d.target.p(1:2) - subtarget.p(1:2);
% radius = mean(collision_distance);
% % quadratic equation to find intersection: (a x^2 + b x + c = 0)
% a = norm(subtarget2target_line)^2;
% b = 2 * (subtarget2target_line(1)*(subtarget.p(1)-p_obstacles(:,1)) + subtarget2target_line(2)*(subtarget.p(2)-p_obstacles(:,2)));
% c = (subtarget.p(1)-p_obstacles(:,1)).^2 + (subtarget.p(2)-p_obstacles(:,2)).^2 - radius^2;
% D = b.^2 - 4*a*c;
% valid_discriminant = D>=0;
% intersection1 = (-b + sqrt(D))/2/a;
% intersection2 = (-b - sqrt(D))/2/a;
% valid_intersection1 = (0 <= intersection1) & (intersection1 <=1);
% valid_intersection2 = (0 <= intersection2) & (intersection2 <=1);
% intersects = any(valid_discriminant & (valid_intersection1 | valid_intersection2));
% % if obstacle obstructs line-of-sight from subtarget to target, then I don't want to choose that subtarget (avoids getting stuck in local-minima)
% if intersects
%     violation = update_violation(violation, 'obstacle', 1e11);
% end
% 
% % clip subtarget x-velocity such that robot stays in field (can brake in time)
% dist2sideline = (d.par.field_size(1)*.5 + d.par.field_border_margin) - abs(subtarget.p(1));
% if abs(2*d.par.dmax_move*dist2sideline) < subtarget.v(1)^2
%     subtarget.v(1) = 2*d.par.dmax_move*dist2sideline;
% end
% % clip subtarget y-velocity such that robot stays in field (can brake in time)
% dist2goalline = (d.par.field_size(2)*.5 + d.par.field_border_margin) - abs(subtarget.p(2));
% if abs(2*d.par.dmax_move*dist2goalline) < subtarget.v(2)^2
%     subtarget.v(2) = 2*d.par.dmax_move*dist2goalline;
% end
% 
% subtarget.collisionfree = violation.collisionfree;
% subtarget.violation_count = violation.count;
% 
% function violation = update_violation(violation, field, violation_value)
% if violation_value>0
%     violation.count = violation.count+1;
% end
% if violation_value > violation.(field)
%     violation.collisionfree = false;
% else
%     violation.(field) = violation_value;
% end


function [subtarget, p_robot, p_obstacles_traj] = check_collisionfree(d, subtarget, obstacle_margin)

%updates: subtarget.segment, subtarget.collisionfree, subtarget.eta

%% determine robot trajectory prediction

subtarget.segment = spg.setpoint.get_segments(subtarget.segment, d.setpoint.p, d.setpoint.v, subtarget.p, subtarget.v, subtarget.vmax, subtarget.amax, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
subtarget.eta = max(subtarget.segment(3).t(1:2));
d = spg.setpoint.traj_predict(d, subtarget.segment);

subtarget.segment_id = d.traj.segment_id(1,1:3);
p_robot = d.traj.p;
v_robot = d.traj.v;

%% preparations for trajectory check

p_obstacles = d.input.obstacles.p(d.input.obstacles.active,:);
v_obstacles = d.input.obstacles.v(d.input.obstacles.active,:);
r_obstacles = d.input.obstacles.r(d.input.obstacles.active);
collision_distance = d.par.robot_radius+r_obstacles;

violation = struct('collisionfree', true, 'count', 0, 'SubtargetAvoidPolygon', 0, 'obstacle', 1e10, 'field', 1e10);

npropagate = size(d.traj.p,1); %amount of samples to propagate trajectory
p_obstacles_traj = nan([size(p_obstacles) npropagate]);

%% if trajectory ended with nonzero velocity

%if norm(d.setpoint.p(1:2)-subtarget.p(1:2))<1e-3 && norm(subtarget.v(1:2))>1e-3
%    subtarget.collisionfree = false;
%    return
%end

%% propagate to assess trajectory feasibility

for i=1:npropagate

    %obstacle path (constant/declining velocity model)
    v_obstacles = v_obstacles*d.par.obstacle_vel_gain;
    p_obstacles = p_obstacles+v_obstacles*d.par.Ts_predict; %Aneesh: could make Ts_predict a number that changes with obstacle velocity?
    p_obstacles_traj(:,:,i) = p_obstacles;

    %check collision --> WAAN: only when there are obstacles (min in C does not work on empty array)
    if isempty(p_obstacles)
        violation = update_violation(violation, 'obstacle', 0);
    else
        %minimum obstacle distance
        p_diff = bsxfun(@minus,p_obstacles,p_robot(i,1:2));
        p_diff_norm = sqrt(sum(p_diff.^2,2));

        % if we're intercepting and we're closer to the target than the
        % opponent, then I don't mind a collision because I will win the
        % referees decision
        p_diff_obst2Target = min(vecnorm(abs(bsxfun(@minus,p_obstacles,d.target.p(1:2))),2,2));
        p_diff_robot2Target = bsxfun(@minus,p_robot(i,1:2),d.target.p(1:2));
        target2ball_distance = norm(d.target.p(1:2) - d.input.ball.p(1:2));
        margin = d.input.robot.dist2ball_vs_opp;
        ball2obstacle = bsxfun(@minus,p_obstacles,d.input.ball.p(1:2));
        robot2ball = p_robot(i,1:2) - d.input.ball.p(1:2);
        
        angle = zeros(1,size(ball2obstacle,1));
        for j=1:size(ball2obstacle,1)
            norm_arr = norm(cross([ball2obstacle(j,1:2) 0], [robot2ball 0]));
            dot_arr = dot(ball2obstacle(j,1:2), robot2ball);
            angle(j) = abs(rad2deg(atan2(norm_arr, dot_arr)));
        end
        no_obstacle_between_me_and_ball = all(angle >= 45);
                        
        if ((d.input.robot.skillID == 5) || (d.input.robot.skillID == 0))...
                && (p_diff_obst2Target + margin > norm(p_diff_robot2Target(1:2)))...
                && (target2ball_distance < 0.5)...
                && no_obstacle_between_me_and_ball
            violation_value = 0;
        elseif (d.input.robot.skillID == 5) && (p_diff_obst2Target > norm(p_diff_robot2Target(1:2)))
            violation_value = 0;
        else
            obs_violation = max(0, (collision_distance+obstacle_margin*norm(v_robot(i,1:2)))-p_diff_norm);
            violation_value = max(obs_violation);
        end

        %scaled projection point
        violation = update_violation(violation, 'obstacle', violation_value);
    end

    % check illegal driving zone
    if d.input.SubtargetAvoidPolygon.valid
        if inpolygon(p_robot(i,1), p_robot(i,2), d.input.SubtargetAvoidPolygon.polygon(1,:), d.input.SubtargetAvoidPolygon.polygon(2,:)) && ...
            ~inpolygon(d.target.p(1), d.target.p(2), d.input.SubtargetAvoidPolygon.polygon(1,:), d.input.SubtargetAvoidPolygon.polygon(2,:)) && ...
            ~inpolygon(p_robot(1,1), p_robot(1,2), d.input.SubtargetAvoidPolygon.polygon(1,:), d.input.SubtargetAvoidPolygon.polygon(2,:))
            violation = update_violation(violation, 'SubtargetAvoidPolygon', 1);
        else
            violation = update_violation(violation, 'SubtargetAvoidPolygon', 0);
        end
    else
        violation = update_violation(violation, 'SubtargetAvoidPolygon', 0);
    end

    % check field if possess ball
    if norm(subtarget.p(1:2)-d.target.p(1:2))>1e-3
        if any(d.input.robot.skillID==1:4) %ball possession
            robot_ball_distance = d.par.robot_radius+d.par.ball_radius; %[m]
            ball_pos = p_robot(i,1:2)+robot_ball_distance*[-sin(p_robot(i,3)) cos(p_robot(i,3))];
            violation_value = max([0 abs(ball_pos)-d.par.field_size*.5]);
            violation = update_violation(violation, 'field', violation_value);
        else
            violation = update_violation(violation, 'field', 0);
        end
        if -(d.par.field_size(1)*.5 + d.par.field_border_margin) < subtarget.p(1) && subtarget.p(1) < (d.par.field_size(1)*.5 + d.par.field_border_margin) ... % Clip subtarget to inside field
            && -(d.par.field_size(2)*.5 + d.par.field_border_margin) < subtarget.p(2) && subtarget.p(2) < (d.par.field_size(2)*.5 + d.par.field_border_margin) % Clip subtarget to inside field

            % clip to 3m radius if dribbling
            max_radius = 3.0;
            if d.input.robot.human_dribble_flag == 1
                distance_between_cpb_poi_xy_and_subtarget = 0.0;
            else
                distance_between_cpb_poi_xy_and_subtarget = norm(subtarget.p(1:2) - d.input.robot.cpb_poi_xy(1:2));
            end

            % don't go inside goal area
            xpos = .5*d.par.field_goal_area(1);
            ypos = d.par.field_size(2)*.5-d.par.field_goal_area(2);
            subtarget_is_in_goal_area = abs(subtarget.p(1))<xpos & abs(subtarget.p(2))>ypos;

            if (any(d.input.robot.skillID==1:4) && (distance_between_cpb_poi_xy_and_subtarget > max_radius)) || (subtarget_is_in_goal_area)
                violation_value = 1e11;
            else
                violation_value = 0;
            end

        else
            if (d.subtarget.automatic_substitution_flag==1) && -(d.par.field_size(1)*.5 + d.par.field_border_margin + d.par.technical_area_width) < subtarget.p(1) && subtarget.p(1) < (d.par.field_size(1)*.5 + d.par.field_border_margin + d.par.technical_area_width) ... % Clip subtarget to inside field + technical area margin
            && -(d.par.field_size(2)*.5 + d.par.field_border_margin) < subtarget.p(2) && subtarget.p(2) < (d.par.field_size(2)*.5 + d.par.field_border_margin) % Clip subtarget to inside field + technical area margin
                violation_value = 0;
            else
                violation_value = 1e11;
            end
        end
        violation = update_violation(violation, 'field', violation_value);
    end

    %check penalty area
%     violation_value = spg.subtarget.replan.get_distance_inside_penalty_area(d, p_robot(i,:));
%     violation = update_violation(violation, 'penalty_area', violation_value);

    %check if close to arrival
%     if all(d.traj.segment_id(i,1:2)==3)
%         break
%     end
    if norm(p_robot(i,1:2)-subtarget.p(1:2))<1e-2
        break
    end
end

% clip subtarget x-velocity such that robot stays in field (can brake in time)
dist2sideline = (d.par.field_size(1)*.5 + d.par.field_border_margin) - abs(subtarget.p(1));
if abs(2*d.par.dmax_move*dist2sideline) < subtarget.v(1)^2
    subtarget.v(1) = 2*d.par.dmax_move*dist2sideline;
end
% clip subtarget y-velocity such that robot stays in field (can brake in time)
dist2goalline = (d.par.field_size(2)*.5 + d.par.field_border_margin) - abs(subtarget.p(2));
if abs(2*d.par.dmax_move*dist2goalline) < subtarget.v(2)^2
    subtarget.v(2) = 2*d.par.dmax_move*dist2goalline;
end

subtarget.collisionfree = violation.collisionfree;
subtarget.violation_count = violation.count;

global traj_trials
if ~isempty(traj_trials)
    traj_trials.item = [traj_trials.item; struct('subtarget', subtarget, 'p_robot', p_robot)];
end

function violation = update_violation(violation, field, violation_value)
if violation_value>0
    violation.count = violation.count+1;
end
if violation_value > violation.(field)
    violation.collisionfree = false;
else
    violation.(field) = violation_value;
end