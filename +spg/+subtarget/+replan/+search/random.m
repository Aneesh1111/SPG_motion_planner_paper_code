function best = random(d, best, search_point, search_distance, defending_opp_with_or_without_ball)

subtarget_candidate = best;
vmax_random = d.par.vmax_move*.5; %prepare nonzero velocity

for i=1:d.par.nattempts_replan
    
    lowerbound = max(-d.par.field_size*.5,search_point(1:2)-search_distance);
    upperbound = min(d.par.field_size*.5,search_point(1:2)+search_distance);
    
    nattempts = 10;
    obstacle_margin = d.par.robot_radius+d.input.obstacles.r+.05;
    
    for j = 1:nattempts
        
        
        % Aneesh - make search within area here (for better subtarget
        % searching)
        
        if defending_opp_with_or_without_ball == 1  % 1 means we are defending the opponent without the ball so we should search THAT region
            
            % Aneesh - improve the way we sample!
            upperbound = min(upperbound, search_point(1:2)+search_distance*0.5);
            %new candidate
            p_candidate = [(upperbound-lowerbound).*rand(1,2)+lowerbound search_point(3)];
            opp_radius = 0.5;
            polygon_subtarget_should_be_inside_x = [-d.par.goalwidth*0.5,...
                                                    d.input.ball.p(1)-opp_radius,...
                                                    d.input.ball.p(1),...
                                                    d.input.ball.p(1)+opp_radius,...
                                                    d.par.goalwidth*0.5,...
                                                    -d.par.goalwidth*0.5];
            polygon_subtarget_should_be_inside_y = [-d.par.field_size(2)*0.5,...
                                                    d.input.ball.p(2),...
                                                    d.input.ball.p(2),...
                                                    d.input.ball.p(2),...
                                                    -d.par.field_size(2)*0.5,...
                                                    -d.par.field_size(2)*0.5];
            in = inpolygon(p_candidate(1),p_candidate(2),polygon_subtarget_should_be_inside_x,polygon_subtarget_should_be_inside_y);
            
            if in == 0
                continue
            end

            %aux
            subtarget_target_distance = norm(p_candidate(1:2)-d.target.p(1:2));
            subtarget_robot_distance = norm(p_candidate(1:2)-d.setpoint.p(1:2));
            target_robot_distance = norm(d.target.p(1:2)-d.setpoint.p(1:2));
            vnorm = norm(d.setpoint.v(1:2));
            
            
        else  % we are not defending the opponent so we should search the region we specify
            %new candidate
            p_candidate = [(upperbound-lowerbound).*rand(1,2)+lowerbound search_point(3)];
%             if norm(search_point(1:2)-p_candidate(1:2))>search_distance
%                 continue
%             end
            %aux
            subtarget_target_distance = norm(p_candidate(1:2)-d.target.p(1:2));
            subtarget_robot_distance = norm(p_candidate(1:2)-d.setpoint.p(1:2));
            target_robot_distance = norm(d.target.p(1:2)-d.setpoint.p(1:2));
            vnorm = norm(d.setpoint.v(1:2));
        end
        
        %should be closer to target
        if subtarget_target_distance<target_robot_distance+d.par.replan_uphill_distance
            
            %no obstacle collisions
            if spg.subtarget.replan.not_touching_obstacle(p_candidate(1:2), d.input.obstacles.p(d.input.obstacles.active,1:2), obstacle_margin(d.input.obstacles.active))
                subtarget_candidate.p = p_candidate;
%                 subtarget_candidate.v = [(rand(1,2)*2-1)*vmax_random 0];
                max_accel_x = sqrt(abs(d.input.robot.v(1)) + 2*(d.par.amax_move/sqrt(2))*abs(subtarget_candidate.p(1)-d.input.robot.p(1)));
                max_accel_y = sqrt(abs(d.input.robot.v(2)) + 2*(d.par.amax_move/sqrt(2))*abs(subtarget_candidate.p(2)-d.input.robot.p(2)));
                max_decel_x = sqrt(abs(d.target.v(1)) + 2*(d.par.dmax_move/sqrt(2))*abs(d.target.p(1)-subtarget_candidate.p(1)));
                max_decel_y = sqrt(abs(d.target.v(2)) + 2*(d.par.dmax_move/sqrt(2))*abs(d.target.p(2)-subtarget_candidate.p(2)));
                vx_max = min(d.par.vmax_move*0.707, min(max_accel_x, max_decel_x));
                vy_max = min(d.par.vmax_move*0.707, min(max_accel_y, max_decel_y));
                v_subtarget_x = sign(d.target.v(1)-d.input.robot.v(1))*min(abs(d.target.v(1)-d.input.robot.v(1)), vx_max);
                v_subtarget_y = sign(d.target.v(2)-d.input.robot.v(2))*min(abs(d.target.v(2)-d.input.robot.v(2)), vy_max);
                subtarget_candidate.v = [v_subtarget_x, v_subtarget_y, 0];
                input1 = d;
                input2 = subtarget_candidate;
                subtarget_candidate = spg.subtarget.replan.determine_setpoint_limits(d, subtarget_candidate);
                output1 = subtarget_candidate;
                subtarget_candidate = spg.subtarget.check_collisionfree(d, subtarget_candidate, d.par.margin_replan);

                d.subtarget_array((i-1)*10 + j) = subtarget_candidate;
                d.traj_array((i-1)*10 + j) = d.traj;

                best = spg.subtarget.replan.update_best(best, subtarget_candidate, d.target);
%                 return
            end
        end
    end
    
end

%requirements candidate point
% - better norm (closer to target)
% - in field
% - outside goal area
% - not on top of obstacle
