function best = beside_obstacle(d, best)

subtarget_candidate = best;
additional_obstacle_margin = .2; %[m] - Aneesh: can we make this number dependent on obstacle state? (if acceleration smaller -> lower margin)

% Aneesh - THIS FUNCTION SHOULD TAKE VELOCITY INTO ACCOUNT?

for i = 1:length(d.input.obstacles.active)
    if d.input.obstacles.active(i)
        pos = d.input.obstacles.p(i,:);
        obstacle_radius = d.input.obstacles.r(i);
        
        v1 = d.target.p(1:2)-d.setpoint.p(1:2);
        v2 = pos-d.setpoint.p(1:2);
        v3 = v1*v2'/sum(v1.^2)*v1;
        v4 = v2-v3;
        
        for speed = linspace(0,d.par.vmax_move*.6,4) % Aneesh - we can improve this search, but not sure if _here_ is the correct place to do so
            displacement = v4/norm(v4)*(d.par.robot_radius+obstacle_radius+.1+d.par.margin_replan*norm(speed));
            
            obstacle_margin = d.par.robot_radius+obstacle_radius+additional_obstacle_margin;
            for side = [-1 1]
                subtarget_candidate.p = [pos+side*displacement atan2(-v1(1),v1(2))];
                if spg.subtarget.replan.not_touching_obstacle(subtarget_candidate.p(1:2), d.input.obstacles.p(d.input.obstacles.active,1:2), obstacle_margin)
                    
                    subtarget_candidate.v = [v1/norm(v1)*speed 0];
                    
                    subtarget_candidate = spg.subtarget.replan.determine_setpoint_limits(d, subtarget_candidate);
                    subtarget_candidate = spg.subtarget.check_collisionfree(d, subtarget_candidate, d.par.margin_replan);
                    
                    best = spg.subtarget.replan.update_best(best, subtarget_candidate, d.target);
                end
            end
        end
    end
end
