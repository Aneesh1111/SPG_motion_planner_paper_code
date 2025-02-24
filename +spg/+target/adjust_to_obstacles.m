function target = adjust_to_obstacles(d, target)

%prep
extra_margin = .05; %[m] move a bit further beyond the collision_distance
nAttempts = 10; %[#] maximum iteration count to step away from nearby obstacles
obstacles_p = d.input.obstacles.p(d.input.obstacles.active,:);
r_obstacles = d.input.obstacles.r(d.input.obstacles.active);
collision_distance = d.par.robot_radius+r_obstacles;

%shift target
if ~isempty(obstacles_p)
    for iAttempt = 1:nAttempts
        
        %get distance of target to obstacles
        v_target_obstacles = bsxfun(@minus, target.p(1:2), obstacles_p);
        
        %nearest obstacle
        v_target_obstacles_norm2 = sum(v_target_obstacles.^2,2);
        [norm2min,ind] = min(v_target_obstacles_norm2-collision_distance.^2);
        
        %project target away from nearest obstacle
        if norm2min<0
            vec = v_target_obstacles(ind,1:2);
            target.p = [obstacles_p(ind,1:2)+vec/norm(vec).*(collision_distance(ind)+extra_margin) target.p(3)];
        else
            break
        end
    end
end
