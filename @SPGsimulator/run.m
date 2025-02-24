function run(s)

%preparations
d = s.d;
i = s.sim.i;
nsim = length(s.sim.t);
log = s.log;
s.h.run_stop.BackgroundColor = [0 1 0];
d.setpoint.v = d.input.robot.v;

%iterate
while ~isequal(s.h.action, "stop")
    if isequal(s.h.action, "restart")
        d = s.d;
        i = 1;
        s.h.action = "run";
    end
    
    if isequal(s.h.action, "pause")
        pause(.1)
        continue
    end
        
    %simulation data
    d = getSampleData(s, d, i); %set d.input.robot, d.input.obstacles, d.par.*
    
    %perfect robot tracking
    d.input.robot.p = d.setpoint.p;
    d.input.robot.v = d.setpoint.v;
    
    %save robot path and velocity
    s.sim.error.path(i,:) = d.input.robot.p(1:2);
    s.sim.error.vel(i,:) = d.input.robot.v(1:2);
    
    %step 1 sample
    d = step(s, d);
    
    %update visualisation
    update(s, d, i);
    
    %use breakpoint here to stop at newly planned subtargets
    if d.subtarget.action==2
    end
    
    %save position data
    log(i,:) = d.setpoint.p;
    
    if i==1
        % save collision scenario for debugging SPG code
        s.sim.error.obstP = d.input.obstacles.p;
        s.sim.error.obstV = d.input.obstacles.v;
    end
    
    %if collision, fail the test
    p_obstacles = d.input.obstacles.p(d.input.obstacles.active,:);
    r_obstacles = d.input.obstacles.r(d.input.obstacles.active);
    collision_distance = d.par.robot_radius+r_obstacles;
    if ~isempty(p_obstacles)
        smallest_robot2obstacle_dist = min(abs( vecnorm(d.input.robot.p(1:2) - p_obstacles(:,1:2),2,2) ) - collision_distance);
        s.sim.error.smallest_robot2obstacle_dist(i) = smallest_robot2obstacle_dist;
    %     if smallest_robot2obstacle_dist < 0
    %         error('collision occured');
    %     end
    end
    
    i = mod(i,nsim)+1;
    
    %robot stationary at target
    if norm(d.setpoint.p(1,1:2)-d.target.p(1:2))<.001 && norm(d.input.robot.v(1,1:2))<.001
        s.h.action = "stop";
    end
    
    if s.h.action == "step"
        s.h.action = "pause";
    end
    
end

%save state
s.log = log;
s.sim.i = i;
s.d = d;
s.h.run_stop.BackgroundColor = [.9400    0.9400    0.9400];

% function run(s)
% 
% %preparations
% d = s.d;
% i = s.sim.i;
% nsim = length(s.sim.t);
% log = s.log;
% s.h.run_stop.BackgroundColor = [0 1 0];
% d.setpoint.v = d.input.robot.v;
% 
% %iterate
% while ~isequal(s.h.action, "stop")
%     if isequal(s.h.action, "restart")
%         d = s.d;
%         i = 1;
%         s.h.action = "run";
%     end
%     
%     if isequal(s.h.action, "pause")
%         pause(.1)
%         continue
%     end
%         
%     %simulation data
%     d = getSampleData(s, d, i); %set d.input.robot, d.input.obstacles, d.par.*
%     
%     %perfect robot tracking
%     d.input.robot.p = d.setpoint.p;
%     d.input.robot.v = d.setpoint.v;
%     
%     %step 1 sample
%     d = step(s, d);
%     
%     %update visualisation
%     update(s, d, i);
%     
%     %use breakpoint here to stop at newly planned subtargets
%     if d.subtarget.action==2
%     end
%     
%     %save position data
%     log(i,:) = d.setpoint.p;
%     
%     i = mod(i,nsim)+1;
%     
%     if norm(d.setpoint.p(1,1:2)-d.target.p(1:2))<.001
%         s.h.action = "stop";
%     end
%     
%     %if collision, fail the test
%     p_obstacles = d.input.obstacles.p(d.input.obstacles.active,:);
%     r_obstacles = d.input.obstacles.r(d.input.obstacles.active);
%     collision_distance = d.par.robot_radius+r_obstacles;
%     smallest_robot2obstacle_dist = min(abs( vecnorm(d.input.robot.p(1:2) - p_obstacles(:,1:2),2,2) ) - collision_distance);
%     if smallest_robot2obstacle_dist < 0
%         error('collision occured');
%     end
%     
%     if s.h.action == "step"
%         s.h.action = "pause";
%     end
%     
% end
% 
% %save state
% s.log = log;
% s.sim.i = i;
% s.d = d;
% s.h.run_stop.BackgroundColor = [.9400    0.9400    0.9400];