function [d, robot, obstacles] = getSampleData(s, d, i)

d.input.robot.p = s.sim.robot_pose.signals.values(:,:,i);
d.input.robot.v = s.sim.robot_vel.signals.values(:,:,i);

d.input.robot.target = s.sim.robot_target.signals.values(:,:,i);

if norm(d.input.ball.v) ~= 0
    d.input.robot.target(1:2) = d.input.robot.target(1:2) + d.input.ball.v .* d.par.Ts .* (i - 1);
end

d.input.robot.target_vel = s.sim.robot_target_vel.signals.values(:,:,i);
d.input.robot.skillID = s.sim.skillID(i);
d.input.robot.CPPA = s.sim.CPPA(i);
robot = d.input.robot;

d.input.obstacles.p =  s.sim.obstacle_pos.signals.values(:,:,i);
d.input.obstacles.v =  s.sim.obstacle_vel.signals.values(:,:,i);
d.input.obstacles.r = .25*ones(size(d.input.obstacles.p,1),1);
d.input.obstacles.active =  s.sim.obstacle_active.signals.values(:,:,i);
obstacles = d.input.obstacles;

if isfield(s.sim,'vmax')
    d.par.vmax_move = s.sim.vmax.signals.values(1,1,i);
    d.par.vmax_rotate = s.sim.vmax.signals.values(1,2,i);
end
if isfield(s.sim,'amax')
    d.par.amax_move = s.sim.amax.signals.values(1,1,i);
    d.par.amax_rotate = s.sim.amax.signals.values(1,2,i);
end
