function setInputData(s, t, pose, vel, target, skillID, CPPA, obstacles_pos, obstacles_vel, obstacles_active, target_vel)

n = length(t);
s.sim.t = t;
s.sim.robot_pose = to_vec(t, pose);
s.sim.robot_vel = to_vec(t, vel);
s.sim.robot_target = to_vec(t, target);
s.sim.skillID = skillID*ones(n/numel(skillID),1);
s.sim.CPPA = CPPA*ones(n/numel(CPPA),1);
s.sim.obstacle_pos = to_vec(t, obstacles_pos);
s.sim.obstacle_vel = to_vec(t, obstacles_vel);
s.sim.obstacle_active = to_vec(t, obstacles_active~=0);
s.sim.i = 1;
s.sim.robot_target_vel = to_vec(t, target_vel);
s.sim.error.smallest_robot2obstacle_dist = 0;
s.sim.error.obstP = to_vec(t, obstacles_pos);
s.sim.error.obstV = to_vec(t, obstacles_vel);
s.sim.error.path = zeros(1,2);
s.sim.error.vel = zeros(1,2);

function vec = to_vec(t,x)
vec.time = t;
vec.signals.values = repmat(x,[1 1 length(t)/size(x,3)]);
vec.signals.dimensions = size(x);

