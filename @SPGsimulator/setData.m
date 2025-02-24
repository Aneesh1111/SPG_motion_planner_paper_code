function d = setData(s)

seed = rng;
rng(s.data_seed)

%time
Ts = s.d0.par.Ts;
nsim = 2e3;
s_sim.Ts = Ts;
s_sim.t = (0:nsim-1)'*Ts;
s_sim.i = 1;

%% fixed parameters for compilation

s_sim.spg_init = struct('nobstacles',s.d0.par.nobstacles,'npredict',s.d0.par.npredict);

%% obstacles

obstacle_pos = [...
    (rand(s.d0.par.nobstacles,1)-.5)*s.d0.par.field_size(1) ...
    (rand(s.d0.par.nobstacles,1)-.5)*s.d0.par.field_size(2)];
s_sim.obstacle_pos = to_vec(s_sim.t, obstacle_pos);

timeChange = 3; %s
nsteps = round(s_sim.t(end)/timeChange);
vmax = .6*s.d0.par.vmax_move;
amax = .6*s.d0.par.amax_move;
vtarget = vmax*rand(nsteps,s.d0.par.nobstacles);
if s.d0.par.nobstacles < 2
    vel_a = 0;
else
    vel_a = interp1(linspace(s_sim.t(1),s_sim.t(end),nsteps),vtarget,s_sim.t,'cubic');
end
if vtarget == 0
    wmax = 0;
else
    wmax = amax./vtarget;
end
if s.d0.par.nobstacles < 1
    dphi = 0;
else
    dphi = interp1(linspace(s_sim.t(1),s_sim.t(end),nsteps),(2*rand(nsteps,s.d0.par.nobstacles)-1).*wmax,s_sim.t,'cubic');
end
phi = cumsum(dphi)*Ts + 2*pi*randn(1,s.d0.par.nobstacles);

vel_x = cos(phi).*vel_a;
vel_y = sin(phi).*vel_a;
pos_x = cumsum(vel_x)*Ts;
pos_y = cumsum(vel_y)*Ts;
s_sim.obstacle_pos.signals.values = s_sim.obstacle_pos.signals.values+permute(cat(3,pos_x,pos_y),[2 3 1]);

s_sim.obstacle_vel = s_sim.obstacle_pos;
v = diff(permute(s_sim.obstacle_pos.signals.values,[3 1 2]))/Ts;
v = cat(1,zeros(1,size(v,2),size(v,3)),v);
n = 1/Ts/2;
for i=1:size(v,3)
    v(:,:,i) = conv2(v(:,:,i),ones(n,1)/n,'same');
end
s_sim.obstacle_vel.signals.values = permute(v,[2 3 1]);
obstacle_active = false(s.d0.par.nobstacles,1);
obstacle_active(1:round(.9*end)) = true;
s_sim.obstacle_active = to_vec(s_sim.t, obstacle_active);

%% robot
robot_target = s.d.target.p;
s_sim.robot_target = to_vec(s_sim.t, robot_target);
robot_target_vel = s.d.target.v;
s_sim.robot_target_vel = to_vec(s_sim.t, robot_target_vel);

robot_pose = s.d0.input.robot.p;
s_sim.robot_pose = to_vec(s_sim.t, robot_pose);
robot_vel = s.d0.input.robot.v;
s_sim.robot_vel = to_vec(s_sim.t, robot_vel);

s_sim.skillID = 0*ones(nsim,1);

s_sim.CPPA = ones(nsim,1);

vmax = [s.d0.par.vmax_move s.d0.par.vmax_rotate];
s_sim.vmax = to_vec(s_sim.t, vmax);

amax = [s.d0.par.amax_move s.d0.par.amax_rotate];
s_sim.amax = to_vec(s_sim.t, amax);

rng(seed)

s.sim = s_sim;
d = s.d0;

function vec = to_vec(t,x)
vec.time = t;
vec.signals.values = repmat(x,[1 1 length(t)]);
vec.signals.dimensions = size(x);
