format long

set(0,'DefaultLineLineWidth',2) % to get thicker lines in the plots
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')

figure;
ax = subplot(2,1,1)
hold on
plot(s.time, s.position(:,1))
plot(s.time, s.position(:,2))
hold off
grid on
box on
set(gca,'TickLabelInterpreter','latex', 'XTickLabel',[]);
ylabel('Position [$m$]', 'Interpreter', 'latex')
legend("x-direction", "y-direction", 'Location', 'northwest', 'Interpreter', 'latex')
% xlim([0 6])
ylim([-6.5 3.5])

h3 = subplot(2,1,2)
hold on
plot(s.time, s.velocity(:,1))
plot(s.time, s.velocity(:,2))
hold off
grid on
box on
set(gca,'TickLabelInterpreter','latex');
xlabel('Time [$s$]', 'Interpreter', 'latex')
ylabel('Velocity [$\frac{m}{s}$]', 'Interpreter', 'latex')
legend("x-direction", "y-direction", 'Location', 'northwest', 'Interpreter', 'latex')
% xlim([0 6])
set(h3, 'Units', 'normalized');
set(h3, 'Position', [ax.Position(1), ax.Position(2)-0.4, ax.Position(3), ax.Position(4)]);


figure(2)
plot(p_initial(1), p_initial(2), 'x')
hold on
plot(s.d.target.p(1), s.d.target.p(2), 'x')
plot(s.position(1:end-1,1)', s.position(1:end-1,2)', "-o")
hold off
ylim([-6,6])
xlim([-4,4])
title('path')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SPG simulations for paper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_tests = 35;
test_scenario = "very_dynamic2";
tests_passed = 0;
tests_failed = 0;
n_collisions = 0;
n_time_constraints = 0;
index_of_failed_tests = zeros(1, n_tests);
tests_duration = zeros(1, n_tests);
min_obs_dist = zeros(1, n_tests);
path_length = zeros(1, n_tests);
path_vel = zeros(1, n_tests);
path_vel_std = zeros(1, n_tests);

range_of_velG1 = [-0.1.*ones(1,20), -1.*ones(1,20), -2.*ones(1,30), -3.*ones(1,30)];
range_of_velG2 = [0.1.*ones(1,2), 1.*ones(1,2), 2.*ones(1,3), 3.*ones(1,3)];
range_of_accG1 = [-1.5.*ones(1,10), -2.*ones(1,10), -0.5.*ones(1,10), -1.*ones(1,10), 1.*ones(1,10), 0.5.*ones(1,10), -0.5.*ones(1,10), 2.*ones(1,10), 1.25.*ones(1,10), 0.5.*ones(1,10)];
range_of_accG2 = [1.5, 2, 0.5, 1, -1, -0.5, 0.5, -2, -1.25, -0.5];

% range_of_velG1 = zeros(1,100);
% for jj = 1:100
%     range_of_velG1(jj) = -0.04*jj;
% end

error_casesV = zeros(5,1);
error_casesA = zeros(5,1);

for i=n_tests:n_tests
    
    group1V = [range_of_velG1(i)].*ones(3,1);
    group2V = [range_of_velG2(mod(i,10)+1)].*ones(2,1);
    group1A = [range_of_accG1(i)].*ones(3,1);
    group2A = [range_of_accG2(mod(i,10)+1)].*ones(2,1);
    
    group1V = [range_of_velG1(i)].*ones(3,1);
    group2V = [range_of_velG2(mod(i,10)+1)].*ones(2,1);
    group1A = [range_of_accG1(i)].*ones(3,1);
    group2A = [range_of_accG2(mod(i,10)+1)].*ones(2,1);

    
    obst_sim_velocities = [group1V;group2V];
%     obst_sim_velocities = [group1V];
    obst_sim_acc = [group1A;group2A];
    save("obst_sim_velocities.mat", "obst_sim_velocities")
    save("obst_sim_acc.mat", "obst_sim_acc")
    
    switch test_scenario
        case "static"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','static');
        case "static_jitter"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','static_jitter');
        case "dynamic1"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','dynamic1');
        case "dynamic2"
%             test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','dynamic2_old');
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','dynamic2');
        case "dynamic1_jitter"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','dynamic1_jitter');
        case "dynamic2_jitter"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','dynamic2_jitter');
        case "very_dynamic1"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','very_dynamic1');
        case "very_dynamic_jitter1"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','very_dynamic1_jitter');
        case "very_dynamic2"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','very_dynamic2');
%             test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','very_dynamic2_old');
        case "very_dynamic_jitter2"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','very_dynamic2_jitter');
        case "local_minima"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','local_minima');
        case "local_minima_jitter"
            test_results = runtests('unittest.test_SPG_paper_simulations','ProcedureName','local_minima_jitter');
        otherwise
            disp("this test scenario does not exist");
    end
    
    min_obs_dist(i) = min(collision_dist);
    tests_duration(i) = test_results.Duration;
    path_length(i) = sum(vecnorm(diff(path),2,2));
    path_vel(i) = mean(vecnorm(vel,2,2));
    path_vel_std(i) = std(vecnorm(vel,2,2));
    % if collision or test takes longer than 20 seconds, then test failed
    if any(collision_dist<0) || test_results.Duration>20
        tests_failed = tests_failed + 1;
        index_of_failed_tests(i) = 1;
        % save state of obstacles in event of a collision (for debugging)
        obst_collision_pos(:,:,tests_failed) = obstacle_pos(1:5,:);
        obst_collision_vel(:,:,tests_failed) = obstacle_vel(1:5,:);
        
%         error_casesV = [error_casesV, obst_sim_velocities];
%         error_casesA = [error_casesA, obst_sim_acc];
        
        if any(collision_dist<0)
            n_collisions = n_collisions + 1;
            collision_impact_distance(n_collisions) = min(collision_dist);
        end
        if test_results.Duration>20
            n_time_constraints = n_time_constraints + 1;
            over_time_limit(n_time_constraints) = test_results.Duration;
        end
    else
        tests_passed = tests_passed + 1;
    end
end

exlude_idx = find(index_of_failed_tests == 1);
remove_failed_tests = true(size(index_of_failed_tests));
remove_failed_tests(exlude_idx) = false;
tests_duration_passed = tests_duration(remove_failed_tests);
max_duration = max(tests_duration_passed);
min_duration = min(tests_duration_passed);
avg_duration = mean(tests_duration_passed);
std_duration = std(tests_duration_passed);

results.avg_duration = avg_duration;
results.std_duration = std_duration;
results.tests_failed = tests_failed;
results.tests_passed = tests_passed;
if tests_failed>0
    results.obst_collision_pos = obst_collision_pos;
    results.obst_collision_vel = obst_collision_vel;
end
results.min_obs_dist_mean = mean(min_obs_dist);
results.min_obs_dist_std = std(min_obs_dist);
results.avg_path_len = mean(path_length);
results.std_path_len = std(path_length);
results.avg_path_vel = mean(path_vel);
results.std_path_vel = mean(path_vel_std.^2) + mean((path_vel - results.avg_path_vel).^2);
results.avg_coll = mean(collision_impact_distance);
results.std_coll = std(collision_impact_distance);
% results.error_cases_v = error_casesV;
% results.error_cases_a = error_casesA;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% safe plots

figure(1)
plot(s.time, s.position(:,1))
hold on
plot(s.time, s.position(:,2))
hold off
grid on
set(gca,'TickLabelInterpreter','latex');
xlabel('Time [$s$]', 'Interpreter', 'latex')
ylabel('Position [$m$]', 'Interpreter', 'latex')
legend("x-direction", "y-direction", 'Location', 'northwest', 'Interpreter', 'latex')
xlim([0 6])
ylim([-6.5 3.5])

figure(2)
plot(s.time, s.velocity(:,1))
hold on
plot(s.time, s.velocity(:,2))
hold off
grid on
set(gca,'TickLabelInterpreter','latex');
xlabel('Time [$s$]', 'Interpreter', 'latex')
ylabel('Velocity [$\frac{m}{s}$]', 'Interpreter', 'latex')
legend("x-direction", "y-direction", 'Location', 'northwest', 'Interpreter', 'latex')
xlim([0 6])



%% own 1D SPG
p0 = [0 0 0];
v0 = [2 0 0];
pe = [1 10 0];
ve = [0 0 0];
vm = [1 1 0].*4;
am = [1 1 0].*5;
dm = [1 1 0].*3;

d.aux = struct('segment',repmat(struct('dt',zeros(1,3),'t',zeros(1,3),'p',zeros(1,3),'v',zeros(1,3),'a',zeros(1,3)),1,4));

npredict = 20;
segment = get_segments(d.aux.segment, p0, v0, pe, ve, vm, am, dm);
traj = struct('p',zeros(npredict,3),'v',zeros(npredict,3),'a',zeros(npredict,3),'t',zeros(npredict,1),'segment_id',zeros(npredict,3)); %defines prediction horizon
d.par = struct('Ts',.02,...
    'vmax_move',norm(vm),... %[m/s]   
    'vmax_rotate',13,... %[rad/s]
    'amax_move',norm(am),... %[m/s2]
    'amax_rotate',13,... %[rad/s2]
    'scale_rotate',.3,... %disabled for now
    'scale_angle',40/180*pi,... %[rad]
    'dmax_move',norm(dm),... %[m/s2]
    'dmax_rotate',13);

dt = d.par.Ts;
tmax = 10;
t = (0:dt:tmax)';
v = zeros(length(t)+1, 3);
p = zeros(length(t)+1, 3);
p(1,:) = p0;
v(1,:) = v0;
t_debug = zeros(npredict, length(t));
a = zeros(length(t)+1, 3);

time = [0 segment(1).t(1) segment(2).t(1) segment(3).t(1)];
velocity = [v0(1) segment(1).v(1) segment(2).v(1) segment(3).v(1)];
% figure(6)
% plot(time, velocity)

for i=1:1:length(t)

    [segment, vm, am, dm] = determine_setpoint_limits(d, p0, v0, pe, ve, segment);
    % calculate segments every timestep    
    segment = get_segments(segment, p(i,:), v(i,:), pe, ve, vm, am, dm);
    %propagate 1 sample
    traj = traj1(traj, segment, d.par.Ts); %propagate only 1 sample
    p(i+1,:) = traj.p(1,:);
    v(i+1,:) = traj.v(1,:);
    a(i+1,:) = traj.a(1,:);
    t_debug(:,i) = traj.t;
    
 end

figure(1)
plot(t, p(1:end-1,1), 'r')
hold on
plot(t, p(1:end-1,2), 'b')
hold off
title('position')

figure(2)
plot(t, v(1:end-1,1), 'r')
hold on
plot(t, v(1:end-1,2), 'b')
hold off
title('velocity')

figure(3)
plot(t, a(1:end-1,1), 'r')
hold on
plot(t, a(1:end-1,2), 'b')
hold off
title('acceleration')

figure(4)
plot(p0(1), p0(2), 'x')
hold on
plot(pe(1), pe(2), 'x')
plot(p(1:end-1,1)', p(1:end-1,2)')
hold off
title('path')


function segment = get_segments(segment, p0, v0, pe, ve, vm, am, dm)

% %correct for rotation
% pe(3) = spg.setpoint.wrap(pe(3), p0(3));

%determine maximum speed
[v1, tmax] = get_max_speed(segment(1), p0, v0, pe, ve, vm, am, dm);

%move towards maximum speed
segment(1) = move_to_vel(segment(1), p0, v0, 0, v1, am, dm);

%move at max constant speed
segment(2) = move_at_constant_vel(segment(2), segment(1).p, segment(1).v, segment(1).t, tmax);

%move towards end speed/position
segment(3) = move_to_vel(segment(3), segment(2).p, segment(2).v, segment(2).t, ve, am, dm);

%move at max constant speed
segment(4) = move_at_constant_vel(segment(4), segment(3).p, segment(3).v, segment(3).t, [1e10 1e10 1e10]);

% %move to standstill
% segment(4) = move_to_vel(segment(4), segment(3).p, segment(3).v, segment(3).t, [0 0 0], am);
% 
% %standstill
% segment(5) = segment(4);
% segment(5).a = [0 0 0];
% segment(5).dt = [1e10 1e10 1e10];
% segment(5).t = segment(4).t+segment(5).dt;
end


function [v1, tmax] = get_max_speed(segment, p0, v0, pe, ve, vm, am, dm)

%accelerate to end velocity to determine correct direction
segment = move_to_vel(segment, p0, v0, 0, ve, am, dm); % max(abs(am),abs(dm))
direction = functions.sign(pe-segment.p);

%determine maximum speed needed when not using velocity constraint
a = direction.*am;
d = direction.*dm;

%speed to swap acceleration
acc_ratio = a./d;
numerator = v0.^2 + acc_ratio.*ve.^2 + 2.*a.*(pe-p0);
denom = 1 + acc_ratio;
% tmp = max(0,.5*v0.^2+.5*ve.^2+a.*(pe-p0));
tmp = max(0,numerator./denom);
v = sqrt(tmp); 

%determine maximum velocity while satisfying velocity limit
v1 = direction.*min(v,vm);

%determine extra time needed at maximum speed (only needed if maximum speed is exceeded)
% tmax = max(0,(v.^2-vm.^2)./dm./vm);
tmax = max(0,(v.^2-vm.^2)./2./dm./vm + (v.^2-vm.^2)./2./am./vm);

%given the case that the intial velocity is higher than the maximum velocity
sel = abs(v0)>abs(vm);
if nnz(sel)
    tmax(sel) = (pe(sel)-segment.p(sel))./vm(sel).*direction(sel);
end

%time to brake!
sel = abs(pe-segment.p)<1e-8;
tmax(sel) = 0;
v1(sel) = v0(sel);
end

function segment = move_at_constant_vel(segment, p0, v0, t0, dt)

segment.dt = max(0,dt);
segment.a = zeros(size(dt));
segment.t = t0+segment.dt;
segment.p = p0+v0.*segment.dt;
segment.v = v0;
end

function segment = move_to_vel(segment, p0, v0, t0, ve, am, dm)

if -functions.sign(ve-v0)>=0
    if norm(am(1:2))>norm(dm(1:2))
        acc = dm;
    else
        acc = am;
    end
else
    if norm(am(1:2))>norm(dm(1:2))
        acc = am;
    else
        acc = dm;
    end
end 
segment.dt = abs(ve-v0)./acc;
segment.a = functions.sign(ve-v0).*acc;
segment.t = t0+segment.dt;
segment.p = p0+v0.*segment.dt+.5*segment.a.*segment.dt.^2;
segment.v = ve;
end

function traject = traj1(traject, segment, Ts)

%equidistant time vector - with a different deceleration/acceleration this
%is no longer equidistant (most likely)
n = 1;
nvec = 1;
t = nvec*Ts;

%preparations
ndof = length(segment(1).p);
nseg = length(segment);

%get time response for all segments
[P,V,A,tseg] = traj_segment(segment, t);

%determine active segment at each time instance
segment_id = sum(reshape(tseg,[n ndof nseg]),3);

%determine correct segment sample selection
ind = bsxfun(@plus, nvec, (0:ndof-1)*n); %initialize first segment
ind = ind + segment_id*n*ndof; %change if in other segments

%set output
traject.t(nvec) = t;
traject.p(nvec,:) = P(ind);
traject.v(nvec,:) = V(ind);
traject.a(nvec,:) = A(ind);
traject.segment_id(nvec,:) = segment_id;
end

function [P, V, A, tseg] = traj_segment(segment, time)

%t is the end time of the segment
%v is the velocity at time t
%a is the acceleration in the interval [t-Ts t]
%p is the position at time t

[t,p,v,a] = combine_segment_data(segment);

%aux variables
t_rel = bsxfun(@minus, time, t);
vt = bsxfun(@times, v, t_rel);
at = bsxfun(@times, a, t_rel);

%compute setpoints
P = bsxfun(@plus, p, vt+.5*at.*t_rel); 
V = bsxfun(@plus, v, at);
A = repmat(a,length(time),1);

tseg = bsxfun(@gt, time, t);
end

function [t,p,v,a] = combine_segment_data(segment)

nseg = length(segment);
ndim = length(segment(1).p);
t = zeros(1,nseg*ndim);
p = t;
v = t;
a = t;
ind = 1:ndim;
for i=1:nseg
    t(ind) = segment(i).t;
    p(ind) = segment(i).p;
    v(ind) = segment(i).v;
    a(ind) = segment(i).a;
    ind = ind+ndim;
end
end


function [segment, vmax, amax, dmax] = determine_setpoint_limits(d, p0, v0, pe, ve, segment)

angle_diff = mod(p0(3)-pe(3)+pi,2*pi)-pi;
sc = d.par.scale_rotate;
large_angle = abs(angle_diff)>d.par.scale_angle;
low_velocity = norm(v0(1:2))<1;

if large_angle && low_velocity
    vmax = [d.par.vmax_move*[sc sc]/sqrt(2) d.par.vmax_rotate];
    amax = [d.par.amax_move*[sc sc]/sqrt(2) d.par.amax_rotate];
    dmax = [d.par.dmax_move*[sc sc]/sqrt(2) d.par.dmax_rotate];
else
    vmax = [d.par.vmax_move*[1 1]/sqrt(2) d.par.vmax_rotate];
    amax = [d.par.amax_move*[1 1]/sqrt(2) d.par.amax_rotate];
    dmax = [d.par.dmax_move*[1 1]/sqrt(2) d.par.dmax_rotate];
end


%clip desired subtarget velocity to maximum velocity
if norm(ve(1:2))>norm(vmax(1:2))
    ve(1:2) = ve(1:2)/norm(ve(1:2))*norm(vmax(1:2));
end


[segment, vmax, amax, dmax] = balance_xy(segment, p0, v0, pe, ve, vmax, amax, dmax);
end

function [segment, vmax, amax, dmax] = balance_xy(segment, p0, v0, pe, ve, vm, am, dm)

vmax_move = norm(vm(1:2));
amax_move = norm(am(1:2));
dmax_move = norm(dm(1:2));

%balance xy-acceleration
if all(abs(pe(1:2)-p0(1:2))>1e-8)
    a = 45;
    stepsize = a/2;
    niter = 12;
    for i=1:niter
        max_downscale = .01;
        A = max(max_downscale,[cosd(a) sind(a)]);
        amax = [amax_move*A am(3)];
        vmax = [vmax_move*A vm(3)];
        dmax = [dmax_move*A dm(3)];
        segment = get_segments(segment, p0, v0, pe, ve, vmax, amax, dmax);

        if segment(3).t(1)<segment(3).t(2)
            a = a+stepsize;
        else
            a = a-stepsize;
        end
        stepsize = stepsize/2;
    end
%     for i=1:niter
%         max_downscale = .01;
%         A = max(max_downscale,[cosd(a) sind(a)]);
%         amax = [amax_move*A am(3)];
%         vmax = [vmax_move*A vm(3)];
%         dmax = [dmax_move*A dm(3)];
%         segment = get_segments(segment, p0, v0, pe, ve, vmax, amax, dmax);
% 
%         if segment(3).t(1)>segment(3).t(2)
%             a = a+stepsize;
%         else
%             a = a-stepsize;
%         end
%         stepsize = stepsize/2;
%     end
else
    amax = am;
    vmax = vm;
    dmax = dm;
end
end