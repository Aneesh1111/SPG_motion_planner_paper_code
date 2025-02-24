close all
clear
clc

%load data
[data, t] = sim.simulink_data_to_struct;
Ts = data(1).par.Ts;

%pos/vel/acc
p = arrayfun(@(d)d.traj.p(1,:),data,'uniformoutput',false);
p = vertcat(p{:});
v = filter([1 -1],Ts,p-p(1,:));
a = filter([1 -1],Ts,v-v(1,:));
c = arrayfun(@(d)nnz(d.input.obstacles.active), data);
skillID = arrayfun(@(d)d.input.robot.skillID, data);
target = arrayfun(@(d)d.input.robot.target, data,'uniformoutput',false);
target = [0 2 0];%vertcat(target{:});
target_vel = arrayfun(@(d)d.input.robot.target_vel, data,'uniformoutput',false);
target_vel = vertcat(target_vel{:});
subtarget_amax = arrayfun(@(d)d.subtarget.amax, data,'uniformoutput',false);
subtarget_amax = vertcat(subtarget_amax{:});
CPPA = arrayfun(@(d)d.input.robot.CPPA, data);
obstacles_pos = arrayfun(@(d)d.input.obstacles.p, data,'uniformoutput',false);
obstacles_pos = cat(3, obstacles_pos{:});
obstacles_vel = arrayfun(@(d)d.input.obstacles.v, data,'uniformoutput',false);
obstacles_vel = cat(3, obstacles_vel{:});
obstacles_active = arrayfun(@(d)d.input.obstacles.active, data,'uniformoutput',false);
obstacles_active = cat(3, obstacles_active{:});

%% last state analysis

tCheck = t(end);
[~, ind] = min(abs(t-tCheck));
d = data(end);
info = cell2table({'pose', d.traj.p(1,:); 'target', d.target.p; 'target_vel', d.target.v; 'subtarget', d.subtarget.p},'VariableNames',{'item','pose'});
disp(info)

spg.next_sample(d);

s = SPGsimulator(d);
setInputData(s, t, p, v, target, skillID, CPPA, obstacles_pos, obstacles_vel, obstacles_active)

%% show results

figure
tiledlayout('flow','padding','tight','tilespacing','tight')

nexttile
plot(t, p)
title('pose')

nexttile
plot(t, v)
title('vel')
ylim([-1 4])

nexttile
plot(t, a)
title('acc')
ylim([-4 4])

nexttile
plot(t, skillID)
title('skillID')

nexttile
plot(t, sqrt(sum(subtarget_amax(:,1:2).^2,2)))
title('|subtarget_amax xy|')

% nexttile
% plot(t, c)
% title('obstacle count')
% 
xlabel('time [s]')

return

%%

tDuration = 10;
tStart = t(end)-tDuration;
[~,iStart] = min(abs(t-tStart));
[~,iStop] = min(abs(t-tStart-tDuration));
for i=iStart:iStop
    d = data(i);
    update(s, d, i);
end

