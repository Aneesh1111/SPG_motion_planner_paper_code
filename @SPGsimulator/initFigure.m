% function h = initFigure(s, d)
% 
% %data
% h.shape_robot = [[0 .25 -.25 0 0]' [-.28 .17 .17 -.28 .35]'];
% h.shape_collision = [sin(s.phi) cos(s.phi)]*d.par.robot_radius;
% h.shape_obstacle = [sin(s.phi) cos(s.phi)]*d.par.robot_radius;
% 
% %figure
% h.fig = figure('un','n','pos',[1.0453    0.0389    0.4802    0.8602]); %[.52 .04 .48 .86]); [1.0453    0.0389    0.4802    0.8602]
% 
% %field
% plot([-1 -1 1 1 -1 -1 1 1]*.5*d.par.field_size(1),[0 1 1 0 0 -1 -1 0]*.5*d.par.field_size(2),'w','linewidth',4)
% hold on
% plot(cos(s.phi)*d.par.field_circle_radius,sin(s.phi)*d.par.field_circle_radius,'w','linewidth',4)
% plot([-1 -1 1 1]*.5*d.par.field_penalty_area(1),d.par.field_size(2)*.5-[0 1 1 0]*d.par.field_penalty_area(2),'w','linewidth',4)
% plot([-1 -1 1 1]*.5*d.par.field_penalty_area(1),-d.par.field_size(2)*.5+[0 1 1 0]*d.par.field_penalty_area(2),'w','linewidth',4)
% 
% %illegal driving zone
% h.SubtargetAvoidPolygon = patch(nan, nan, nan, [.8 0 0], 'facealpha', .6);
% 
% %collision
% h.collision = patch(nan,nan,nan,[.2 .2 .2]);
% 
% %obstacles
% n = zeros(length(s.phi),d.par.nobstacles);
% h.obstacles = patch(n,n,n,[.5 .5 .5]);
% 
% %traject
% h.traj_target = plot(nan,'m.-');
% h.traj_subtarget = plot(nan,'b.-');
% h.traj_obs = plot(nan,'k.-');
% 
% %other traj's
% h.traj_subtarget_array = plot(nan,'r.-');
% %other subtargets
% h.subtarget_array = patch(nan,nan,nan,[0 0 1]);
% 
% %subtarget
% h.subtarget = patch(nan,nan,nan,[0 0 1]);
% 
% %target shifted
% h.target_shifted = patch(nan,nan,nan,[0 1 .7]);
% 
% %target
% h.target = patch(nan,nan,nan,[0 1 0]);
% 
% %robot
% h.setpoint = patch(nan,nan,nan,[0 .7 1]);
% 
% %buttons
% H = 0.05;
% h.restart = uicontrol('style','pushbutton','un','n','pos',[0 0 .2 H],'string','restart','callback',@(~,~)s.callback("restart"));
% h.pause = uicontrol('style','togglebutton','un','n','pos',[.2 0 .2 H],'string','pause/continue','callback',@(~,~)s.callback("pause"));
% h.step = uicontrol('style','togglebutton','un','n','pos',[.4 0 .2 H],'string','step','callback',@(~,~)s.callback("step"));
% h.quit = uicontrol('style','pushbutton','un','n','pos',[.6 0 .2 H],'string','quit','callback',@(~,~)s.callback("quit"));
% % h.restart_seed_minus = uicontrol('style','pushbutton','un','n','pos',[.4 0 .1 H],'string','<','callback',@(~,~)s.callback("random_seed_minus"));
% % h.data_seed = uicontrol('style','edit','un','n','pos',[.5 0 .2 H],'string',num2str(s.data_seed),'callback',@(~,~)s.callback("set_seed"));
% % h.restart_seed_plus = uicontrol('style','pushbutton','un','n','pos',[.7 0 .1 H],'string','>','callback',@(~,~)s.callback("random_seed_plus"));
% % h.run_simulink = uicontrol('style','pushbutton','un','n','pos',[.8 0 .2 H],'string','run simulink','callback',@(~,~)s.callback("run_simulink"));
% h.action = "idle";
% 
% %axis
% axis image
% axis([-d.par.field_size(1)*.5-1 d.par.field_size(1)*.5+1 -d.par.field_size(2)*.5-1 d.par.field_size(2)*.5+1])
% xlabel('x [m]')
% ylabel('y [m]')
% set(gca,'color',[.25 .62 .33]) % [.88 .88 .88]
% 
% %info
% axes('un','n','pos',[0 .6 .2 .4])
% h.info = text(0,0,'','horizontalalignment','left','verticalalignment','bottom','fontsize',8);
% axis([0 1 0 1])
% axis off

function h = initFigure(s, d)

%data
h.shape_robot = [[0 .25 -.25 0 0]' [-.28 .17 .17 -.28 .35]'];
h.shape_collision = [sin(s.phi) cos(s.phi)]*d.par.robot_radius;
h.shape_obstacle = [sin(s.phi) cos(s.phi)]*d.par.robot_radius;

%figure
h.fig = figure('un','n','pos',[1.0453    0.0389    0.4802    0.8602]); %[.52 .04 .48 .86]); [1.0453    0.0389    0.4802    0.8602]

%field
plot([-1 -1 1 1 -1 -1 1 1]*.5*d.par.field_size(1),[0 1 1 0 0 -1 -1 0]*.5*d.par.field_size(2),'w','linewidth',4)
hold on
plot(cos(s.phi)*d.par.field_circle_radius,sin(s.phi)*d.par.field_circle_radius,'w','linewidth',4)
plot([-1 -1 1 1]*.5*d.par.field_penalty_area(1),d.par.field_size(2)*.5-[0 1 1 0]*d.par.field_penalty_area(2),'w','linewidth',4)
plot([-1 -1 1 1]*.5*d.par.field_penalty_area(1),-d.par.field_size(2)*.5+[0 1 1 0]*d.par.field_penalty_area(2),'w','linewidth',4)

%illegal driving zone
h.SubtargetAvoidPolygon = patch(nan, nan, nan, [.8 0 0], 'facealpha', .6);

%collision
% h.collision = patch(nan,nan,nan,[.2 .2 .2]);
h.collision = patch(nan,nan,nan,[.9 .7 .1]);

%obstacles
n = zeros(length(s.phi),d.par.nobstacles);
h.obstacles = patch(n,n,n,[.5 .5 .5]);

%traject
if 1
    h.traj_trials_ok = plot(nan, '.-','color', [.3 .7 .9]);
    h.traj_trials_nok = plot(nan, '.-','color', [1 .1 .4]);
    global traj_trials
    traj_trials.item = [];
end
h.traj_target = plot(nan,'m.-');
h.traj_subtarget = plot(nan,'c.-','LineWidth',3.0,'MarkerSize',20);
h.traj_obs = plot(nan,'k.-');

%subtarget
h.subtarget = patch(nan,nan,nan,[0 0 1]);

%target shifted
h.target_shifted = patch(nan,nan,nan,[0 1 .7]);

%target
h.target = patch(nan,nan,nan,[0 1 0]);

%robot
% h.setpoint = patch(nan,nan,nan,[0 1 0]);
h.setpoint = plot(nan,'g.','MarkerSize',30);

%buttons
H = 0.05;
h.restart = uicontrol('style','pushbutton','un','n','pos',[0 0 .2 H],'string','restart','callback',@(~,~)s.callback("restart"));
h.pause = uicontrol('style','togglebutton','un','n','pos',[.2 0 .2 H],'string','pause/continue','callback',@(~,~)s.callback("pause"));
h.step = uicontrol('style','togglebutton','un','n','pos',[.4 0 .2 H],'string','step','callback',@(~,~)s.callback("step"));
h.quit = uicontrol('style','pushbutton','un','n','pos',[.6 0 .2 H],'string','quit','callback',@(~,~)s.callback("quit"));
% h.restart_seed_minus = uicontrol('style','pushbutton','un','n','pos',[.4 0 .1 H],'string','<','callback',@(~,~)s.callback("random_seed_minus"));
% h.data_seed = uicontrol('style','edit','un','n','pos',[.5 0 .2 H],'string',num2str(s.data_seed),'callback',@(~,~)s.callback("set_seed"));
% h.restart_seed_plus = uicontrol('style','pushbutton','un','n','pos',[.7 0 .1 H],'string','>','callback',@(~,~)s.callback("random_seed_plus"));
% h.run_simulink = uicontrol('style','pushbutton','un','n','pos',[.8 0 .2 H],'string','run simulink','callback',@(~,~)s.callback("run_simulink"));
h.action = "idle";

%axis
axis image
axis([-d.par.field_size(1)*.5-1 d.par.field_size(1)*.5+1 -d.par.field_size(2)*.5-1 d.par.field_size(2)*.5+1])
xlabel('x [m]', 'Interpreter', 'latex')
ylabel('y [m]', 'Interpreter', 'latex')
% set(gca,'color',[.25 .62 .33]) % [.88 .88 .88]
set(gca,'color',[.88 .88 .88],'TickLabelInterpreter','latex')

%info
axes('un','n','pos',[0 .6 .2 .4])
h.info = text(0,0,'','horizontalalignment','left','verticalalignment','bottom','fontsize',8);
axis([0 1 0 1])
axis off