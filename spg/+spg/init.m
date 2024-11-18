function d = init(p_initial, v_initial, nobstacles, npredict, p_initial_ball, v_initial_ball, nintercept_positions)

% SKILLID_MOVE = 0,   /* Move (without ball) */
% SKILLID_DRIBBLE,	/* Dribble to target (with ball) */
% SKILLID_AIM,        /* Aim at target */
% SKILLID_KICK,       /* Kick (further specified in shot types enum) */
% SKILLID_SHIELD,     /* Shield ball from closest opponent */
% SKILLID_INTERCEPT	/* Intercept a ball */

% /* trajectory smooth stop types */
% typedef enum{
% 	SMOOTHSTOP_NONE,
% 	SMOOTHSTOP_SMOOTH_XYPHI,   /* Smooth stop xyphi */
%     SMOOTHSTOP_SMOOTH_XY,      /* Smooth stop for xy only, phi maintains target */
%     SMOOTHSTOP_SMOOTH_PHI      /* Smooth stop for phi only, xy maintains target */
% } TrajectorySmoothStop_t, *pTrajectorySmoothStop_t;

%parameters
d.par = struct(...
    'nobstacles',nobstacles,... %[#]     --------------------------[algorithm]
    'Ts',.02,... %[s]
    'field_size',[8 12],... %[m] --------------------------[field geometry]
    'field_circle_radius',1.5,... %[m]
    'field_penalty_area',[4 1.5],... %[m]
    'field_goal_area',[2 .4],... %[m]
    'field_border_margin',.5,... %[m]
    'goalwidth',2.4,... % [m]
    'technical_area_width',0.9,... % [m]
    'Ts_predict',.1,... %[s]     --------------------------[subtarget]
    'npredict',npredict,...
    'robot_radius',.25,...  %[m]
    'ball_radius',.11,... %[m]
    'obstacle_vel_gain',.95,... %[gain]
    'nattempts_replan',10,... %[#]
    'search_distance',6,... %[m]
    'replan_uphill_distance',2,... %[m]
    'margin_replan',.1,... %[s] replan margin scaling with velocity
    'vmax_move',4,... %[m/s]    --------------------------[setpoint]
    'vmax_rotate',13,... %[rad/s]
    'amax_move',1.8,... %[m/s2]
    'amax_quickstop',3.5,... %[m/s2]
    'amax_rotate',13,... %[rad/s2]
    'scale_rotate',.3,... %disabled for now
    'scale_angle',40/180*pi,... %[rad]
    'dmax_move',1.8,... %[m/s2]
    'dmax_rotate',13,... %[m/s2]
    'nintercept_positions',nintercept_positions);

%inputs
d.input.obstacles = struct('p',zeros(nobstacles,2),'v',zeros(nobstacles,2),'r',zeros(nobstacles,1),'active',false(nobstacles,1));
d.input.robot = struct('p',p_initial,'v',v_initial,'target',p_initial,'skillID',0,'CPPA',0,'CPBteam',0,'reset_trigger',0,'quickstop_trigger',0,'cpb_poi_xy',zeros(1,2),'target_vel',[0 0 0],'IMU_orientation',[0.0 0.0 0.0],'human_dribble_flag',0,'dist2ball_vs_opp',1.0);
d.input.ball = struct('p',p_initial_ball,'v',v_initial_ball);
d.input.SubtargetAvoidPolygon = struct('polygon',zeros(2,4),'valid',false);

%state
nsegments = 4;
d.aux = struct('segment',repmat(struct('dt',zeros(1,3),'t',zeros(1,3),'p',zeros(1,3),'v',zeros(1,3),'a',zeros(1,3)),1,nsegments));
d.traj = struct('p',zeros(npredict,3),'v',zeros(npredict,3),'a',zeros(npredict,3),'t',zeros(npredict,1),'segment_id',zeros(npredict,3)); %defines prediction horizon

%outputs
d.setpoint = struct('p',p_initial,'v',[0 0 0],'a',[0 0 0]);
d.subtarget = struct('p',p_initial,'v',[0 0 0],'vmax',[1 1 1],'amax',[1 1 1],'dmax',[1 1 1],...
    'action',0,'collisionfree',false,'target',[0 0 0],'eta',0,'segment_id',zeros(1,3),'age',1e10,...
    'violation_count',0,...
    'segment',repmat(struct('dt',zeros(1,3),'t',zeros(1,3),'p',zeros(1,3),'v',zeros(1,3),'a',zeros(1,3)),1,nsegments),...
    'automatic_substitution_flag',0);
d.target = struct('p',p_initial,'v',[0 0 0],'eta',0);