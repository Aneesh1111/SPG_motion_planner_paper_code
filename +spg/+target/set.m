function d = set(d)

%initialize target output
d.target.p = d.input.robot.target;
d.target.v = d.input.robot.target_vel;
d.target.eta = 0;

%adjust target
d.target = spg.target.adjust_to_field(d, d.target);
d.target = spg.target.adjust_to_penalty_area(d, d.target);
d.target = spg.target.adjust_to_goal_area(d, d.target);
d.target = spg.target.adjust_to_obstacles(d, d.target);
%TODO: maximum move distance with ball
d.target = spg.target.adjust_to_3m_rule(d, d.target);
