function best = new_subtarget(d, subtarget)

%% initialize best result

best = d.subtarget;
% best.p = d.setpoint.p;
% best.violation_count = 1e6;
% best.age = 0;
% best.collisionfree = false;
phi = subtarget.p(3);

%% if defending search for subtarget in line2goal of opposing robots

ROBOT_DEFENDING_OPP_WITHOUT_BALL = 0; % ANEESH - FIX THIS - Need to implement this in strategy 
% for robot defending the opponent without the ball
if d.input.robot.skillID==0 && d.input.robot.CPBteam<0.5 && ROBOT_DEFENDING_OPP_WITHOUT_BALL % TODO
    search_point = [d.target.p(1:2) phi];
    search_distance = 3;
    defending_opp_with_or_without_ball = 1;  % 1 means we are defending the opponent without the ball so we should search THAT region
    best = spg.subtarget.replan.search.random(d, best, search_point, search_distance, defending_opp_with_or_without_ball);
end
defending_opp_with_or_without_ball = 0;

%% search beside obstacles

best = spg.subtarget.replan.search.beside_obstacle(d, best);

%% search near target, only without ball - because we have a 3m radius rule when we have the ball

if (d.input.robot.skillID==0 || d.input.robot.skillID==1 || d.input.robot.skillID==5) || d.input.robot.human_dribble_flag==1
    search_point = [d.target.p(1:2) phi];
    search_distance = max(d.par.search_distance,norm(d.setpoint.p(1:2)-d.target.p(1:2)));
    best = spg.subtarget.replan.search.random(d, best, search_point, search_distance, defending_opp_with_or_without_ball);
end

%% search near current position

search_point = [d.setpoint.p(1:2) phi];
best = spg.subtarget.replan.search.random(d, best, search_point, d.par.search_distance, defending_opp_with_or_without_ball);
