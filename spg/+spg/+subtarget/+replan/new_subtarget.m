function best = new_subtarget(d, subtarget)

%% initialize best result

best = d.subtarget;
% best.p = d.setpoint.p;
% best.violation_count = 1e6;
% best.age = 0;
% best.collisionfree = false;
phi = subtarget.p(3);

%% search beside obstacles

best = spg.subtarget.replan.search.beside_obstacle(d, best);

%% search near target, only without ball - because we have a 3m radius rule when we have the ball

search_point = [d.target.p(1:2) phi];
search_distance = max(d.par.search_distance,norm(d.setpoint.p(1:2)-d.target.p(1:2)));
best = spg.subtarget.replan.search.random(d, best, search_point, search_distance, defending_opp_with_or_without_ball);

%% search near current position

search_point = [d.setpoint.p(1:2) phi];
best = spg.subtarget.replan.search.random(d, best, search_point, d.par.search_distance, defending_opp_with_or_without_ball);
