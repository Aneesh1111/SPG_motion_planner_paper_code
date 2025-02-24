function ok = not_touching_obstacle(p_robot, p_obstacles, obstacle_margin)

obsdist2 = sum(bsxfun(@minus, p_obstacles, p_robot).^2,2);
ok = all(obsdist2>obstacle_margin.^2);
