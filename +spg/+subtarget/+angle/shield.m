function angle = shield(d)

%get distance to nearest obstacle
obstacles = d.input.obstacles.p(d.input.obstacles.active,1:2);
obsdist2 = sum(bsxfun(@minus,obstacles,d.setpoint.p(1:2)).^2,2);
[mindist2,nearest_obstacle] = min(obsdist2);

%point away from nearest obstacle
v = -(obstacles(nearest_obstacle,1:2)-d.setpoint.p(1:2));
v = v/norm(v);

%determine shielding fraction
%     frac = exp(-.3*mindist2);
x_end = 2;
x_start = 1;
frac = max(0,min(1,(x_end-sqrt(mindist2))/(x_end-x_start)));

%if moving, don't get bothered by obstacles far away
if nnz(d.setpoint.v(1:2))
    v_robot = d.setpoint.v(1:2);
    v_robot = v_robot/norm(v_robot);
    v = frac*v+(1-frac)*v_robot;
else
    robot_direction = [-sin(d.subtarget.p(3)) cos(d.subtarget.p(3))];
    v = frac*v+(1-frac)*robot_direction;
end

angle = atan2(-v(1),v(2));
