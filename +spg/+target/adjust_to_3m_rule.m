function target = adjust_to_3m_rule(d, target)

max_radius = 3.0;
dx = target.p(1) - d.input.robot.cpb_poi_xy(1);
dy = target.p(2) - d.input.robot.cpb_poi_xy(2);
distance_between_cpb_poi_xy_and_target = sqrt(dx^2 + dy^2);
angle_between_cpb_poi_xy_and_target = atan2(dy,dx);

if any(d.input.robot.skillID==1:4) && (distance_between_cpb_poi_xy_and_target > max_radius) && (d.input.robot.human_dribble_flag == 0)
    % clip target to 3m radius    
    clipped_target_x = max_radius*cos(angle_between_cpb_poi_xy_and_target);
    clipped_target_y = max_radius*sin(angle_between_cpb_poi_xy_and_target);
    target.p(1:2) = [clipped_target_x, clipped_target_y];
end

