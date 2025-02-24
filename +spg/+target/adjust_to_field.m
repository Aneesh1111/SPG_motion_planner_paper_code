function target = adjust_to_field(d, target)

if any(d.input.robot.skillID==1:4)
    %check ball-field violation at target
    robot_ball_distance = d.par.robot_radius+d.par.ball_radius; %[m]
    dball = robot_ball_distance*[-sin(target.p(3)) cos(target.p(3))];
    ball_pos_at_target = target.p(1:2)+dball;
    ball_pos_at_target = max(ball_pos_at_target,-d.par.field_size*.5);
    ball_pos_at_target = min(ball_pos_at_target,d.par.field_size*.5);
    target.p(1:2) = ball_pos_at_target-dball;
else
    %clip to field size with some additional margin
    
    %if automatic substitution, make the margin larger
    if d.subtarget.automatic_substitution_flag==1
        target.p(1:2) = max(target.p(1:2),-d.par.field_size*.5-d.par.field_border_margin-d.par.technical_area_width);
        target.p(1:2) = min(target.p(1:2),d.par.field_size*.5+d.par.field_border_margin+d.par.technical_area_width);
    else
        target.p(1:2) = max(target.p(1:2),-d.par.field_size*.5-d.par.field_border_margin);
        target.p(1:2) = min(target.p(1:2),d.par.field_size*.5+d.par.field_border_margin);
    end
end
