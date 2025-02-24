function target = adjust_to_goal_area(d, target)

xpos = .5*d.par.field_goal_area(1);
ypos = d.par.field_size(2)*.5-d.par.field_goal_area(2);

is_in_goal_area = abs(target.p(1))<xpos & abs(target.p(2))>ypos;

if is_in_goal_area
    distance = [xpos-abs(target.p(1)) abs(target.p(2))-ypos];
    
    if distance(1)<distance(2) %clip to side
        target.p(1) = xpos*functions.sign(target.p(1));
        if abs(target.p(2))>d.par.field_size(2)*.5
            target.p(2) = d.par.field_size(2)*.5*functions.sign(target.p(2));
        end
    else %clip to front
        target.p(2) = ypos*functions.sign(target.p(2));
    end
end
