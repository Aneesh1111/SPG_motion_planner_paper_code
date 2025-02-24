function target = adjust_to_penalty_area(d, target)

xpos = .5*d.par.field_penalty_area(1);
ypos = d.par.field_size(2)*.5-d.par.field_penalty_area(2);

is_in_penalty_area = abs(target.p(1))<xpos & abs(target.p(2))>ypos;

if is_in_penalty_area
    if ~d.input.robot.CPPA
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
end
